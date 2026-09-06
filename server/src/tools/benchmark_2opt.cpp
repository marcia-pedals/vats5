#include <CLI/CLI.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>

#include "solver/search_event.h"
#include "solver/tarel_graph.h"
#include "solver/two_opt.h"
#include "tools/benchmark_events.h"

using namespace vats5;

template <class... Ts>
struct Overloaded : Ts... {
  using Ts::operator()...;
};

std::string FormatDuration(int ms) {
  if (ms < 1000) {
    return std::to_string(ms) + " ms";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1) << (ms / 1000.0) << " s";
  return ss.str();
}

int main(int argc, char* argv[]) {
  CLI::App app{"Benchmark 2-opt local search solver"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  TwoOptOptions options;
  app.add_option("--restarts", options.restarts, "Number of restarts");
  app.add_option("--seed", options.seed, "Random seed");
  double time_limit = 0;
  app.add_option("--time-limit", time_limit, "Time limit in seconds");
  bool quiet = false;
  app.add_flag("--quiet", quiet, "Suppress per-restart progress output");
  int known_lb = 0;
  app.add_option(
      "--known-lb",
      known_lb,
      "A proven lower bound on the optimal duration in seconds; the search "
      "stops as soon as it finds a tour achieving it"
  );

  std::optional<std::string> events_out;
  app.add_option(
      "--events-out",
      events_out,
      "Path to write bound events as JSON Lines (see benchmark_events.h)"
  );

  CLI11_PARSE(app, argc, argv);
  if (time_limit > 0) {
    options.time_limit_seconds = time_limit;
  }

  BenchmarkEventLog events(events_out);

  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << "\n";
    return 1;
  }

  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState state = j.get<ProblemState>();
  in.close();

  std::cout << "Loaded problem state from: " << input_path << "\n";
  std::cout << "Stops: " << state.minimal.NumStops() << "\n";
  std::cout << "Required stops: " << state.required.size() << "\n";
  std::cout << "\n";

  auto on_event = [&](const SearchEvent& event) {
    std::visit(
        Overloaded{
            [&](const TarelSolve&) {},
            [&](const NewLowerBound& e) { events.LowerBound(e.lb); },
            [&](const NewUpperBound& e) { events.UpperBound(e.ub); },
        },
        event
    );
  };

  auto start = std::chrono::steady_clock::now();
  events.Start();
  TwoOptResult result = TwoOptSolve(
      state, known_lb, options, quiet ? nullptr : &std::cerr, on_event
  );
  auto end = std::chrono::steady_clock::now();
  // 2-opt is a heuristic: it has converged only if it reached the bound the
  // caller proved no tour can beat.
  if (known_lb > 0 && result.best_val <= known_lb) {
    events.Converged();
  }

  int total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  std::cout << "\nRestarts: " << result.restarts_completed
            << ", evaluations: " << result.evaluations << "\n";
  if (known_lb > 0) {
    std::cout << "Known lb: " << TimeSinceServiceStart{known_lb}
              << (result.best_val <= known_lb ? " (reached)" : " (not reached)")
              << "\n";
  }
  std::cout << "Best duration: " << TimeSinceServiceStart{result.best_val}
            << "\n";
  if (!result.best_tour.empty()) {
    std::cout << "Tour (" << result.best_tour.size() << " stops):\n";
    for (StopId stop : result.best_tour) {
      std::cout << "  " << state.StopName(stop) << "\n";
    }
  }

  std::cout << "\nTotal time: " << FormatDuration(total_ms) << "\n";

  const std::vector<double>& secs = result.restart_seconds;
  if (!secs.empty()) {
    double n = static_cast<double>(secs.size());
    double mean = std::accumulate(secs.begin(), secs.end(), 0.0) / n;
    std::cout << "Time per restart: mean " << std::fixed << std::setprecision(3)
              << (mean * 1000) << " ms";
    if (secs.size() > 1) {
      double sum_sq = 0;
      for (double s : secs) {
        sum_sq += (s - mean) * (s - mean);
      }
      double sd = std::sqrt(sum_sq / (n - 1));
      std::cout << ", sd " << (sd * 1000) << " ms";
    }
    std::cout << "\n";
  }

  return 0;
}
