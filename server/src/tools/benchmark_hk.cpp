#include <CLI/CLI.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <string>

#include "solver/held_karp_dp.h"
#include "solver/tarel_graph.h"
#include "tools/benchmark_events.h"

using namespace vats5;

std::string FormatDuration(int ms) {
  if (ms < 1000) {
    return std::to_string(ms) + " ms";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1) << (ms / 1000.0) << " s";
  return ss.str();
}

int main(int argc, char* argv[]) {
  CLI::App app{"Benchmark Held-Karp DP solver"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  std::optional<std::string> events_out;
  app.add_option(
      "--events-out",
      events_out,
      "Path to write bound events as JSON Lines (see benchmark_events.h)"
  );

  CLI11_PARSE(app, argc, argv);

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

  auto start = std::chrono::steady_clock::now();
  events.Start();
  auto result = HeldKarpDPSolve(state, 0, &std::cerr);
  auto end = std::chrono::steady_clock::now();

  // Held-Karp is exact, so its answer is both bounds at once, and only known
  // once the whole sweep is done.
  if (result.best_val < std::numeric_limits<int>::max()) {
    events.LowerBound(result.best_val);
    events.UpperBound(result.best_val);
  }
  events.Converged();

  int total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  std::cout << "\nBest duration: " << TimeSinceServiceStart{result.best_val}
            << "\n";
  if (!result.best_tour.empty()) {
    std::cout << "Tour (" << result.best_tour.size() << " stops):\n";
    for (StopId stop : result.best_tour) {
      std::cout << "  " << state.StopName(stop) << "\n";
    }
  }

  std::cout << "\nTotal time: " << FormatDuration(total_ms) << "\n";

  return 0;
}
