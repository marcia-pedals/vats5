#include <CLI/CLI.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

#include "solver/tarel_graph.h"
#include "solver/two_opt.h"

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

  CLI11_PARSE(app, argc, argv);
  if (time_limit > 0) {
    options.time_limit_seconds = time_limit;
  }

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
  TwoOptResult result =
      TwoOptSolve(state, options, quiet ? nullptr : &std::cerr);
  auto end = std::chrono::steady_clock::now();

  int total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  std::cout << "\nRestarts: " << result.restarts_completed
            << ", evaluations: " << result.evaluations << "\n";
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
    std::cout << "Time per restart: mean " << std::fixed
              << std::setprecision(3) << (mean * 1000) << " ms";
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
