#include <CLI/CLI.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

#include "solver/glkh.h"
#include "solver/tarel_graph.h"

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
  CLI::App app{"Benchmark GLKH-based GTSP heuristic solver"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  GlkhOptions options;
  app.add_option(
      "--max-trials", options.max_trials, "LKH MAX_TRIALS (<=0: LKH default)"
  );
  app.add_option("--runs", options.runs, "Number of independent LKH runs");
  app.add_option("--seed", options.seed, "Random seed");
  double time_limit = 0;
  app.add_option("--time-limit", time_limit, "TIME_LIMIT per run in seconds");
  app.add_option(
      "--max-times-per-stop",
      options.max_times_per_stop,
      "Thin time vertices per stop (0: keep all)"
  );
  bool quiet = false;
  app.add_flag("--quiet", quiet, "Suppress GLKH output");

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
  std::optional<GlkhSolution> result =
      SolveGtspWithGlkh(state, options, quiet ? nullptr : &std::cerr);
  auto end = std::chrono::steady_clock::now();

  int total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  if (!result.has_value()) {
    std::cout << "\nNo feasible tour found.\n";
    std::cout << "\nTotal time: " << FormatDuration(total_ms) << "\n";
    return 1;
  }

  std::cout << "\nAGTSP vertices: " << result->num_vertices
            << ", clusters: " << result->num_clusters << "\n";
  std::cout << "GTSP tour value: " << TimeSinceServiceStart{result->gtsp_value}
            << "\n";
  std::cout << "Best duration: " << TimeSinceServiceStart{result->best_val}
            << "\n";
  if (!result->best_tour.empty()) {
    std::cout << "Tour (" << result->best_tour.size() << " stops):\n";
    for (StopId stop : result->best_tour) {
      std::cout << "  " << state.StopName(stop) << "\n";
    }
  }

  std::cout << "\nTotal time: " << FormatDuration(total_ms) << "\n";

  return 0;
}
