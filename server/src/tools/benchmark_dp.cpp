#include <CLI/CLI.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "solver/subset_dp.h"
#include "solver/tarel_graph.h"

using namespace vats5;

int main(int argc, char* argv[]) {
  CLI::App app{"Benchmark subset DP exact solver"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  CLI11_PARSE(app, argc, argv);

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
  std::optional<SubsetDpResult> result = SubsetDpSolve(state, &std::cerr);
  auto end = std::chrono::steady_clock::now();

  int total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  if (!result.has_value()) {
    std::cout << "No feasible tour found.\n";
  } else {
    std::cout << "Best duration: "
              << TimeSinceServiceStart{result->best_duration_seconds} << " ("
              << result->start_time << " -> " << result->end_time << ")\n";
    std::vector<StopId> tour;
    for (StopId stop : result->stop_sequence) {
      ExpandStop(stop, state.original_edges, tour);
    }
    std::cout << "Tour (" << tour.size() << " stops):\n";
    for (StopId stop : tour) {
      std::cout << "  " << state.StopName(stop) << "\n";
    }
  }

  std::cout << "\nTotal time: " << total_ms << " ms\n";

  return 0;
}
