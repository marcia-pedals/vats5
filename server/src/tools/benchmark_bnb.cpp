#include <CLI/CLI.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "solver/branch_and_bound.h"
#include "solver/tarel_graph.h"

using namespace vats5;

int main(int argc, char* argv[]) {
  CLI::App app{"Benchmark Branch and Bound solver"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  int max_iter = -1;
  app.add_option(
         "--max-iter", max_iter, "Maximum iterations (-1 for unlimited)"
  )
      ->default_val(-1);

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
  auto result = BranchAndBoundSolve(state, &std::cerr, std::nullopt, max_iter);
  auto end = std::chrono::steady_clock::now();

  auto duration_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  std::cout << "\n";
  std::cout << "Optimal value: " << result.best_ub << " seconds ("
            << result.best_ub / 60 << " min)\n";
  std::cout << "Paths: " << result.best_paths.size() << "\n";
  for (size_t i = 0; i < result.best_paths.size(); ++i) {
    const auto& path = result.best_paths[i];
    std::cout << "  Path " << i << ": ";
    path.VisitAllStops([&](StopId stop) { std::cout << stop.v << " "; });
    std::cout << "(" << path.DurationSeconds() << "s)\n";
  }
  std::cout << "Time: " << duration_ms << " ms\n";

  return 0;
}
