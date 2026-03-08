#include <CLI/CLI.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "solver/branch_and_bound.h"
#include "solver/search_event.h"
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

  std::vector<TarelSolve> solves;
  auto on_event = [&](const SearchEvent& event) {
    std::visit([&](const TarelSolve& e) { solves.push_back(e); }, event);
  };

  auto start = std::chrono::steady_clock::now();
  auto result =
      BranchAndBoundSolve(state, &std::cerr, std::nullopt, max_iter, on_event);
  auto end = std::chrono::steady_clock::now();

  int total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  int total_concorde_ms = 0;
  for (const auto& s : solves) {
    total_concorde_ms += s.concorde_ms;
  }
  int non_concorde_ms = total_ms - total_concorde_ms;

  std::cout << "\nBest duration: "
            << TimeSinceServiceStart{result.best_ub}.ToString() << "\n";
  if (!result.best_paths.empty()) {
    const auto& path = result.best_paths[0];
    std::cout << "Path (" << path.steps.size() << " steps):\n";
    for (const Step& step : path.steps) {
      std::cout << "  " << state.StopName(step.origin.stop) << " ("
                << step.origin.time.ToString() << ") -> "
                << state.StopName(step.destination.stop) << " ("
                << step.destination.time.ToString() << ")\n";
    }
  }

  std::cout << "\nConcorde solves (" << solves.size() << "):\n";
  for (int i = 0; i < static_cast<int>(solves.size()); ++i) {
    const auto& s = solves[i];
    std::cout << "  #" << (i + 1) << ": " << s.vertex_count << " vertices, "
              << s.edge_count << " edges, " << FormatDuration(s.concorde_ms)
              << "\n";
  }

  std::cout << "\nTotal time:       " << FormatDuration(total_ms) << "\n";
  std::cout << "Concorde time:    " << FormatDuration(total_concorde_ms)
            << "\n";
  std::cout << "Non-concorde time: " << FormatDuration(non_concorde_ms) << "\n";

  return 0;
}
