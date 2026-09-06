#include <CLI/CLI.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "solver/branch_and_bound.h"
#include "solver/search_event.h"
#include "solver/tarel_graph.h"
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
  CLI::App app{"Benchmark Branch and Bound solver"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  int max_iter = -1;
  app.add_option(
         "--max-iter", max_iter, "Maximum iterations (-1 for unlimited)"
  )
      ->default_val(-1);

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

  std::vector<TarelSolve> solves;
  auto on_event = [&](const SearchEvent& event) {
    std::visit(
        Overloaded{
            [&](const TarelSolve& e) { solves.push_back(e); },
            [&](const NewLowerBound& e) { events.LowerBound(e.lb); },
            [&](const NewUpperBound& e) { events.UpperBound(e.ub); },
        },
        event
    );
  };

  auto start = std::chrono::steady_clock::now();
  events.Start();
  auto result = BranchAndBoundSolve(
      state, 0, &std::cerr, std::nullopt, max_iter, on_event
  );
  auto end = std::chrono::steady_clock::now();
  // The search only returns once it has proven its best tour optimal.
  events.Converged();

  int total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  int total_concorde_ms = 0;
  for (const auto& s : solves) {
    total_concorde_ms += s.concorde_ms;
  }
  int non_concorde_ms = total_ms - total_concorde_ms;

  std::cout << "\nBest duration: " << TimeSinceServiceStart{result.best_ub}
            << "\n";
  if (!result.best_paths.empty()) {
    const auto& path = result.best_paths[0];
    std::vector<StopId> tour;
    path.VisitAllStops([&](StopId stop) {
      ExpandStop(stop, result.original_edges, tour);
    });
    std::cout << "Tour (" << tour.size() << " stops):\n";
    for (StopId stop : tour) {
      std::cout << "  " << state.StopName(stop) << "\n";
    }
  }

  std::cout << "\nConcorde solves (" << solves.size() << "):\n";
  for (int i = 0; i < static_cast<int>(solves.size()); ++i) {
    const auto& s = solves[i];
    std::cout << "  #" << (i + 1) << ": " << s.vertex_count << " vertices, "
              << s.edge_count << " edges, " << FormatDuration(s.concorde_ms)
              << (s.feasible ? "" : " (infeasible)") << "\n";
  }

  std::cout << "\nTotal time:       " << FormatDuration(total_ms) << "\n";
  std::cout << "Concorde time:    " << FormatDuration(total_concorde_ms)
            << "\n";
  std::cout << "Non-concorde time: " << FormatDuration(non_concorde_ms) << "\n";

  return 0;
}
