#include <CLI/CLI.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <limits>
#include <numeric>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "solver/gtsp.h"
#include "solver/tarel_graph.h"

using namespace vats5;

struct StepCounts {
  int flex = 0;
  int scheduled = 0;
};

void PrintCounts(const StepCounts& counts) {
  std::cout << "  Flex steps (excluding START/END): " << counts.flex << "\n";
  std::cout << "  Scheduled steps: " << counts.scheduled << "\n";
}

void PrintGtspStats(const StepsAdjacencyList& list, ProblemBoundary boundary) {
  auto start = std::chrono::steady_clock::now();
  Gtsp gtsp = BuildGtsp(list, boundary);
  auto end = std::chrono::steady_clock::now();
  int ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  int min_time = std::numeric_limits<int>::max();
  int max_time = std::numeric_limits<int>::min();
  for (size_t v = 2; v < gtsp.vertices.size(); ++v) {
    min_time = std::min(min_time, gtsp.vertices[v].time.seconds);
    max_time = std::max(max_time, gtsp.vertices[v].time.seconds);
  }
  int time_gcd = 0;
  for (size_t v = 2; v < gtsp.vertices.size(); ++v) {
    time_gcd = std::gcd(time_gcd, gtsp.vertices[v].time.seconds - min_time);
  }
  std::cout << "  GTSP (" << ms << " ms): " << gtsp.vertices.size()
            << " vertices, " << gtsp.edges.size() << " edges\n";
  std::cout << "  GTSP time range: " << TimeSinceServiceStart{min_time}
            << " to " << TimeSinceServiceStart{max_time}
            << ", time gcd " << time_gcd << " s\n";
}

int main(int argc, char* argv[]) {
  CLI::App app{"Print statistics about a ProblemState"};

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

  const StopId start = state.boundary.start;
  const StopId end = state.boundary.end;
  auto touches_boundary = [&](StopId origin, StopId destination) {
    return origin == start || origin == end || destination == start ||
           destination == end;
  };

  StepCounts minimal_counts;
  for (int stop_v = 0; stop_v < state.minimal.NumStops(); ++stop_v) {
    StopId origin{stop_v};
    for (const StepGroup& group : state.minimal.GetGroups(origin)) {
      if (group.flex_step.has_value() &&
          !touches_boundary(origin, group.destination_stop)) {
        ++minimal_counts.flex;
      }
      minimal_counts.scheduled += group.steps_end - group.steps_start;
    }
  }

  std::cout << "Loaded problem state from: " << input_path << "\n";
  std::cout << "Stops: " << state.minimal.NumStops() << "\n";
  std::cout << "Required stops: " << state.required.size() << "\n";
  std::cout << "\nMinimal graph:\n";
  PrintCounts(minimal_counts);
  PrintGtspStats(state.minimal, state.boundary);

  auto completion_start = std::chrono::steady_clock::now();
  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();
  auto completion_end = std::chrono::steady_clock::now();
  int completion_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          completion_end - completion_start
  )
                          .count();

  StepCounts completed_counts;
  for (const Step& step : completed.AllMergedSteps()) {
    if (step.is_flex) {
      if (!touches_boundary(step.origin.stop, step.destination.stop)) {
        ++completed_counts.flex;
      }
    } else {
      ++completed_counts.scheduled;
    }
  }

  std::cout << "\nDense completion on required stops (" << completion_ms
            << " ms):\n";
  PrintCounts(completed_counts);
  PrintGtspStats(
      MakeAdjacencyList(completed.AllMergedSteps()), state.boundary
  );

  return 0;
}
