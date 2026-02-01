#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "solver/branch_and_bound2.h"
#include "solver/step_merge.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

int FindBestMatch(
  const std::vector<StopId>& greedy_path,
  const std::vector<StopId>& candidate_path
) {
  if (greedy_path.size() == 0) {
    return 0;
  }

  int candidate_size = candidate_path.size();
  int greedy_size = greedy_path.size();

  // std::cout << "candidate size " << candidate_size << "\n";
  // std::cout << "greedy size " << greedy_size << "\n";

  int best_match_size = -1;
  int best_match_candidate_offset = -1;

  for (int candidate_offset = -(candidate_size - 1); candidate_offset < greedy_size; ++candidate_offset) {
    // std::cout << candidate_offset << "\n";
    int greedy_index_start = std::max(0, candidate_offset);
    int greedy_index_end = std::min(greedy_size, candidate_size + candidate_offset);
    bool matches = true;
    for (int greedy_index = greedy_index_start; greedy_index < greedy_index_end; ++greedy_index) {
      if (greedy_path[greedy_index] != candidate_path[greedy_index - candidate_offset]) {
        matches = false;
        break;
      }
    }
    int match_size = greedy_index_end - greedy_index_start;
    if (matches && match_size > best_match_size) {
      best_match_size = match_size;
      best_match_candidate_offset = candidate_offset;
    }
  }
  return best_match_candidate_offset;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <subpaths.json>" << std::endl;
        return 1;
    }

    std::string path = argv[1];
    std::ifstream file(path);
    if (!file) {
        std::cerr << "Error: Could not open file: " << path << std::endl;
        return 1;
    }

    nlohmann::json j;
    file >> j;

    ProblemState state = j.at("state").get<ProblemState>();
    TspTourResult lb_result = j.at("lb_result").get<TspTourResult>();

    // Recompute completed from minimal and required_stops.
    state.completed = ReduceToMinimalSystemPaths(state.minimal, state.required_stops, true);
    state.completed.adjacent[state.boundary.end].push_back({ZeroPath(state.boundary.end, state.boundary.start)});

    std::vector<Path> subpaths = FindSubpaths(state, lb_result.tour_edges);

    std::cout << "Loaded state with " << state.required_stops.size() << " required stops" << std::endl;
    std::cout << "Computed " << subpaths.size() << " subpaths" << std::endl;

    FindWorstJump(state, lb_result.tour_edges);
    return 0;

    std::vector<StopId> greedy_path;

    int num_matched = 0;
    int num_unmatched = 0;

    for (int subpath_index = 0; subpath_index < subpaths.size(); ++subpath_index) {
      const Path& subpath = subpaths[subpath_index];
      if (subpath.steps.size() == 0) {
        continue;
      }
      std::vector<StopId> candidate_path;
      candidate_path.push_back(subpath.steps[0].origin.stop);
      for (int i = 0; i < subpath.steps.size(); ++i) {
        candidate_path.push_back(subpath.steps[i].destination.stop);
      }

      int candidate_offset = FindBestMatch(greedy_path, candidate_path);
      if (candidate_offset == -1) {
        num_unmatched += 1;
        continue;
      }
      num_matched += 1;
      int prepend_count = 0;
      int append_count = 0;
      if (candidate_offset < 0) {
        prepend_count = -candidate_offset;
      }
      if (candidate_offset + candidate_path.size() > greedy_path.size()) {
        append_count = candidate_offset + candidate_path.size() - greedy_path.size();
      }

      std::vector<StopId> new_greedy_path;
      for (int i = 0; i < prepend_count; ++i) {
        new_greedy_path.push_back(candidate_path[i]);
      }
      for (int i = 0; i < greedy_path.size(); ++i) {
        new_greedy_path.push_back(greedy_path[i]);
      }
      for (int i = 0; i < append_count; ++i) {
        new_greedy_path.push_back(candidate_path[candidate_path.size() - append_count + i]);
      }

      greedy_path = std::move(new_greedy_path);
    }

    std::cout << "matched: " << num_matched << "\n";
    std::cout << "unmatched: " << num_unmatched << "\n";
    std::cout << "greedy path size: " << greedy_path.size() << "\n";

    for (int i = 0; i < greedy_path.size(); ++i) {
      std::cout << state.StopName(greedy_path[i]) << "\n";
    }

    for (int subpath_index = 0; subpath_index < 5; ++subpath_index) {
      const Path& subpath = subpaths[subpath_index];
      std::cout << TimeSinceServiceStart{subpath.DurationSeconds()}.ToString();
      if (subpath.merged_step.is_flex) {
        std::cout << " flex";
      }
      std::cout << "\n";
      for (int i = 0; i < subpath.steps.size(); ++i) {
        if (i == 0) {
          std::cout << state.StopName(subpath.steps[i].origin.stop) << " " << subpath.steps[i].origin.time.ToString() << "\n";
        }
        std::cout
          << state.StopName(subpath.steps[i].destination.stop)
          << " " << subpath.steps[i].destination.time.ToString()
          << " " << state.PartitionName(subpath.steps[i].destination.partition);
        if (subpath.steps[i].is_flex) {
          std::cout << " flex";
        }
        std::cout << "\n";
      }
      std::cout << "\n";
    }

    return 0;
}
