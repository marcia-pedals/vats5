#include <algorithm>
#include <asio/execution/start.hpp>
#include <iostream>
#include <unordered_set>

#include "solver/concorde.h"
#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"

using namespace vats5;

struct SolutionMetadata {
  // stop_names[stop_id.v] is the name of stop_id
  std::vector<std::string> stop_names;

  SolutionMetadata Remapped(const StopIdMapping& mapping) {
    SolutionMetadata result;
    result.stop_names.resize(mapping.new_to_original.size(), "");
    for (int i = 0; i < mapping.new_to_original.size(); ++i) {
      result.stop_names[i] = this->stop_names[mapping.new_to_original[i].v];
    }
    return result;
  }
};

SolutionMetadata InitializeSolutionMetadata(const DataGtfsMapping& mapping) {
  int max_stop_id = 0;
  for (const auto& [stop_id, _]: mapping.stop_id_to_stop_name) {
    if (stop_id.v > max_stop_id) {
      max_stop_id = stop_id.v;
    }
  }

  SolutionMetadata result;
  result.stop_names.resize(max_stop_id + 1, "");

  for (const auto& [stop_id, stop_name]: mapping.stop_id_to_stop_name) {
    result.stop_names[stop_id.v] = stop_name;
  }

  return result;
}

struct SolutionState {
  StepsAdjacencyList adj;
  SolutionMetadata metadata;
};

SolutionState InitializeSolutionState(
  const StepsFromGtfs& steps_from_gtfs,
  const std::unordered_set<StopId> system_stops
) {
  // Compute minimal adj list.
  StepPathsAdjacencyList minimal_paths_sparse = ReduceToMinimalSystemPaths(MakeAdjacencyList(steps_from_gtfs.steps), system_stops);
  StepsAdjacencyList minimal_steps_sparse = MakeAdjacencyList(minimal_paths_sparse.AllMergedSteps());

  // Compact minimal adj list and make compact solution metadata.
  CompactStopIdsResult minimal_compact = CompactStopIds(minimal_steps_sparse);
  SolutionMetadata solution_metadata = InitializeSolutionMetadata(steps_from_gtfs.mapping).Remapped(minimal_compact.mapping);

  // Add the "START" vertex to the state.
  StopId start_vertex = StopId{minimal_compact.list.NumStops()};
  solution_metadata.stop_names.push_back("START");
  assert(solution_metadata.stop_names[start_vertex.v] == "START");

  auto zero_edge = [](StopId a, StopId b) -> Step {
    return Step{
          a,
          b,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{0},
          TripId{-2}, // TODO
          TripId{-2}, // TODO
          /*is_flex=*/true
        };
  };

  // ... with 0-duration flex steps to and from every other vertex.
  std::vector<Step> steps = minimal_compact.list.AllSteps();
  for (StopId normal_stop = StopId{0}; normal_stop.v < minimal_compact.list.NumStops(); normal_stop.v += 1) {
    steps.push_back(zero_edge(start_vertex, normal_stop));
    steps.push_back(zero_edge(normal_stop, start_vertex));
  }

  return SolutionState{
    MakeAdjacencyList(steps),
    solution_metadata,
  };
}

int main() {
    const std::string gtfs_path = "../data/RG_20260108_all";

    std::cout << "Loading GTFS data from: " << gtfs_path << std::endl;
    GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);

    std::cout << "Normalizing stops..." << std::endl;
    gtfs_day = GtfsNormalizeStops(gtfs_day);

    std::cout << "Getting steps..." << std::endl;
    StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});

    std::unordered_set<StopId> bart_stops =
        GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

    SolutionState state = InitializeSolutionState(steps_from_gtfs, bart_stops);

    RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyList(state.adj);
    ConcordeSolution solution = SolveTspWithConcorde(relaxed);

    std::cout << "Concorde optimal value " << solution.optimal_value << "\n";

    return 0;
}
