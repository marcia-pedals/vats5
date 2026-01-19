#include <cassert>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_set>

#include "solver/data.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

int main() {
    const std::string gtfs_path = "../data/RG_20260108_all";

    std::cout << "Loading GTFS data from: " << gtfs_path << std::endl;
    GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);

    gtfs_day = GtfsNormalizeStops(gtfs_day);
    StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(
      gtfs_day,
      GetStepsOptions{
        .max_walking_distance_meters=1000.0,
        .walking_speed_ms=1.0,
      }
    );

    std::unordered_set<StopId> bart_stops =
        GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

    std::cout << "Initializing solution state...\n";
    SolutionState state = InitializeSolutionState(steps_from_gtfs, bart_stops);

    // Dummy code for typechecking.
    StepPathsAdjacencyList completed =
      ReduceToMinimalSystemPaths(state.adj, state.stops, /*keep_through_other_destination=*/true);

    // Add END -> START edge.
    // It's safe to push a new group without checking for an existing group with
    // `start` dest because no edges (other than this one we're adding right
    // now) go into `start`.
    assert(completed.adjacent[state.boundary.end].size() == 0);
    completed.adjacent[state.boundary.end].push_back({ZeroPath(state.boundary.end, state.boundary.start)});

    std::unordered_map<std::string, StepPartitionId> partition_to_id;
    std::unordered_map<StepPartitionId, std::string> id_to_partition;
    for (const auto& [_, v] : state.dest_trip_id_to_partition) {
      auto [it, inserted] = partition_to_id.try_emplace(v, StepPartitionId{static_cast<int>(partition_to_id.size())});
      if (inserted) {
        id_to_partition[it->second] = it->first;
      }
    }

    auto tarel_result = MakeTarelEdges(
      completed,
      std::function<StepPartitionId(Step)>([&](const Step& s) -> StepPartitionId {
        if (s.is_flex) {
          return partition_to_id.at("flex");
        }
        return partition_to_id.at(state.dest_trip_id_to_partition[s.destination_trip]);
      })
    );
    auto tarel_es_2 = MergeEquivalentTarelStates(tarel_result);

    TspGraphData graph = MakeTspGraphEdges(tarel_es_2, state.boundary);
    TspTourResult tour_result = SolveTspAndExtractTour(tarel_es_2, graph, state.boundary);
    PrintTarelTourResults(tour_result, state, completed, id_to_partition);

    // TODO: Think about whether it's possible for there to be a situation where
    // merging multiple times makes progress each time.
    // ... it seems like merging states could make it be so that some states who
    // previously had distinct dest states could now have the same dest states.
    // But:
    // - Maybe there's a reason why anything that ends up with same dest states must have started with same dest states anyways.
    // - Or not quite, but where there has to be a very unlikely coincidence of unrelated weights for dest states to get merged in such a way.
    // auto tarel_es_3 = MergeEquivalentTarelStates(tarel_es_2, state, tarel_result.state_descriptions);

    return 0;
}
