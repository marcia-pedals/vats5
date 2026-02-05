#include <fstream>
#include <iostream>
#include <unordered_set>

#include <nlohmann/json.hpp>

#include "solver/data.h"
#include "solver/graph_util.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: save_problem_state <output_path.json>\n";
        return 1;
    }

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
    ProblemState state = InitializeProblemState(steps_from_gtfs, bart_stops, /*optimize_edges=*/true);

    auto extreme_stops = ComputeExtremeStops(
      state.completed, state.required_stops, state.boundary.start
    );
    std::cout << "Final extreme stop count: " << extreme_stops.size() << "\n";
    state = MakeProblemState(
      MakeAdjacencyList(ReduceToMinimalSystemPaths(state.minimal, extreme_stops).AllMergedSteps()),
      state.boundary,
      extreme_stops,
      state.stop_names,
      state.step_partition_names,
      state.original_origins,
      state.original_destinations
    );

    std::cout << "Serializing to JSON...\n";
    nlohmann::json j = state;
    std::string output_path = argv[1];
    std::ofstream out(output_path);
    out << j.dump();
    out.close();
    std::cout << "Saved problem state to: " << output_path << "\n";

    return 0;
}
