#include <fstream>
#include <iostream>
#include <unordered_set>
#include <string>
#include <cstring>

#include <nlohmann/json.hpp>

#include "solver/data.h"
#include "solver/graph_util.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

void print_usage(const char* program_name) {
    std::cerr << "Usage: " << program_name << " <gtfs_path> <output_path.json> [options]\n";
    std::cerr << "\nOptions:\n";
    std::cerr << "  --max-walking-distance=<meters>  Maximum walking distance (default: 500.0)\n";
    std::cerr << "  --walking-speed=<m/s>            Walking speed in m/s (default: 1.0)\n";
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    const std::string gtfs_path = argv[1];
    const std::string output_path = argv[2];

    // Default options
    GetStepsOptions options{
        .max_walking_distance_meters = 500.0,
        .walking_speed_ms = 1.0,
    };

    // Parse optional named arguments
    for (int i = 3; i < argc; ++i) {
        if (strncmp(argv[i], "--max-walking-distance=", 23) == 0) {
            options.max_walking_distance_meters = std::stod(argv[i] + 23);
        } else if (strncmp(argv[i], "--walking-speed=", 16) == 0) {
            options.walking_speed_ms = std::stod(argv[i] + 16);
        } else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    std::cout << "Loading GTFS data from: " << gtfs_path << std::endl;
    std::cout << "Options: max_walking_distance=" << options.max_walking_distance_meters
              << "m, walking_speed=" << options.walking_speed_ms << "m/s\n";
    GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);

    gtfs_day = GtfsNormalizeStops(gtfs_day);
    StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(gtfs_day, options);

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
      state.original_edges
    );

    std::cout << "Serializing to JSON...\n";
    nlohmann::json j = state;
    std::ofstream out(output_path);
    out << j.dump();
    out.close();
    std::cout << "Saved problem state to: " << output_path << "\n";

    return 0;
}
