#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unordered_set>

#include "solver/branch_and_bound.h"
#include "solver/branch_and_bound2.h"
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"

using namespace vats5;

std::string CreateRunDir() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time);

    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string dir_name = ss.str();

    mkdir(dir_name.c_str(), 0755);
    return dir_name;
}

int main() {
    const std::string gtfs_path = "../data/RG_20260108_all";
    // const std::string gtfs_path = "../data/RG_20250718_BA";

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
    ProblemState initial_state = InitializeProblemState(steps_from_gtfs, bart_stops, true);

    // StopId berryessa = initial_state.StopIdFromName("Berryessa / North San Jose");
    // PrintStopPartitions(initial_state, berryessa);

    std::string run_dir = CreateRunDir();
    std::cout << "Run directory: " << run_dir << std::endl;

    BranchAndBoundSolve(initial_state, &std::cout, run_dir);

    return 0;
}
