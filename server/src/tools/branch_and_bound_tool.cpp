#include <iostream>
#include <unordered_set>

#include "solver/branch_and_bound_solver.h"
#include "solver/data.h"
#include "solver/shortest_path.h"

using namespace vats5;

int main() {
    const std::string gtfs_path = "../data/RG_20260108_all";

    std::cerr << "Loading GTFS data from: " << gtfs_path << std::endl;
    GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);

    std::cerr << "Normalizing stops..." << std::endl;
    gtfs_day = GtfsNormalizeStops(gtfs_day);

    std::cerr << "Getting steps..." << std::endl;
    StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});

    std::cerr << "Making adjacency list..." << std::endl;
    StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);

    std::unordered_set<StopId> bart_stops =
        GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

    std::cerr << "Running branch and bound solver..." << std::endl;
    DoIt(steps_from_gtfs.mapping, adjacency_list, bart_stops);

    return 0;
}
