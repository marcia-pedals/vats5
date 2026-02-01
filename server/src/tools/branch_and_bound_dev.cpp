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

// Extract all intermediate stops from a path (excluding origin and destination)
std::unordered_set<StopId> GetIntermediateStops(const Path& path, StopId a, StopId b) {
    std::unordered_set<StopId> stops;
    for (const Step& step : path.steps) {
        if (step.origin.stop != a && step.origin.stop != b) {
            stops.insert(step.origin.stop);
        }
        if (step.destination.stop != a && step.destination.stop != b) {
            stops.insert(step.destination.stop);
        }
    }
    return stops;
}

// Find stops that appear in ALL paths
std::unordered_set<StopId> StopsOnAllPaths(std::span<const Path> paths, StopId a, StopId b) {
    if (paths.empty()) {
        return {};
    }

    // Start with stops from the first path
    std::unordered_set<StopId> result = GetIntermediateStops(paths[0], a, b);

    // Intersect with stops from all other paths
    for (size_t i = 1; i < paths.size(); ++i) {
        std::unordered_set<StopId> path_stops = GetIntermediateStops(paths[i], a, b);
        std::unordered_set<StopId> intersection;
        for (StopId stop : result) {
            if (path_stops.count(stop)) {
                intersection.insert(stop);
            }
        }
        result = std::move(intersection);
    }
    return result;
}

// Compute extreme stops: required stops that are not "inner" stops.
// Inner stops are stops that appear on all paths between some pair of required stops.
std::vector<StopId> ComputeExtremeStops(const ProblemState& state) {
  std::vector<StopId> required_stops_vec(
      state.required_stops.begin(),
      state.required_stops.end());

  // Collect all inner stops
  std::unordered_set<StopId> inner_stops;
  for (size_t i = 0; i < required_stops_vec.size(); ++i) {
    for (size_t j = i + 1; j < required_stops_vec.size(); ++j) {
      StopId a = required_stops_vec[i];
      StopId b = required_stops_vec[j];

      auto paths_ab = state.completed.PathsBetween(a, b);
      auto paths_ba = state.completed.PathsBetween(b, a);

      // Get stops on all a->b paths
      std::unordered_set<StopId> stops_ab = StopsOnAllPaths(paths_ab, a, b);
      // Get stops on all b->a paths
      std::unordered_set<StopId> stops_ba = StopsOnAllPaths(paths_ba, a, b);

      // Add stops that are on all paths in both directions
      for (StopId stop : stops_ab) {
        if (stops_ba.count(stop)) {
          inner_stops.insert(stop);
        }
      }
    }
  }

  // Collect extreme stops (required stops that are not inner stops, excluding START and END)
  std::vector<StopId> extreme_stops;
  for (StopId stop : state.required_stops) {
    if (!inner_stops.count(stop) &&
        stop != state.boundary.start &&
        stop != state.boundary.end) {
      extreme_stops.push_back(stop);
    }
  }
  return extreme_stops;
}

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

    std::vector<StopId> extreme_stops = ComputeExtremeStops(initial_state);

    initial_state.required_stops = {initial_state.boundary.start, initial_state.boundary.end};
    std::cout << "Extreme stops:\n";
    for (StopId stop : extreme_stops) {
      initial_state.required_stops.insert(stop);
      std::cout << "  " << initial_state.StopName(stop) << "\n";
    }

    // StopId berryessa = initial_state.StopIdFromName("Berryessa / North San Jose");
    // PrintStopPartitions(initial_state, berryessa);

    std::string run_dir = CreateRunDir();
    std::cout << "Run directory: " << run_dir << std::endl;

    BranchAndBoundSolve2(initial_state, &std::cout, run_dir);

    return 0;
}
