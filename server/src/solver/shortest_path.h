#pragma once

#include "solver/data.h"

namespace vats5 {

// A group of steps from one origin to one destination, sorted by origin time.
// Includes a separate array of departure times for cache-friendly binary
// search.
struct StepGroup {
  std::vector<Step> steps;
  // Parallel array: departure_times_div10[i] = steps[i].origin_time.seconds /
  // 10 for non-flex steps. If steps[0] is flex, departure_times_div10[0] is
  // ignored and the array still has the same size as steps. Divided by 10 to
  // fit in int16_t (max 32767 * 10 = 327670 seconds ≈ 91 hours).
  std::vector<int16_t> departure_times_div10;
};

inline void to_json(nlohmann::json& j, const StepGroup& sg) {
  j = nlohmann::json{
      {"steps", sg.steps}, {"departure_times_div10", sg.departure_times_div10}
  };
}

inline void from_json(const nlohmann::json& j, StepGroup& sg) {
  sg.steps = j.at("steps").get<std::vector<Step>>();
  sg.departure_times_div10 =
      j.at("departure_times_div10").get<std::vector<int16_t>>();
}

struct StepsAdjacencyList {
  // Mapping from stop to step groups originating at that stop, grouped by
  // destination stop. Each group of steps is sorted by origin time and minimal.
  std::unordered_map<StopId, std::vector<StepGroup>> adjacent;

  // Strict upper bound on all StopId values in this adjacency list.
  // All stop IDs s satisfy s.v < stop_id_ub.
  int stop_id_ub = 0;
};

// Custom JSON serialization for StepsAdjacencyList
// Convert the StopId-keyed map to int-keyed for JSON
inline void to_json(nlohmann::json& j, const StepsAdjacencyList& adj) {
  std::vector<std::pair<int, std::vector<StepGroup>>> pairs;
  for (const auto& [k, v] : adj.adjacent) {
    pairs.emplace_back(k.v, v);
  }
  j = nlohmann::json{{"adjacent", pairs}, {"stop_id_ub", adj.stop_id_ub}};
}

inline void from_json(const nlohmann::json& j, StepsAdjacencyList& adj) {
  auto pairs = j.at("adjacent")
                   .get<std::vector<std::pair<int, std::vector<StepGroup>>>>();
  for (const auto& [k, v] : pairs) {
    adj.adjacent[StopId{k}] = v;
  }
  adj.stop_id_ub = j.at("stop_id_ub").get<int>();
}

struct PathsAdjacencyList {
  // Mapping from stop to paths originating at that stop, grouped by destination
  // stop. Each group of paths is sorted by origin time and minimal.
  std::unordered_map<StopId, std::vector<std::vector<Path>>> adjacent;
};

// Custom JSON serialization for PathsAdjacencyList
// Convert the StopId-keyed map to int-keyed for JSON
inline void to_json(nlohmann::json& j, const PathsAdjacencyList& adj) {
  std::vector<std::pair<int, std::vector<std::vector<Path>>>> pairs;
  for (const auto& [k, v] : adj.adjacent) {
    pairs.emplace_back(k.v, v);
  }
  j = nlohmann::json{{"adjacent", pairs}};
}

inline void from_json(const nlohmann::json& j, PathsAdjacencyList& adj) {
  auto pairs =
      j.at("adjacent")
          .get<std::vector<std::pair<int, std::vector<std::vector<Path>>>>>();
  for (const auto& [k, v] : pairs) {
    adj.adjacent[StopId{k}] = v;
  }
}

// Group steps into an adjacency list.
StepsAdjacencyList MakeAdjacencyList(const std::vector<Step>& steps);

// The data we keep at each stop along a shortest path search.
//
// Stores the step used to reach this stop. The full path can be reconstructed
// by backtracking through the result map.
struct PathState {
  // The step taken to reach this stop.
  Step step;
};

// Backtrack through the search results to reconstruct the full path.
// Returns the steps in order from origin to destination.
std::vector<Step> BacktrackPath(
    const std::vector<PathState>& search_result, StopId dest
);

// Compute the merged step for a path, with proper origin time calculation.
// The origin time adjustment handles flex paths that transition to fixed trips.
Step ComputeMergedStep(const std::vector<Step>& path);

// Find earliest times you can get to a set of stops from `origin` when starting
// at `time`.
//
// The return includes at least values for `destinations` and for all
// intermediate stops on the optimal routes to them. It may include values for
// other stops visited during the search.
//
// The returned steps may depart later than `time` from `origin`, but you should
// not rely on this to find the latest possible departure time (see the
// SuboptimalDepartureTimeExposure test).
//
// Returns a vector indexed by StopId.v. Unvisited stops have
// destination_time.seconds == numeric_limits<int>::max().
std::vector<PathState> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list,
    TimeSinceServiceStart time,
    StopId origin,
    const std::unordered_set<StopId>& destinations
);

// Return a minimal set of paths from `origin` to destinations, with origin
// times in [00:00, 36:00). Return value is a map from destination to the paths
// from `origin` to that destination.
//
// Minimal means that these properties hold:
// (a) Any path in `adjacency_list` from `origin` to a destination can be
// matched or beat by a path in the return. (b) If you remove any path from the
// return value, (a) no longer holds.
std::unordered_map<StopId, std::vector<Path>> FindMinimalPathSet(
    const StepsAdjacencyList& adjacency_list,
    StopId origin,
    const std::unordered_set<StopId>& destinations
);

// Return an adjacency list ("reduced list") with these properties:
// (a) Any path in `adjacency_list` between two `system_stops` [1] can be
// matched or beat by a path in the reduced list. (b) If you remove any step
// from the reduced list, (a) no longer holds.
//
// [1] Usually-unimportant qualification: All departures from `system_stops` in
// the path happen at <36:00.
PathsAdjacencyList ReduceToMinimalSystemPaths(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& system_stops
);

PathsAdjacencyList SplitPathsAt(
    const PathsAdjacencyList& paths,
    const std::unordered_set<StopId> intermediate_stops
);

StepsAdjacencyList AdjacentPathsToStepsList(const PathsAdjacencyList& paths);

// Return a modified `path` where any intermediate stops within
// `threshold_meters` of a stop in `stops` are replaced by that stop, and then
// steps that are thusly changed to be from a stop to itself are dropped.
//
// Note that this doesn't change any trip ids or times in the modified steps, so
// the steps become steps that may not actually be possible, so maybe this
// should be used only to clean up visualizations and not to do actual
// computations.
std::vector<Step> SnapToStops(
    const DataGtfsMapping& mapping,
    const std::unordered_set<StopId>& stops,
    double threshold_meters,
    const std::vector<Step>& path
);

// Snap all paths in the adjacency list to the stops that are keys of the
// adjacency list.
PathsAdjacencyList AdjacencyListSnapToStops(
    const DataGtfsMapping& mapping,
    double threshold_meters,
    const PathsAdjacencyList& paths
);

}  // namespace vats5
