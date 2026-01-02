#pragma once

#include "solver/data.h"

namespace vats5 {

struct StepsAdjacencyList {
  // Mapping from stop to steps originating at that stop, grouped by destination
  // stop. Each group of steps is sorted by origin time and minimal.
  std::unordered_map<StopId, std::vector<std::vector<Step>>> adjacent;
};

// Custom JSON serialization for StepsAdjacencyList
// Convert the StopId-keyed map to int-keyed for JSON
inline void to_json(nlohmann::json& j, const StepsAdjacencyList& adj) {
  std::vector<std::pair<int, std::vector<std::vector<Step>>>> pairs;
  for (const auto& [k, v] : adj.adjacent) {
    pairs.emplace_back(k.v, v);
  }
  j = nlohmann::json{{"adjacent", pairs}};
}

inline void from_json(const nlohmann::json& j, StepsAdjacencyList& adj) {
  auto pairs =
      j.at("adjacent")
          .get<std::vector<std::pair<int, std::vector<std::vector<Step>>>>>();
  for (const auto& [k, v] : pairs) {
    adj.adjacent[StopId{k}] = v;
  }
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
// Includes enough data to reconstruct a whole path if you have the state at
// each stop along the path. Also includes various derived data useful during
// the search and useful for using the search results.
//
// TODO: A lot of this data is very redundant for convenience. This struct is
// heavily used in the hot path of the search, so making it smaller may make the
// search a lot faster. One simple idea might be to make the whole search be
// based off `current_step`, and rely on backtracking to get info about the
// departure, instead of computing `whole_step` througout the whole search?
struct PathState {
  // All steps in the whole path so far, merged together.
  Step whole_step;

  // The latest step taken in the search.
  Step current_step;
};

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
std::unordered_map<StopId, PathState> FindShortestPathsAtTime(
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
