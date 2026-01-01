#pragma once

#include "solver/data.h"

namespace vats5 {

struct StepsAdjacencyList {
  // Mapping from stop to steps originating at that stop, grouped by destination
  // stop. Each group of steps is sorted by origin time and minimal.
  std::unordered_map<StopId, std::vector<std::vector<Step>>> adjacent;
};

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
std::unordered_map<StopId, std::vector<std::vector<Path>>>
ReduceToMinimalSystemPaths(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& system_stops
);

std::unordered_map<StopId, std::vector<std::vector<Path>>> SplitPathsAt(
    const std::unordered_map<StopId, std::vector<std::vector<Path>>>& paths,
    const std::unordered_set<StopId> intermediate_stops
);

StepsAdjacencyList AdjacentPathsToStepsList(
    const std::unordered_map<StopId, std::vector<std::vector<Path>>>& paths
);

}  // namespace vats5
