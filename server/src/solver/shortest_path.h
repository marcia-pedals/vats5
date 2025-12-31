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

// Return a minimal set of steps from origin to destinations, with origin times
// in [00:00, 24:00).
//
// The result is minimal in two senses:
// - Each std::vector<Step> in the result satisfies `CheckSortedAndMinimal`.
// - Each Step in the result represents a path that does not touch any
// `destinations` other than its
//   ultimate destination.
//
// TODO: I probably want to use this to replace outgoing trips from e.g. BART
// stations when reducing the system, but to be able to handle past-midnight
// trips correctly, we'll actually need to include past-24:00 origin times,
// because a minimal path from another station in a later reduction step might
// use a minimal path from `origin` that departs `origin` after 24:00. I think I
// basically will need to know the latest possible arrival time for any minimal
// path in the sytem and set the ub to above that. Not sure if there's a quick
// general way to figure that out, but proably like 36:00 will be plenty for
// BART and also most other systems. Can probably detect situations where
// whatever threshold we've set is insufficient, and error out of those.
std::unordered_map<StopId, std::vector<Path>> FindMinimalPathSet(
    const StepsAdjacencyList& adjacency_list,
    StopId origin,
    const std::unordered_set<StopId>& destinations
);

}  // namespace vats5
