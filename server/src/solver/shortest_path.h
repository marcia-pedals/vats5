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

// Find earliest times you can get to a set of destinations from `origin` when
// starting at `time`.
//
// The returned steps may depart later than `time` from `origin`, but you should
// not rely on this to find the latest possible departure time (see the
// SuboptimalDepartureTimeExposure test).
std::unordered_map<StopId, Step> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list,
    TimeSinceServiceStart time,
    StopId origin,
    const std::unordered_set<StopId>& destinations
);

// Return a minimal set of steps from origin to destination, with origin times
// between 00:00 and 24:00.
std::vector<Step> FindMinimalPathSet(
    const StepsAdjacencyList& adjacency_list, StopId origin, StopId destination
);

// // Return all minimal steps from origin to destinations.
// std::vector<std::vector<Step>> FindShortestPaths(
//     const StepsAdjacencyList& adjacency_list,
//     StopId origin,
//     std::vector<StopId> destinations
// );

}  // namespace vats5
