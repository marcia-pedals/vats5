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
// in [00:00, 24:00).
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
std::unordered_map<StopId, std::vector<Step>> FindMinimalPathSet(
    const StepsAdjacencyList& adjacency_list,
    StopId origin,
    std::vector<StopId> destinations
);

}  // namespace vats5
