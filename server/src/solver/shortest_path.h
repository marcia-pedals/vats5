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

// Return all minimal steps from origin to destinations.
std::vector<std::vector<Step>> FindShortestPaths(
    const StepsAdjacencyList& adjacency_list, StopId origin,
    std::vector<StopId> destinations);

}  // namespace vats5
