#pragma once

#include <unordered_set>
#include <vector>

#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

// Find shortest paths from origin to all reachable stops using Dijkstra's
// algorithm. Returns a vector indexed by StopId.v containing the shortest
// distance in seconds from origin to each stop. Unreachable stops have distance
// std::numeric_limits<int>::max().
std::vector<int> FindShortestRelaxedPaths(
    const RelaxedAdjacencyList& adjacency_list, StopId origin
);

// Compute relaxed distances from all stops to each destination.
RelaxedDistances ComputeRelaxedDistances(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& destinations
);

}  // namespace vats5
