#pragma once

#include "solver/steps_adjacency_list.h"
namespace vats5 {

// Return the "extreme stops" of `stops` in `g`.
//
// A stop is "extreme" if it is not "inner". Stop V is "inner" if there are
// stops X and Y such that all paths from X to Y and all paths from Y to X go
// through V.
std::unordered_set<StopId> ComputeExtremeStops(
  const StepPathsAdjacencyList& g,
  const std::unordered_set<StopId>& stops
);

}  // namespace vats5
