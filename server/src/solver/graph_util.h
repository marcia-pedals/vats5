#pragma once

#include "solver/steps_adjacency_list.h"
namespace vats5 {

struct ProblemState;

// Return the "extreme stops" of `stops` in `g`.
//
// A stop is "extreme" if it is not "inner". Stop V is "inner" if there are
// stops X and Y such that all paths from X to Y and all paths from Y to X go
// through V.
//
// Paths that go through `block_paths_through` are not considered. (Paths
// starting or ending at it are considered though.)
std::unordered_set<StopId> ComputeExtremeStops(
  const StepsAdjacencyList& g_steps,
  const StepPathsAdjacencyList& g,
  const std::unordered_set<StopId>& stops,
  StopId block_paths_through,
  const ProblemState* state = nullptr
);

}  // namespace vats5
