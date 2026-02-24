#pragma once

#include <vector>

#include "solver/tarel_graph.h"

namespace vats5 {

struct SolutionSpaceElement {
  std::vector<StopId<>> generating_permutation;
  std::vector<StopId<>> actual_path;
  Step merged_step;
};

// Enumerate the whole solution space by finding all tours for all generating
// permutations.
//
// Only counts tours that start at or after 00:00:00 because branching can
// eliminate earlier tours. (This happens because completion only considers
// scheduled steps that happen at or after 00:00:00, and branching can turn a
// flex step plus an early scheduled step into a scheduled step that starts
// before 00:00:00).
std::vector<SolutionSpaceElement> EnumerateSolutionSpace(
    const ProblemState<>& state
);

// Find the optimal tour by brute force enumerating the solution space.
//
// Only counts tours that start at or after 00:00:00. See above.
//
// Returns std::numeric_limits<int>::max() for infeasible problems.
int BruteForceSolveOptimalDuration(const ProblemState<>& state);

}  // namespace vats5
