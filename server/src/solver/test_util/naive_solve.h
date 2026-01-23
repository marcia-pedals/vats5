#pragma once

#include <vector>

#include "solver/tarel_graph.h"

namespace vats5 {

struct NaiveSolveResult {
  int value;
  struct OptimalPermutation {
    std::vector<StopId> stops;
    TimeSinceServiceStart origin_time;
    TimeSinceServiceStart destination_time;
  };
  std::vector<OptimalPermutation> optimal_permutations;
};

// Find the optimal tour using the naive brute force of trying every
// permutation.
//
// Only counts tours that start at or after 00:00:00 because branching can
// eliminate earlier tours. (This happens because completion only considers
// scheduled steps that happen at or after 00:00:00, and branching can turn a
// flex step plus an early scheduled step into a scheduled step that starts
// before 00:00:00).
NaiveSolveResult NaiveSolve(const ProblemState& state);

}  // namespace vats5
