#pragma once

#include <vector>

#include "solver/tarel_graph.h"

namespace vats5 {

struct BranchAndCutResult {
  int best_ub;
  std::vector<Path> best_paths;
};

// Solves the problem using a branch-and-cut strategy.
//
// TODO: This is a stub. Implement me.
BranchAndCutResult BranchAndCutSolve(const ProblemState& initial_state);

}  // namespace vats5
