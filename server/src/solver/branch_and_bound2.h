#pragma once

#include "solver/tarel_graph.h"
namespace vats5 {

struct BranchAndBound2Result {
  int best_ub;
};

BranchAndBound2Result BranchAndBound2Solve(const ProblemState& initial_state);

}  // namespace vats5
