#include "solver/branch_and_cut.h"

#include <stdexcept>

namespace vats5 {

BranchAndCutResult BranchAndCutSolve(const ProblemState& initial_state) {
  // This solver does not yet implement the alternate-stops-allowed logic, so it
  // only supports problems where every required-stop group is a singleton.
  for (const std::vector<StopId>& group : initial_state.required.Groups()) {
    if (group.size() != 1) {
      throw std::invalid_argument(
          "BranchAndCutSolve does not support required-stop groups with more "
          "than one stop"
      );
    }
  }

  // TODO: Implement the branch-and-cut strategy.
  return BranchAndCutResult{.best_ub = 0, .best_paths = {}};
}

}  // namespace vats5
