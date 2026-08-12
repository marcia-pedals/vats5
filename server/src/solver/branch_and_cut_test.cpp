#include "solver/branch_and_cut.h"

#include <gtest/gtest.h>

namespace vats5 {

// TODO: Replace with a meaningful test as BranchAndCutSolve is implemented.
TEST(BranchAndCutTest, StubReturnsZero) {
  ProblemState state;
  BranchAndCutResult result = BranchAndCutSolve(state);
  EXPECT_EQ(result.best_ub, 0);
}

}  // namespace vats5
