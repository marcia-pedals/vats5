#include "solver/branch_and_bound.h"

#include <gtest/gtest.h>

namespace vats5 {

TEST(BranchAndBoundTest, StubReturnsZero) {
  EXPECT_EQ(BranchAndBoundSolve(), 0);
}

}  // namespace vats5
