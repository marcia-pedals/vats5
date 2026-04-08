#include <Highs.h>
#include <gtest/gtest.h>

namespace vats5 {
namespace {

// Solve: minimize x + y
//   subject to: x + y >= 2
//               x >= 0, y >= 0
// Optimal objective value = 2.
TEST(HighsTest, SimpleLP) {
  Highs highs;
  highs.setOptionValue("output_flag", false);

  // Two variables, both with lower bound 0, no upper bound.
  highs.addVar(0.0, kHighsInf);
  highs.addVar(0.0, kHighsInf);

  // Objective: minimize x + y.
  highs.changeColCost(0, 1.0);
  highs.changeColCost(1, 1.0);

  // Constraint: x + y >= 2.
  std::vector<HighsInt> indices = {0, 1};
  std::vector<double> values = {1.0, 1.0};
  highs.addRow(2.0, kHighsInf, indices.size(), indices.data(), values.data());

  HighsStatus status = highs.run();
  ASSERT_EQ(status, HighsStatus::kOk);

  EXPECT_NEAR(highs.getObjectiveValue(), 2.0, 1e-9);

  const auto& solution = highs.getSolution();
  EXPECT_NEAR(solution.col_value[0] + solution.col_value[1], 2.0, 1e-9);
}

}  // namespace
}  // namespace vats5
