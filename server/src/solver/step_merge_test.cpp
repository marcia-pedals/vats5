#include <gtest/gtest.h>
#include "solver/step_merge.h"

namespace vats5 {

TEST(StepMergeTest, DummyMergeFunctionTest) {
    int result = dummy_merge_function(3, 5);
    EXPECT_EQ(result, 8);
}

TEST(StepMergeTest, DummyMergeFunctionZeroTest) {
    int result = dummy_merge_function(0, 0);
    EXPECT_EQ(result, 0);
}

TEST(StepMergeTest, DummyMergeFunctionNegativeTest) {
    int result = dummy_merge_function(-2, 7);
    EXPECT_EQ(result, 5);
}

}  // namespace vats5