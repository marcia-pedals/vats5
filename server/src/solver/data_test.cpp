#include <gtest/gtest.h>
#include "solver/data.h"

namespace vats5 {

TEST(DataTest, GetStepsFromGtfsPlaceholder) {
    GtfsDay gtfs;
    StepsFromGtfs result = GetStepsFromGtfs(gtfs);
    
    EXPECT_TRUE(result.steps.empty());
}

}  // namespace vats5