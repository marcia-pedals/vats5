#include "solver/embiggener.h"

#include <gtest/gtest.h>

namespace vats5 {

TEST(EmbiggenerTest, EmbiggensValue) { EXPECT_EQ(Embiggen(5), 6); }

}  // namespace vats5
