#include "util/date.h"

#include <gtest/gtest.h>

namespace vats5 {
namespace {

TEST(OffsetDateTest, IncrementByOne) {
  EXPECT_EQ(OffsetDate("20250101", 1), "20250102");
}

TEST(OffsetDateTest, DecrementByOne) {
  EXPECT_EQ(OffsetDate("20250102", -1), "20250101");
}

TEST(OffsetDateTest, CrossMonthBoundaryForward) {
  EXPECT_EQ(OffsetDate("20250131", 1), "20250201");
}

TEST(OffsetDateTest, CrossMonthBoundaryBackward) {
  EXPECT_EQ(OffsetDate("20250201", -1), "20250131");
}

TEST(OffsetDateTest, CrossYearBoundaryForward) {
  EXPECT_EQ(OffsetDate("20241231", 1), "20250101");
}

TEST(OffsetDateTest, CrossYearBoundaryBackward) {
  EXPECT_EQ(OffsetDate("20250101", -1), "20241231");
}

TEST(OffsetDateTest, LeapYearFebruary) {
  // 2024 is a leap year
  EXPECT_EQ(OffsetDate("20240228", 1), "20240229");
  EXPECT_EQ(OffsetDate("20240229", 1), "20240301");
}

TEST(OffsetDateTest, NonLeapYearFebruary) {
  // 2025 is not a leap year
  EXPECT_EQ(OffsetDate("20250228", 1), "20250301");
}

TEST(OffsetDateTest, ZeroOffset) {
  EXPECT_EQ(OffsetDate("20250615", 0), "20250615");
}

TEST(OffsetDateTest, LargeOffset) {
  // One year forward (365 days in 2025, a non-leap year)
  EXPECT_EQ(OffsetDate("20250101", 365), "20260101");
}

}  // namespace
}  // namespace vats5
