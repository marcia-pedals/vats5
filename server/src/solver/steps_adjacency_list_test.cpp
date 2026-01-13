#include "solver/steps_adjacency_list.h"

#include <gtest/gtest.h>

namespace vats5 {

TEST(StepsAdjacencyListTest, MakeAdjacencyListBasic) {
  std::vector<Step> steps = {
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{100},
          .destination_time = TimeSinceServiceStart{200},
          .origin_trip = TripId{1},
          .destination_trip = TripId{1},
          .is_flex = false
      },
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{150},
          .destination_time = TimeSinceServiceStart{250},
          .origin_trip = TripId{2},
          .destination_trip = TripId{2},
          .is_flex = false
      }
  };

  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps);

  // Check that stop 1 has groups (and stop 0 has none)
  EXPECT_TRUE(adjacency_list.GetGroups(StopId{0}).empty());
  EXPECT_FALSE(adjacency_list.GetGroups(StopId{1}).empty());
  EXPECT_EQ(adjacency_list.GetGroups(StopId{1}).size(), 1);
  EXPECT_EQ(
      adjacency_list.GetSteps(adjacency_list.GetGroups(StopId{1})[0]).size(), 2
  );
}

}  // namespace vats5
