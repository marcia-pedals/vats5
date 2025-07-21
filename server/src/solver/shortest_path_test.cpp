#include "solver/shortest_path.h"

#include <gtest/gtest.h>

namespace vats5 {

TEST(ShortestPathTest, MakeAdjacencyListBasic) {
  std::vector<Step> steps = {
    Step{
      .origin_stop = StopId{1},
      .destination_stop = StopId{2},
      .origin_time = TimeSinceServiceStart{100},
      .destination_time = TimeSinceServiceStart{200},
      .origin_trip = TripId{1},
      .destination_trip = TripId{1}
    },
    Step{
      .origin_stop = StopId{1},
      .destination_stop = StopId{2},
      .origin_time = TimeSinceServiceStart{150},
      .destination_time = TimeSinceServiceStart{250},
      .origin_trip = TripId{2},
      .destination_trip = TripId{2}
    }
  };
  
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps);
  
  EXPECT_EQ(adjacency_list.adjacent.size(), 1);
  EXPECT_NE(adjacency_list.adjacent.find(StopId{1}), adjacency_list.adjacent.end());
  EXPECT_EQ(adjacency_list.adjacent[StopId{1}].size(), 1);
  EXPECT_EQ(adjacency_list.adjacent[StopId{1}][0].size(), 2);
}

}  // namespace vats5