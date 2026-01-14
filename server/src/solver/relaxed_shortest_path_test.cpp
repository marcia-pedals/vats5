#include "solver/relaxed_shortest_path.h"

#include <gtest/gtest.h>

#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

TEST(RelaxedShortestPathTest, FindShortestRelaxedPathsBasic) {
  // Simple graph:
  //   1 --10--> 2 --20--> 4
  //   |         ^
  //   5         |
  //   |        15
  //   v         |
  //   3 --------+

  std::vector<Step> steps = {
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{0},
          .destination_time = TimeSinceServiceStart{10},
          .origin_trip = TripId{1},
          .destination_trip = TripId{1},
          .is_flex = true
      },
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{3},
          .origin_time = TimeSinceServiceStart{0},
          .destination_time = TimeSinceServiceStart{5},
          .origin_trip = TripId{2},
          .destination_trip = TripId{2},
          .is_flex = true
      },
      Step{
          .origin_stop = StopId{3},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{0},
          .destination_time = TimeSinceServiceStart{15},
          .origin_trip = TripId{3},
          .destination_trip = TripId{3},
          .is_flex = true
      },
      Step{
          .origin_stop = StopId{2},
          .destination_stop = StopId{4},
          .origin_time = TimeSinceServiceStart{0},
          .destination_time = TimeSinceServiceStart{20},
          .origin_trip = TripId{4},
          .destination_trip = TripId{4},
          .is_flex = true
      },
  };

  StepsAdjacencyList steps_list = MakeAdjacencyList(steps);
  std::vector<WeightedEdge> relaxed_edges = MakeRelaxedEdges(steps_list);
  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(relaxed_edges);

  std::vector<int> distances = FindShortestRelaxedPaths(relaxed, StopId{1});

  EXPECT_EQ(distances[1], 0);   // origin
  EXPECT_EQ(distances[2], 10);  // 1 -> 2
  EXPECT_EQ(distances[3], 5);   // 1 -> 3
  EXPECT_EQ(distances[4], 30);  // 1 -> 2 -> 4
}

}  // namespace vats5
