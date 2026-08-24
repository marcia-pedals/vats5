#include "solver/relaxed_shortest_path.h"

#include <gtest/gtest.h>

#include <limits>

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
      Step::PrimitiveFlex(StopId{1}, StopId{2}, 10, TripId{1}),
      Step::PrimitiveFlex(StopId{1}, StopId{3}, 5, TripId{2}),
      Step::PrimitiveFlex(StopId{3}, StopId{2}, 15, TripId{3}),
      Step::PrimitiveFlex(StopId{2}, StopId{4}, 20, TripId{4}),
  };

  StepsAdjacencyList steps_list = MakeAdjacencyList(steps);
  std::vector<WeightedEdge> relaxed_edges = MakeRelaxedEdges(steps_list);
  RelaxedAdjacencyList relaxed =
      MakeRelaxedAdjacencyListFromEdges(relaxed_edges);

  std::vector<int> distances = FindShortestRelaxedPaths(relaxed, StopId{1});

  EXPECT_EQ(distances[1], 0);   // origin
  EXPECT_EQ(distances[2], 10);  // 1 -> 2
  EXPECT_EQ(distances[3], 5);   // 1 -> 3
  EXPECT_EQ(distances[4], 30);  // 1 -> 2 -> 4
}

TEST(RelaxedShortestPathTest, FindShortestRelaxedPathsOriginOutsideList) {
  std::vector<Step> steps = {
      Step::PrimitiveFlex(StopId{1}, StopId{2}, 10, TripId{1}),
      Step::PrimitiveFlex(StopId{2}, StopId{1}, 10, TripId{2}),
  };

  StepsAdjacencyList steps_list = MakeAdjacencyList(steps);
  std::vector<WeightedEdge> relaxed_edges = MakeRelaxedEdges(steps_list);
  RelaxedAdjacencyList relaxed =
      MakeRelaxedAdjacencyListFromEdges(relaxed_edges);

  // StopId{7} appears in no edge, so it sits beyond the adjacency list, which
  // only spans ids up to the largest edge endpoint. It must still get a
  // result: itself at distance 0, everything else unreachable.
  std::vector<int> distances = FindShortestRelaxedPaths(relaxed, StopId{7});

  ASSERT_EQ(distances.size(), 8);
  EXPECT_EQ(distances[7], 0);
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(distances[i], std::numeric_limits<int>::max()) << i;
  }
}

}  // namespace vats5
