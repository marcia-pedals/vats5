#include "solver/relaxed_adjacency_list.h"

#include <gtest/gtest.h>

#include "solver/test_util/cached_test_data.h"
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

TEST(RelaxedAdjacencyListTest, MakeRelaxedAdjacencyListBasic) {
  // Create steps with multiple trips between same stops, different durations.
  // Stop 1 -> Stop 2: durations 100 (fixed), 80 (fixed), 120 (flex)
  // Stop 1 -> Stop 3: duration 50 (fixed)
  // Stop 2 -> Stop 3: duration 30 (flex)
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}),  // duration 100
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{300}, TimeSinceServiceStart{380}, TripId{2}),  // duration 80
      Step::PrimitiveFlex(StopId{1}, StopId{2}, 120, TripId{3}),  // flex, duration 120
      Step::PrimitiveScheduled(StopId{1}, StopId{3}, TimeSinceServiceStart{500}, TimeSinceServiceStart{550}, TripId{4}),  // duration 50
      Step::PrimitiveFlex(StopId{2}, StopId{3}, 30, TripId{5}),  // flex, duration 30
  };

  StepsAdjacencyList steps_list = MakeAdjacencyList(steps);
  std::vector<WeightedEdge> relaxed_edges = MakeRelaxedEdges(steps_list);
  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(relaxed_edges);

  // Check structure
  EXPECT_EQ(relaxed.NumStops(), 4);  // Stops 0, 1, 2, 3

  // Stop 0 has no edges
  EXPECT_TRUE(relaxed.GetEdges(StopId{0}).empty());

  // Stop 1 has edges to Stop 2 (min duration 80) and Stop 3 (duration 50)
  auto edges_1 = relaxed.GetEdges(StopId{1});
  EXPECT_EQ(edges_1.size(), 2);
  // Edges are sorted by destination stop ID
  EXPECT_EQ(edges_1[0].destination_stop, StopId{2});
  EXPECT_EQ(edges_1[0].weight_seconds, 80);  // min of 100, 80, 120
  EXPECT_EQ(edges_1[1].destination_stop, StopId{3});
  EXPECT_EQ(edges_1[1].weight_seconds, 50);

  // Stop 2 has edge to Stop 3 (duration 30)
  auto edges_2 = relaxed.GetEdges(StopId{2});
  EXPECT_EQ(edges_2.size(), 1);
  EXPECT_EQ(edges_2[0].destination_stop, StopId{3});
  EXPECT_EQ(edges_2[0].weight_seconds, 30);

  // Stop 3 has no edges
  EXPECT_TRUE(relaxed.GetEdges(StopId{3}).empty());

  // Test GetWeight
  EXPECT_EQ(relaxed.GetWeight(StopId{1}, StopId{2}), 80);
  EXPECT_EQ(relaxed.GetWeight(StopId{1}, StopId{3}), 50);
  EXPECT_EQ(relaxed.GetWeight(StopId{2}, StopId{3}), 30);
  EXPECT_EQ(relaxed.GetWeight(StopId{0}, StopId{1}), std::nullopt);  // No edge
  EXPECT_EQ(relaxed.GetWeight(StopId{3}, StopId{1}), std::nullopt);  // No edge
  EXPECT_EQ(relaxed.GetWeight(StopId{1}, StopId{1}), std::nullopt);  // Self-loop
}

TEST(RelaxedAdjacencyListTest, MakeRelaxedAdjacencyListFromWeightedEdges) {
  std::vector<WeightedEdge> edges = {
      WeightedEdge{StopId{0}, StopId{1}, 100},
      WeightedEdge{StopId{0}, StopId{2}, 200},
      WeightedEdge{StopId{1}, StopId{2}, 50},
      WeightedEdge{StopId{2}, StopId{0}, 150},
  };

  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(edges);

  EXPECT_EQ(relaxed.NumStops(), 3);

  // Stop 0 has edges to Stop 1 and Stop 2
  EXPECT_EQ(relaxed.GetWeight(StopId{0}, StopId{1}), 100);
  EXPECT_EQ(relaxed.GetWeight(StopId{0}, StopId{2}), 200);

  // Stop 1 has edge to Stop 2
  EXPECT_EQ(relaxed.GetWeight(StopId{1}, StopId{2}), 50);
  EXPECT_EQ(relaxed.GetWeight(StopId{1}, StopId{0}), std::nullopt);

  // Stop 2 has edge to Stop 0
  EXPECT_EQ(relaxed.GetWeight(StopId{2}, StopId{0}), 150);
  EXPECT_EQ(relaxed.GetWeight(StopId{2}, StopId{1}), std::nullopt);
}

TEST(RelaxedAdjacencyListTest, MakeRelaxedAdjacencyListFromWeightedEdgesEmpty) {
  std::vector<WeightedEdge> edges = {};
  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(edges);
  EXPECT_EQ(relaxed.NumStops(), 0);
}

TEST(RelaxedAdjacencyListTest, MakeRelaxedAdjacencyListFromBART) {
  const auto test_data = GetCachedTestData("../data/RG_20250718_BA");
  std::vector<WeightedEdge> relaxed_edges =
      MakeRelaxedEdges(test_data.adjacency_list);
  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(relaxed_edges);

  // Find Berkeley and North Berkeley stops
  StopId berkeley =
      test_data.steps_from_gtfs.mapping.GetStopIdFromName("Downtown Berkeley");
  StopId north_berkeley =
      test_data.steps_from_gtfs.mapping.GetStopIdFromName("North Berkeley");

  // Find the edge from Berkeley to North Berkeley
  auto edges = relaxed.GetEdges(berkeley);
  const RelaxedEdge* edge_to_north_berkeley = nullptr;
  for (const auto& edge : edges) {
    if (edge.destination_stop == north_berkeley) {
      edge_to_north_berkeley = &edge;
      break;
    }
  }

  ASSERT_NE(edge_to_north_berkeley, nullptr)
      << "No edge from Berkeley to North Berkeley";
  EXPECT_EQ(edge_to_north_berkeley->weight_seconds, 120);
}

}  // namespace vats5
