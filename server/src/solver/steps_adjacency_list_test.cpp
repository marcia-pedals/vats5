#include "solver/steps_adjacency_list.h"

#include <gtest/gtest.h>

namespace vats5 {

TEST(StepsAdjacencyListTest, MakeAdjacencyListBasic) {
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{250}, TripId{2})
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

TEST(StepsAdjacencyListTest, RemapStopIdsBasic) {
  // Create a sparse adjacency list: stops 10 -> 50, 10 -> 100
  // This will have NumStops() = 101 but only 3 stops actually used
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{10}, StopId{50}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}),
      Step::PrimitiveScheduled(StopId{10}, StopId{100}, TimeSinceServiceStart{300}, TimeSinceServiceStart{400}, TripId{2}),
  };

  StepsAdjacencyList original = MakeAdjacencyList(steps);
  ASSERT_EQ(original.NumStops(), 101);  // max stop id + 1

  CompactStopIdsResult remapped = CompactStopIds(original);

  // Should have exactly 3 stops now: 10, 50, 100 -> 0, 1, 2
  EXPECT_EQ(remapped.list.NumStops(), 3);

  // Check the mapping
  EXPECT_EQ(remapped.mapping.new_to_original.size(), 3);
  EXPECT_EQ(remapped.mapping.new_to_original[0], StopId{10});
  EXPECT_EQ(remapped.mapping.new_to_original[1], StopId{50});
  EXPECT_EQ(remapped.mapping.new_to_original[2], StopId{100});

  EXPECT_EQ(remapped.mapping.original_to_new[10], StopId{0});
  EXPECT_EQ(remapped.mapping.original_to_new[50], StopId{1});
  EXPECT_EQ(remapped.mapping.original_to_new[100], StopId{2});

  // Check that remapped stop 0 (original 10) has 2 groups (to stops 1 and 2)
  auto groups = remapped.list.GetGroups(StopId{0});
  EXPECT_EQ(groups.size(), 2);

  // Groups should be sorted by destination, so first is to new stop 1 (orig 50)
  EXPECT_EQ(groups[0].destination_stop, StopId{1});
  EXPECT_EQ(groups[1].destination_stop, StopId{2});

  // Check that the steps are preserved
  auto steps_to_50 = remapped.list.GetSteps(groups[0]);
  ASSERT_EQ(steps_to_50.size(), 1);
  EXPECT_EQ(steps_to_50[0].origin_time.seconds, 100);
  EXPECT_EQ(steps_to_50[0].destination_time.seconds, 200);

  auto steps_to_100 = remapped.list.GetSteps(groups[1]);
  ASSERT_EQ(steps_to_100.size(), 1);
  EXPECT_EQ(steps_to_100[0].origin_time.seconds, 300);
  EXPECT_EQ(steps_to_100[0].destination_time.seconds, 400);
}

}  // namespace vats5
