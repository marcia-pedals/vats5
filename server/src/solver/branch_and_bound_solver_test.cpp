#include "solver/branch_and_bound_solver.h"

#include <gtest/gtest.h>

namespace vats5 {

TEST(BranchAndBoundSolverTest, RemapStopIdsBasic) {
  // Create a small adjacency list with sparse stop IDs:
  // Stop 10 -> Stop 20 (two steps)
  // Stop 10 -> Stop 30 (one step)
  // Stop 20 -> Stop 30 (one step)
  //
  // Used stops: 10, 20, 30 -> should become 0, 1, 2

  std::vector<Step> steps = {
      Step{
          .origin_stop = StopId{10},
          .destination_stop = StopId{20},
          .origin_time = TimeSinceServiceStart{100},
          .destination_time = TimeSinceServiceStart{200},
          .origin_trip = TripId{1},
          .destination_trip = TripId{1},
          .is_flex = false
      },
      Step{
          .origin_stop = StopId{10},
          .destination_stop = StopId{20},
          .origin_time = TimeSinceServiceStart{150},
          .destination_time = TimeSinceServiceStart{250},
          .origin_trip = TripId{2},
          .destination_trip = TripId{2},
          .is_flex = false
      },
      Step{
          .origin_stop = StopId{10},
          .destination_stop = StopId{30},
          .origin_time = TimeSinceServiceStart{100},
          .destination_time = TimeSinceServiceStart{300},
          .origin_trip = TripId{3},
          .destination_trip = TripId{3},
          .is_flex = false
      },
      Step{
          .origin_stop = StopId{20},
          .destination_stop = StopId{30},
          .origin_time = TimeSinceServiceStart{250},
          .destination_time = TimeSinceServiceStart{350},
          .origin_trip = TripId{4},
          .destination_trip = TripId{4},
          .is_flex = false
      },
  };

  StepsAdjacencyList original = MakeAdjacencyList(steps);
  ASSERT_EQ(original.NumStops(), 31);  // 0..30 inclusive

  RemappedAdjacencyList remapped = RemapStopIds(original);

  // Should have exactly 3 stops now
  EXPECT_EQ(remapped.adj.NumStops(), 3);

  // Check bidirectional mappings exist for all 3 original stops
  EXPECT_EQ(remapped.new_to_original.size(), 3);
  EXPECT_EQ(remapped.original_to_new.size(), 3);

  // Check that mappings are consistent
  for (int old_id : {10, 20, 30}) {
    ASSERT_TRUE(remapped.original_to_new.contains(old_id));
    StopId new_id = remapped.original_to_new.at(old_id);
    EXPECT_GE(new_id.v, 0);
    EXPECT_LT(new_id.v, 3);
    EXPECT_EQ(remapped.new_to_original[new_id.v].v, old_id);
  }

  // Check that the graph structure is preserved:
  // The new stop corresponding to original 10 should have 2 destination groups
  StopId new_10 = remapped.original_to_new.at(10);
  auto groups_from_10 = remapped.adj.GetGroups(new_10);
  EXPECT_EQ(groups_from_10.size(), 2);

  // The new stop corresponding to original 20 should have 1 destination group
  StopId new_20 = remapped.original_to_new.at(20);
  auto groups_from_20 = remapped.adj.GetGroups(new_20);
  EXPECT_EQ(groups_from_20.size(), 1);

  // The new stop corresponding to original 30 should have 0 destination groups
  StopId new_30 = remapped.original_to_new.at(30);
  auto groups_from_30 = remapped.adj.GetGroups(new_30);
  EXPECT_EQ(groups_from_30.size(), 0);

  // Check that destination stops are remapped correctly
  StopId new_20_expected = remapped.original_to_new.at(20);
  StopId new_30_expected = remapped.original_to_new.at(30);

  // Find destinations from new_10
  bool found_20 = false, found_30 = false;
  for (const auto& group : groups_from_10) {
    if (group.destination_stop.v == new_20_expected.v) found_20 = true;
    if (group.destination_stop.v == new_30_expected.v) found_30 = true;
  }
  EXPECT_TRUE(found_20);
  EXPECT_TRUE(found_30);

  // Check step count is preserved
  EXPECT_EQ(remapped.adj.steps.size(), original.steps.size());
}

}  // namespace vats5
