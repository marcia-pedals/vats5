#include "solver/steps_adjacency_list.h"

#include <algorithm>
#include <unordered_map>
#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include "solver/step_merge.h"

namespace vats5 {

static rc::Gen<Step> GenStep() {
  return rc::gen::apply(
      [](int origin_stop, int dest_stop, int origin_time, int duration,
         int origin_trip, int dest_trip, int origin_partition,
         int dest_partition, bool origin_is_flex, bool dest_is_flex,
         bool is_flex) {
        TimeSinceServiceStart ot = is_flex ? TimeSinceServiceStart{0}
                                           : TimeSinceServiceStart{origin_time};
        TimeSinceServiceStart dt = is_flex ? TimeSinceServiceStart{duration}
                                           : TimeSinceServiceStart{origin_time + duration};
        return Step{
            StepEndpoint{StopId{origin_stop}, origin_is_flex, StepPartitionId{origin_partition}, ot, TripId{origin_trip}},
            StepEndpoint{StopId{dest_stop}, dest_is_flex, StepPartitionId{dest_partition}, dt, TripId{dest_trip}},
            is_flex
        };
      },
      rc::gen::inRange(0, 6),
      rc::gen::inRange(0, 6),
      rc::gen::inRange(0, 3600),
      rc::gen::inRange(1, 3600),
      rc::gen::inRange(0, 10),
      rc::gen::inRange(0, 10),
      rc::gen::inRange(-1, 5),
      rc::gen::inRange(-1, 5),
      rc::gen::arbitrary<bool>(),
      rc::gen::arbitrary<bool>(),
      rc::gen::arbitrary<bool>()
  );
}

static bool StepLess(const Step& a, const Step& b) {
  auto key = [](const Step& s) {
    return std::tie(s.origin.stop.v, s.destination.stop.v, s.is_flex,
                    s.origin.time.seconds, s.destination.time.seconds,
                    s.origin.trip.v, s.destination.trip.v,
                    s.origin.partition.v, s.destination.partition.v,
                    s.origin.is_flex, s.destination.is_flex);
  };
  return key(a) < key(b);
}

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

RC_GTEST_PROP(StepsAdjacencyListTest, MakeAdjacencyListAndAllStepsAreInverses, ()) {
  auto steps = *rc::gen::container<std::vector<Step>>(GenStep());

  // Compute expected: group by (origin, dest), sort, and apply MakeMinimalCover.
  std::unordered_map<int, std::unordered_map<int, std::vector<Step>>> grouped;
  for (const Step& step : steps) {
    grouped[step.origin.stop.v][step.destination.stop.v].push_back(step);
  }
  std::vector<Step> expected;
  for (auto& [origin, dest_map] : grouped) {
    for (auto& [dest, group] : dest_map) {
      SortSteps(group);
      MakeMinimalCover(group);
      for (const Step& s : group) {
        expected.push_back(s);
      }
    }
  }

  auto actual = MakeAdjacencyList(steps).AllSteps();

  std::sort(expected.begin(), expected.end(), StepLess);
  std::sort(actual.begin(), actual.end(), StepLess);

  RC_ASSERT(expected == actual);
}

}  // namespace vats5
