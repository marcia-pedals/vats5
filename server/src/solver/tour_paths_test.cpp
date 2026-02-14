#include "solver/tour_paths.h"

#include <gtest/gtest.h>

#include <unordered_set>

#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"

using namespace vats5;

namespace {

// Helper: build a completed StepPathsAdjacencyList from raw steps and stops,
// using the same pipeline as production code.
StepPathsAdjacencyList MakeCompleted(
    const std::vector<Step>& steps,
    const std::unordered_set<StopId>& stops
) {
  return ReduceToMinimalSystemPaths(MakeAdjacencyList(steps), stops, true);
}

}  // namespace

TEST(TourPathsTest, EmptySequenceReturnsEmpty) {
  StepPathsAdjacencyList completed;
  std::vector<StopId> stop_sequence;
  auto result = ComputeMinDurationFeasiblePaths(stop_sequence, completed);
  EXPECT_TRUE(result.empty());
}

TEST(TourPathsTest, SingleStopReturnsEmpty) {
  StepPathsAdjacencyList completed;
  std::vector<StopId> stop_sequence = {StopId{1}};
  auto result = ComputeMinDurationFeasiblePaths(stop_sequence, completed);
  EXPECT_TRUE(result.empty());
}

TEST(TourPathsTest, SingleEdgeFlexPath) {
  StopId a{1}, b{2};
  std::vector<Step> steps = {
      Step::PrimitiveFlex(a, b, 100, TripId{1}),
  };
  auto completed = MakeCompleted(steps, {a, b});

  std::vector<StopId> stop_sequence = {a, b};
  auto result = ComputeMinDurationFeasiblePaths(stop_sequence, completed);
  ASSERT_EQ(result.size(), 1);
  EXPECT_EQ(result[0].merged_step.origin.stop, a);
  EXPECT_EQ(result[0].merged_step.destination.stop, b);
  EXPECT_EQ(result[0].DurationSeconds(), 100);
}

TEST(TourPathsTest, MultiHopSelectsMinDuration) {
  StopId a{1}, b{2}, c{3};
  // a->b: depart 100, arrive 200 (duration 100)
  // b->c: depart 200, arrive 350 (duration 150)
  // Total: 250
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(
          a, b, TimeSinceServiceStart{100}, TimeSinceServiceStart{200},
          TripId{1}
      ),
      Step::PrimitiveScheduled(
          b, c, TimeSinceServiceStart{200}, TimeSinceServiceStart{350},
          TripId{2}
      ),
  };
  auto completed = MakeCompleted(steps, {a, b, c});

  std::vector<StopId> stop_sequence = {a, b, c};
  auto result = ComputeMinDurationFeasiblePaths(stop_sequence, completed);
  ASSERT_GE(result.size(), 1);
  EXPECT_EQ(result[0].DurationSeconds(), 250);
}

TEST(TourPathsTest, NoFeasiblePathReturnsEmpty) {
  StopId a{1}, b{2};
  // No paths between a and b in completed.
  StepPathsAdjacencyList completed;

  std::vector<StopId> stop_sequence = {a, b};
  auto result = ComputeMinDurationFeasiblePaths(stop_sequence, completed);
  EXPECT_TRUE(result.empty());
}

TEST(TourPathsTest, FiltersNegativeTime) {
  StopId a{1}, b{2};
  // A step with negative origin time should be filtered out.
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(
          a, b, TimeSinceServiceStart{-100}, TimeSinceServiceStart{50},
          TripId{1}
      ),
  };
  auto completed = MakeCompleted(steps, {a, b});

  std::vector<StopId> stop_sequence = {a, b};
  auto result = ComputeMinDurationFeasiblePaths(stop_sequence, completed);
  // The step has origin.time < 0, so after filtering there are no feasible
  // steps, and we should get empty result.
  EXPECT_TRUE(result.empty());
}

TEST(TourPathsTest, MultiplePathsOnlyMinDurationReturned) {
  StopId a{1}, b{2};
  // Two scheduled paths a->b with different durations.
  // Path 1: depart 100, arrive 200 (duration 100) — min
  // Path 2: depart 300, arrive 500 (duration 200) — not min
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(
          a, b, TimeSinceServiceStart{100}, TimeSinceServiceStart{200},
          TripId{1}
      ),
      Step::PrimitiveScheduled(
          a, b, TimeSinceServiceStart{300}, TimeSinceServiceStart{500},
          TripId{2}
      ),
  };
  auto completed = MakeCompleted(steps, {a, b});

  std::vector<StopId> stop_sequence = {a, b};
  auto result = ComputeMinDurationFeasiblePaths(stop_sequence, completed);
  ASSERT_GE(result.size(), 1);
  // All returned paths should have the minimum duration (100).
  for (const auto& path : result) {
    EXPECT_EQ(path.DurationSeconds(), 100);
  }
}

TEST(TourPathsTest, FlexStepUsedInPath) {
  StopId a{1}, b{2}, c{3};
  // a->b: flex with 60s duration (can depart any time)
  // b->c: scheduled depart 200, arrive 300
  // Since flex can depart any time, it departs at 140, arrives 200, then
  // takes the scheduled b->c.
  // Total: depart 140, arrive 300 = 160s.
  std::vector<Step> steps = {
      Step::PrimitiveFlex(a, b, 60, TripId{1}),
      Step::PrimitiveScheduled(
          b, c, TimeSinceServiceStart{200}, TimeSinceServiceStart{300},
          TripId{2}
      ),
  };
  auto completed = MakeCompleted(steps, {a, b, c});

  std::vector<StopId> stop_sequence = {a, b, c};
  auto result = ComputeMinDurationFeasiblePaths(stop_sequence, completed);
  ASSERT_GE(result.size(), 1);
  EXPECT_EQ(result[0].DurationSeconds(), 160);
}

TEST(TourPathsTest, FlexStepFilteredWhenRequiresNegativeStart) {
  StopId a{1}, b{2}, c{3};
  // a->b: flex with 120s duration
  // b->c: scheduled depart 100, arrive 200
  // To catch b->c at t=100, flex a->b must depart at t=100-120 = t=-20.
  // That's before 0, so it gets filtered.
  // No other paths exist, so result is empty.
  std::vector<Step> steps = {
      Step::PrimitiveFlex(a, b, 120, TripId{1}),
      Step::PrimitiveScheduled(
          b, c, TimeSinceServiceStart{100}, TimeSinceServiceStart{200},
          TripId{2}
      ),
  };
  auto completed = MakeCompleted(steps, {a, b, c});

  std::vector<StopId> stop_sequence = {a, b, c};
  auto result = ComputeMinDurationFeasiblePaths(stop_sequence, completed);
  EXPECT_TRUE(result.empty());
}
