#include "solver/tour_paths.h"

#include <gtest/gtest.h>

using namespace vats5;

namespace {

// Helper: build a StepPathsAdjacencyList containing the given paths.
StepPathsAdjacencyList MakeCompleted(const std::vector<Path>& paths) {
  StepPathsAdjacencyList result;
  for (const Path& p : paths) {
    StopId origin = p.merged_step.origin.stop;
    StopId dest = p.merged_step.destination.stop;
    // Find or create the group for this origin->dest pair.
    auto& groups = result.adjacent[origin];
    bool found = false;
    for (auto& group : groups) {
      if (!group.empty() && group[0].merged_step.destination.stop == dest) {
        group.push_back(p);
        found = true;
        break;
      }
    }
    if (!found) {
      groups.push_back({p});
    }
  }
  return result;
}

// Helper: make a scheduled path (a single-step path with fixed times).
Path MakeScheduledPath(
    StopId from, StopId to, int origin_seconds, int dest_seconds
) {
  Step step = Step::PrimitiveScheduled(
      from, to, TimeSinceServiceStart{origin_seconds},
      TimeSinceServiceStart{dest_seconds}, TripId{1}
  );
  return Path{step, {step}};
}

// Helper: make a flex path (a single-step path with flex duration).
Path MakeFlexPath(StopId from, StopId to, int duration_seconds) {
  Step step = Step::PrimitiveFlex(from, to, duration_seconds, TripId{1});
  return Path{step, {step}};
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
  auto completed = MakeCompleted({MakeFlexPath(a, b, 100)});

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
  auto completed = MakeCompleted(
      {MakeScheduledPath(a, b, 100, 200), MakeScheduledPath(b, c, 200, 350)}
  );

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
  auto completed = MakeCompleted({MakeScheduledPath(a, b, -100, 50)});

  std::vector<StopId> stop_sequence = {a, b};
  auto result = ComputeMinDurationFeasiblePaths(stop_sequence, completed);
  // The step has origin.time < 0, so after filtering there are no feasible
  // steps, and we should get empty result.
  EXPECT_TRUE(result.empty());
}
