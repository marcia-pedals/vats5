#include "solver/path_massage.h"

#include <gtest/gtest.h>

#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"

using namespace vats5;

// Build a tiny ProblemState from flex steps and required stops.
static ProblemState MakeTinyProblem(
    const std::vector<Step>& steps,
    ProblemBoundary boundary,
    RequiredStops required,
    std::unordered_map<StopId, ProblemStateStopInfo> stop_infos
) {
  return MakeProblemState(
      MakeAdjacencyList(steps),
      boundary,
      std::move(required),
      std::move(stop_infos),
      /*step_partition_names=*/{},
      /*original_edges=*/{}
  );
}

TEST(PathMassageTest, GreedyExtendPicksUpFreeStop) {
  // Graph: START(0) -> A(1) -> B(2) -> END(3)
  // Also:  A(1) -> END(3) directly (15s)
  //
  // Durations:
  //   START->A: 10s, A->B: 5s, B->END: 10s, A->END: 15s
  //
  // Initial partial solution: tour [START, A, END]
  //   path = START->A->END = 10 + 15 = 25s
  //
  // Adding B: tour [START, A, B, END]
  //   path = START->A->B->END = 10 + 5 + 10 = 25s (same duration!)
  //
  // So greedy extend should pick up B for free.

  StopId START{0}, A{1}, B{2}, END{3};

  std::vector<Step> steps = {
      Step::PrimitiveFlex(START, A, 10, TripId{1}),
      Step::PrimitiveFlex(A, B, 5, TripId{2}),
      Step::PrimitiveFlex(B, END, 10, TripId{3}),
      Step::PrimitiveFlex(A, END, 15, TripId{4}),
  };

  ProblemBoundary boundary{.start = START, .end = END};

  RequiredStops required;
  required.representative[START] = START;
  required.representative[A] = A;
  required.representative[B] = B;
  required.representative[END] = END;

  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos = {
      {START, {GtfsStopId{"S"}, "START"}},
      {A, {GtfsStopId{"A"}, "A"}},
      {B, {GtfsStopId{"B"}, "B"}},
      {END, {GtfsStopId{"E"}, "END"}},
  };

  ProblemState problem = MakeTinyProblem(steps, boundary, required, stop_infos);

  // Build the initial partial solution path: START -> A -> END.
  std::vector<StopId> initial_tour = {START, A, END};
  std::vector<Path> initial_paths =
      ComputeMinimalFeasiblePathsAlong(initial_tour, problem.minimal);
  ASSERT_FALSE(initial_paths.empty());

  // Pick the min-duration path.
  auto best_it = std::ranges::min_element(initial_paths, {}, [](const Path& p) {
    return p.DurationSeconds();
  });
  ASSERT_NE(best_it, initial_paths.end());
  EXPECT_EQ(best_it->DurationSeconds(), 25);

  PartialSolutionPath initial{
      .path = *best_it,
      .subset_tour = initial_tour,
  };

  int initial_count = CountRequiredStops(initial.path, problem.required);

  // Greedy extend should pick up B.
  PartialSolutionPath extended =
      GreedilyExtendAsMuchAsPossibleWithoutIncreasingDuration(problem, initial);

  int extended_count = CountRequiredStops(extended.path, problem.required);
  EXPECT_GT(extended_count, initial_count);
  EXPECT_LE(extended.path.DurationSeconds(), initial.path.DurationSeconds());
}

TEST(PathMassageTest, CountRequiredStopsBasic) {
  StopId A{0}, B{1};
  Step step = Step::PrimitiveFlex(A, B, 10, TripId{1});
  Path path{.merged_step = step, .steps = {step}};

  RequiredStops required;
  required.representative[A] = A;
  required.representative[B] = B;

  EXPECT_EQ(CountRequiredStops(path, required), 2);
}
