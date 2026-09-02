#include "solver/two_opt.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "rapidcheck/Log.h"
#include "solver/held_karp_dp.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"
#include "solver/tour_paths.h"

namespace vats5 {

namespace {

// Builds a ProblemState with middle stops 0..num_middle_stops-1 and the given
// steps between them, adding the boundary and making every stop its own
// required group.
ProblemState MakeTestState(int num_middle_stops, std::vector<Step> steps) {
  std::unordered_set<StopId> stops;
  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos;
  RequiredStops required;
  for (int i = 0; i < num_middle_stops; ++i) {
    StopId stop{i};
    stops.insert(stop);
    stop_infos[stop] = ProblemStateStopInfo{
        GtfsStopId{std::string(1, 'a' + i)}, std::string(1, 'a' + i)
    };
    required.representative[stop] = stop;
  }

  ProblemBoundary boundary{
      .start = StopId{num_middle_stops},
      .end = StopId{num_middle_stops + 1},
  };
  AddBoundary(steps, stops, stop_infos, boundary);
  required.representative[boundary.start] = boundary.start;
  required.representative[boundary.end] = boundary.end;

  return MakeProblemState(
      MakeAdjacencyList(steps),
      boundary,
      std::move(required),
      std::move(stop_infos),
      {},
      {}
  );
}

// Checks that the returned tour actually achieves best_val on the minimal
// graph. This cross-validates the solver's cover-based evaluator against the
// path-based one.
void CheckTourAchievesBestVal(
    const ProblemState& state, const TwoOptResult& result
) {
  std::vector<Path> paths =
      ComputeMinimalFeasiblePathsAlong(result.best_tour, state.minimal);
  ASSERT_FALSE(paths.empty());
  int best_duration = std::numeric_limits<int>::max();
  for (const Path& path : paths) {
    best_duration = std::min(best_duration, path.DurationSeconds());
  }
  EXPECT_EQ(best_duration, result.best_val);
}

TEST(TwoOptTest, MatchesHeldKarpOnDenseScheduledInstance) {
  std::vector<Step> steps;
  int trip = 0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (i == j) {
        continue;
      }
      for (int k = 0; k < 20; ++k) {
        int dep = k * 600 + i * 137 + j * 71;
        steps.push_back(
            Step::PrimitiveScheduled(
                StopId{i},
                StopId{j},
                TimeSinceServiceStart{dep},
                TimeSinceServiceStart{dep + 300 + ((i + j) % 3) * 60},
                TripId{trip++}
            )
        );
      }
    }
  }
  ProblemState state = MakeTestState(4, std::move(steps));

  HeldKarpDPResult hk = HeldKarpDPSolve(state, 0);
  ASSERT_LT(hk.best_val, std::numeric_limits<int>::max());

  TwoOptResult result = TwoOptSolve(state, {}, &std::cerr);
  EXPECT_EQ(result.best_val, hk.best_val);
  CheckTourAchievesBestVal(state, result);
}

TEST(TwoOptTest, HandlesFlexOnlyMiddleSteps) {
  // Unlike the GLKH encoding, 2-opt evaluates on the real completed graph and
  // handles flex steps between middle stops.
  std::vector<Step> steps;
  steps.push_back(Step::PrimitiveFlex(StopId{0}, StopId{1}, 100, TripId{0}));
  steps.push_back(Step::PrimitiveFlex(StopId{1}, StopId{0}, 150, TripId{1}));
  ProblemState state = MakeTestState(2, std::move(steps));

  HeldKarpDPResult hk = HeldKarpDPSolve(state, 0);
  ASSERT_LT(hk.best_val, std::numeric_limits<int>::max());

  TwoOptResult result = TwoOptSolve(state, {}, &std::cerr);
  EXPECT_EQ(result.best_val, hk.best_val);
  CheckTourAchievesBestVal(state, result);
}

RC_GTEST_PROP(TwoOptTest, IsSoundOnRandomStates, ()) {
  ProblemState state = *GenProblemState();

  TwoOptOptions options;
  options.restarts = 5;
  TwoOptResult result = TwoOptSolve(state, options, &RC_LOG());
  if (result.best_val == std::numeric_limits<int>::max()) {
    return;
  }

  // The tour must achieve best_val, and best_val can't beat the brute-force
  // optimum.
  RC_ASSERT(result.best_val >= BruteForceSolveOptimalDuration(state));
  std::vector<Path> paths =
      ComputeMinimalFeasiblePathsAlong(result.best_tour, state.minimal);
  RC_ASSERT(!paths.empty());
  int best_duration = std::numeric_limits<int>::max();
  for (const Path& path : paths) {
    best_duration = std::min(best_duration, path.DurationSeconds());
  }
  RC_ASSERT(best_duration == result.best_val);
}

}  // namespace

}  // namespace vats5
