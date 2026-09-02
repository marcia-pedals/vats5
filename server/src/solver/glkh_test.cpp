#include "solver/glkh.h"

#include <gtest/gtest.h>

#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "solver/held_karp_dp.h"
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
// graph.
void CheckTourAchievesBestVal(
    const ProblemState& state, const GlkhSolution& solution
) {
  std::vector<Path> paths =
      ComputeMinimalFeasiblePathsAlong(solution.best_tour, state.minimal);
  ASSERT_FALSE(paths.empty());
  int best_duration = std::numeric_limits<int>::max();
  for (const Path& path : paths) {
    best_duration = std::min(best_duration, path.DurationSeconds());
  }
  EXPECT_EQ(best_duration, solution.best_val);
}

TEST(GlkhTest, TwoStopsScheduledOnly) {
  // The best tour is START -> a -> b -> END using the 17:00 departure from a,
  // for a duration of 60 seconds.
  std::vector<Step> steps;
  steps.push_back(
      Step::PrimitiveScheduled(
          StopId{0},
          StopId{1},
          TimeSinceServiceStart{1000},
          TimeSinceServiceStart{1100},
          TripId{0}
      )
  );
  steps.push_back(
      Step::PrimitiveScheduled(
          StopId{0},
          StopId{1},
          TimeSinceServiceStart{5000},
          TimeSinceServiceStart{5060},
          TripId{1}
      )
  );
  steps.push_back(
      Step::PrimitiveScheduled(
          StopId{1},
          StopId{0},
          TimeSinceServiceStart{2000},
          TimeSinceServiceStart{2300},
          TripId{2}
      )
  );
  ProblemState state = MakeTestState(2, std::move(steps));

  std::optional<GlkhSolution> solution =
      SolveGtspWithGlkh(state, {}, &std::cerr);
  ASSERT_TRUE(solution.has_value());
  EXPECT_EQ(solution->best_val, 60);
  EXPECT_EQ(solution->gtsp_value, 60);
  ASSERT_EQ(solution->best_tour.size(), 4);
  EXPECT_EQ(solution->best_tour[1], StopId{0});
  EXPECT_EQ(solution->best_tour[2], StopId{1});
  CheckTourAchievesBestVal(state, *solution);
}

TEST(GlkhTest, FlexOnlyMiddleStepsAreInfeasibleInEncoding) {
  // The only steps between middle stops are flex, which the GLKH encoding
  // excludes, so it finds no tour even though one exists.
  std::vector<Step> steps;
  steps.push_back(Step::PrimitiveFlex(StopId{0}, StopId{1}, 100, TripId{0}));
  steps.push_back(Step::PrimitiveFlex(StopId{1}, StopId{0}, 100, TripId{1}));
  ProblemState state = MakeTestState(2, std::move(steps));

  EXPECT_EQ(SolveGtspWithGlkh(state, {}, &std::cerr), std::nullopt);
  EXPECT_LT(HeldKarpDPSolve(state, 0).best_val, 1000);
}

TEST(GlkhTest, MatchesHeldKarpOnDenseScheduledInstance) {
  // Four middle stops with periodic scheduled service between every ordered
  // pair. GLKH should match the Held-Karp optimum.
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

  std::optional<GlkhSolution> solution =
      SolveGtspWithGlkh(state, {}, &std::cerr);
  ASSERT_TRUE(solution.has_value());
  EXPECT_EQ(solution->best_val, hk.best_val);
  CheckTourAchievesBestVal(state, *solution);
}

}  // namespace

}  // namespace vats5
