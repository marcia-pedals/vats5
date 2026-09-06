#include "solver/branch_and_bound.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>
#include <sys/stat.h>

#include <optional>
#include <unordered_map>
#include <vector>

#include "rapidcheck/Assertions.h"
#include "rapidcheck/Log.h"
#include "solver/concorde.h"
#include "solver/data.h"
#include "solver/tarel_graph.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {

RC_GTEST_PROP(BranchAndBoundTest, BranchPreservesSolutionSpace, ()) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state_orig = *GenProblemState(
      std::nullopt,
      rc::gen::construct<StepPartitionId>(
          rc::gen::inRange(0, num_partitions - 1)
      )
  );
  NamedBranchEdge named_edge = *GenBranchEdge(state_orig);

  ProblemState state_forbid =
      ApplyConstraints(state_orig, {named_edge.edge.Forbid()});
  ProblemState state_require =
      ApplyConstraints(state_orig, {named_edge.edge.Require()});

  int opt_orig = BruteForceSolveOptimalDuration(state_orig);
  int opt_forbid = BruteForceSolveOptimalDuration(state_forbid);
  int opt_require = BruteForceSolveOptimalDuration(state_require);

  RC_LOG() << "opt_orig " << TimeSinceServiceStart{opt_orig} << "\n";
  RC_LOG() << "opt_forbid " << TimeSinceServiceStart{opt_forbid} << "\n";
  RC_LOG() << "opt_require " << TimeSinceServiceStart{opt_require} << "\n";

  RC_ASSERT(opt_orig == std::min(opt_forbid, opt_require));

  // TODO: I think that the branches should be a partition of the original
  // solution space, so it would be nice to assert that every element of the
  // original solution space is in exactly one of the branches. Doing this
  // is tricky because all the solution spaces are infinite (there are
  // infinitely many tours when you are allowed to repeat stops) and our
  // enumeration strategy does not enumerate the same things in the original as
  // in the branches, so it's not as easy as asserting that the "enumerated
  // solution space" for the original is the disjoint union of the spaces for
  // the branches.
}

// Branching must never make the lower bound invalid: each branch's bound has
// to stay at or below that branch's own optimum. Note the bound is NOT
// monotone across branching, in two ways: requiring an edge can re-classify
// extreme vs inner stops (via ComputeExtremeStops), splitting a merged
// journey and losing its waiting time; and slack forwarding's floors and
// forwards shift when branching changes the step groups, so even a forbid
// branch's bound can come out below its parent's. Branch and bound is robust
// to both because it prunes each node against the max of its own bound and
// its parent's.
RC_GTEST_PROP(BranchAndBoundTest, BranchLowerBoundStillValid, ()) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state_orig = *GenProblemState(
      std::nullopt,
      rc::gen::construct<StepPartitionId>(
          rc::gen::inRange(0, num_partitions - 1)
      )
  );
  NamedBranchEdge named_edge = *GenBranchEdge(state_orig);

  ProblemState state_forbid =
      ApplyConstraints(state_orig, {named_edge.edge.Forbid()});
  ProblemState state_require =
      ApplyConstraints(state_orig, {named_edge.edge.Require()});
  RC_LOG() << "state_forbid: " << rc::toString(state_forbid) << "\n\n";
  RC_LOG() << "state_require: " << rc::toString(state_require) << "\n\n";

  std::optional<TspTourResult> result_orig, result_forbid, result_require;
  try {
    result_orig = ComputeTarelLowerBound(state_orig);
    result_forbid = ComputeTarelLowerBound(state_forbid);
    result_require = ComputeTarelLowerBound(state_require);
  } catch (const InvalidTourStructure&) {
    RC_DISCARD("InvalidTourStructure");
  }

  auto LogResult = [&](const ProblemState& state, const TspTourResult& result) {
    RC_LOG() << result.optimal_value << " ";
    RC_LOG() << state.StopName(result.tour_edges[0].origin.stop);
    for (int i = 0; i < result.tour_edges.size(); ++i) {
      RC_LOG() << " -> "
               << state.StopName(result.tour_edges[i].destination.stop);
    }
    RC_LOG() << "\n\n";
  };

  RC_ASSERT(result_orig.has_value());
  RC_LOG() << "result_orig: ";
  LogResult(state_orig, result_orig.value());

  RC_LOG() << "result_forbid: ";
  if (result_forbid.has_value()) {
    LogResult(state_forbid, result_forbid.value());
  } else {
    RC_LOG() << "no solution\n";
  }

  RC_LOG() << "result_require: ";
  if (result_require.has_value()) {
    LogResult(state_require, result_require.value());
  } else {
    RC_LOG() << "no solution\n";
  }

  RC_ASSERT(
      result_orig->optimal_value <= BruteForceSolveOptimalDuration(state_orig)
  );
  // Use if-guards instead of || inside RC_ASSERT, because RC_ASSERT uses
  // expression templates that overload operator||, which does NOT
  // short-circuit in C++.
  if (result_forbid.has_value()) {
    RC_ASSERT(
        result_forbid->optimal_value <=
        BruteForceSolveOptimalDuration(state_forbid)
    );
  }
  if (result_require.has_value()) {
    RC_ASSERT(
        result_require->optimal_value <=
        BruteForceSolveOptimalDuration(state_require)
    );
  }
}

// Regression test for a stop id collision in ApplyConstraints. Requiring an
// edge that was previously forbidden creates a merged stop with no steps,
// which the adjacency list's NumStops() doesn't count. A follow-up
// ApplyConstraints call on that state used to allocate its next merged stop id
// from NumStops(), reusing the step-less stop's id: the second Require below
// then created its merged stop with the same id as its own endpoint, and
// EraseGroup on that endpoint erased the new stop's required group, leaving a
// boundary stop that is not required.
TEST(BranchAndBoundTest, RequireOfStepLessMergedStopKeepsBoundaryRequired) {
  StopId a{0}, b{1}, c{2}, d{3}, start{4}, end{5};
  TripId trip{0};
  std::vector<Step> steps;
  steps.push_back(Step::PrimitiveFlex(a, b, 1200, trip, StepPartitionId{0}));
  steps.push_back(Step::PrimitiveFlex(b, c, 1200, trip, StepPartitionId{0}));
  steps.push_back(Step::PrimitiveFlex(c, d, 1200, trip, StepPartitionId{0}));
  steps.push_back(Step::PrimitiveFlex(d, a, 1200, trip, StepPartitionId{0}));
  for (StopId mid : {a, b, c, d}) {
    steps.push_back(Step::PrimitiveFlex(start, mid, 0, trip));
    steps.push_back(Step::PrimitiveFlex(mid, end, 0, trip));
  }

  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos;
  stop_infos[a] = ProblemStateStopInfo{GtfsStopId{"a"}, "a"};
  stop_infos[b] = ProblemStateStopInfo{GtfsStopId{"b"}, "b"};
  stop_infos[c] = ProblemStateStopInfo{GtfsStopId{"c"}, "c"};
  stop_infos[d] = ProblemStateStopInfo{GtfsStopId{"d"}, "d"};
  stop_infos[start] = ProblemStateStopInfo{GtfsStopId{""}, "START"};
  stop_infos[end] = ProblemStateStopInfo{GtfsStopId{""}, "END"};

  RequiredStops required;
  for (StopId stop : {a, b, c, d, start, end}) {
    required.representative[stop] = stop;
  }

  ProblemState state = MakeProblemState(
      MakeAdjacencyList(steps),
      ProblemBoundary{.start = start, .end = end},
      required,
      stop_infos,
      {},
      {}
  );

  // Applied one at a time: each ApplyConstraints call re-derives its next stop
  // id from the previous call's result, which is what used to collide.
  state = ApplyConstraints(
      state, {ProblemConstraint{ConstraintForbidEdge{start, d}}}
  );
  state = ApplyConstraints(
      state, {ProblemConstraint{ConstraintRequireEdge{start, d}}}
  );
  // The merged stop "(START->d)" is the new boundary start. It has no steps:
  // START->d was just forbidden and nothing enters START.
  StopId merged = state.boundary.start;
  EXPECT_TRUE(state.required.Contains(merged));

  state = ApplyConstraints(
      state, {ProblemConstraint{ConstraintRequireEdge{merged, c}}}
  );
  // The new merged stop "((START->d)->c)" must get a fresh id and stay
  // required.
  EXPECT_NE(state.boundary.start, merged);
  EXPECT_TRUE(state.required.Contains(state.boundary.start));
}

RC_GTEST_PROP(BranchAndBoundTest, SearchFindsOptimalValue, ()) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state = *GenProblemState(
      std::nullopt,
      rc::gen::construct<StepPartitionId>(
          rc::gen::inRange(0, num_partitions - 1)
      )
  );

  try {
    // Pass max_iter to catch infinite loops.
    //
    // I think that in principle it could take the search up to
    //   2^(choose(#stops, 2))
    // steps because there are that many edges and the search could branch on
    // every edge.
    //
    // `state` can have up to 8 stops, so that's 2^28 steps, but in practice the
    // search seems much better than that, so I've set max_iter to 4096. Can
    // increase if we notice flakiness from problem instances that take more
    // steps.
    RC_ASSERT(
        BruteForceSolveOptimalDuration(state) ==
        BranchAndBoundSolve(state, 0, &RC_LOG(), std::nullopt, 4096).best_ub
    );
  } catch (const InvalidTourStructure&) {
    RC_DISCARD("InvalidTourStructure");
  }
}

}  // namespace vats5
