#include "solver/held_karp_dp.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include "rapidcheck/Log.h"
#include "solver/branch_and_bound.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"
#include "solver/tour_paths.h"

namespace vats5 {

namespace {

// Checks that `result.best_val` is the brute-force optimum of `state` and that
// the returned path actually achieves it.
void CheckSolvesOptimally(const ProblemState& state) {
  HeldKarpDPResult result = HeldKarpDPSolve(state, &RC_LOG());
  RC_ASSERT(BruteForceSolveOptimalDuration(state) == result.best_val);

  if (result.best_path.empty()) {
    return;
  }
  std::vector<StopId> tour;
  for (const HeldKarpDPPathPoint& point : result.best_path) {
    tour.push_back(point.stop);
  }
  std::vector<Path> paths =
      ComputeMinimalFeasiblePathsAlong(tour, state.minimal);
  RC_ASSERT(!paths.empty());
  int best_duration = std::numeric_limits<int>::max();
  for (const Path& path : paths) {
    best_duration = std::min(best_duration, path.DurationSeconds());
  }
  RC_ASSERT(best_duration == result.best_val);
}

}  // namespace

RC_GTEST_PROP(HeldKarpDpTest, SolveFindsOptimalValue, ()) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state = *GenProblemState(
      std::nullopt,
      rc::gen::construct<StepPartitionId>(
          rc::gen::inRange(0, num_partitions - 1)
      )
  );

  CheckSolvesOptimally(state);
}

RC_GTEST_PROP(HeldKarpDpTest, SolveFindsOptimalValueOnConstrainedState, ()) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state = *GenProblemState(
      std::nullopt,
      rc::gen::construct<StepPartitionId>(
          rc::gen::inRange(0, num_partitions - 1)
      )
  );

  int num_constraints = *rc::gen::inRange(1, 5);
  for (int i = 0; i < num_constraints; ++i) {
    NamedBranchEdge named_edge = *GenBranchEdge(state);
    bool require = *rc::gen::arbitrary<bool>();
    state = ApplyConstraints(
        state,
        {require ? ProblemConstraint{named_edge.edge.Require()}
                 : ProblemConstraint{named_edge.edge.Forbid()}}
    );
  }
  if (state.required.Representative(state.boundary.start) ==
      state.required.Representative(state.boundary.end)) {
    RC_DISCARD("constraints merged START and END into one group");
  }
  // The brute-force oracle (via ComputeCompletedGraph) indexes out of bounds
  // when a required stop appears in no step of `minimal`, which chained forbid
  // constraints can cause. Such states can't be checked against it.
  std::unordered_set<StopId> stops_with_steps;
  for (const Step& step : state.minimal.AllSteps()) {
    stops_with_steps.insert(step.origin.stop);
    stops_with_steps.insert(step.destination.stop);
  }
  for (StopId stop : state.required.AllFlat()) {
    if (!stops_with_steps.contains(stop)) {
      RC_DISCARD("constraints left a required stop with no steps");
    }
  }
  if (!stops_with_steps.contains(state.boundary.start) ||
      !stops_with_steps.contains(state.boundary.end)) {
    RC_DISCARD("constraints left a boundary stop with no steps");
  }

  CheckSolvesOptimally(state);
}

}  // namespace vats5
