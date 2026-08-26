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

  if (result.best_tour.empty()) {
    return;
  }
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

  CheckSolvesOptimally(state);
}

}  // namespace vats5
