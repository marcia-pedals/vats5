#include "solver/branch_and_bound.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>
#include <sys/stat.h>
#include <vector>
#include "rapidcheck/Assertions.h"
#include "rapidcheck/Log.h"
#include "solver/data.h"
#include "solver/tarel_graph.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {

RC_GTEST_PROP(BranchAndBoundTest, BranchPreservesSolutionSpace, ()) {
  ProblemState state_orig = *GenProblemState();
  NamedBranchEdge named_edge = *GenBranchEdge(state_orig);

  ProblemState state_forbid = ApplyConstraints(state_orig, {named_edge.edge.Forbid()});
  ProblemState state_require = ApplyConstraints(state_orig, {named_edge.edge.Require()});

  int opt_orig = BruteForceSolveOptimalDuration(state_orig);
  int opt_forbid = BruteForceSolveOptimalDuration(state_forbid);
  int opt_require = BruteForceSolveOptimalDuration(state_require);

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

RC_GTEST_PROP(BranchAndBoundTest, BranchLowerBoundNonIncreasing, ()) {
  ProblemState state_orig = *GenProblemState();
  NamedBranchEdge named_edge = *GenBranchEdge(state_orig);

  ProblemState state_forbid = ApplyConstraints(state_orig, {named_edge.edge.Forbid()});
  ProblemState state_require = ApplyConstraints(state_orig, {named_edge.edge.Require()});
  RC_LOG() << "state_forbid: " << rc::toString(state_forbid) << "\n\n";
  RC_LOG() << "state_require: " << rc::toString(state_require) << "\n\n";

  auto partition_same = [](const Step& s) -> StepPartitionId {
    return StepPartitionId{0};
  };

  std::optional<TspTourResult> result_orig = ComputeTarelLowerBound(state_orig, partition_same);
  std::optional<TspTourResult> result_forbid = ComputeTarelLowerBound(state_forbid, partition_same);
  std::optional<TspTourResult> result_require = ComputeTarelLowerBound(state_require, partition_same);

  auto LogResult = [&](const TspTourResult& result) {
    RC_LOG() << result.optimal_value << " ";
    for (int i = 0; i < result.original_stop_tour.size(); ++i) {
      if (i > 0) {
        RC_LOG() << " -> ";
      }
      RC_LOG() << state_require.StopName(result.original_stop_tour[i]);
    }
    RC_LOG() << "\n\n";
  };

  RC_ASSERT(result_orig.has_value());
  RC_LOG() << "result_orig: ";
  LogResult(result_orig.value());

  RC_LOG() << "result_forbid: ";
  if (result_forbid.has_value()) {
    LogResult(result_forbid.value());
  } else {
    RC_LOG() << "no solution\n";
  }

  RC_LOG() << "result_require: ";
  if (result_require.has_value()) {
    LogResult(result_require.value());
  } else {
    RC_LOG() << "no solution\n";
  }

  RC_ASSERT(!result_forbid.has_value() || result_forbid->optimal_value >= result_orig->optimal_value);
  RC_ASSERT(!result_require.has_value() || result_require->optimal_value >= result_orig->optimal_value);
}

}  // namespace vats5
