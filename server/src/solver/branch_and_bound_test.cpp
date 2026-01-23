#include "solver/branch_and_bound.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>
#include <unordered_set>
#include "rapidcheck/Assertions.h"
#include "rapidcheck/gen/Predicate.h"
#include "rapidcheck/gen/Select.h"
#include "solver/tarel_graph.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {

RC_GTEST_PROP(BranchAndBoundTest, BranchPreservesOptimalValue, ()) {
  ProblemState state = *GenProblemState();

  std::unordered_set<StopId> non_boundary = state.required_stops;
  non_boundary.erase(state.boundary.start);
  non_boundary.erase(state.boundary.end);

  StopId a = *rc::gen::elementOf(non_boundary);
  StopId b = *rc::gen::distinctFrom(rc::gen::elementOf(non_boundary), a);

  ProblemState state_forbid = ApplyConstraints(state, {ConstraintForbidEdge(a, b)});
  ProblemState state_require = ApplyConstraints(state, {ConstraintRequireEdge(a, b)});

  auto result_orig = NaiveSolve(state);
  auto result_forbid = NaiveSolve(state_forbid);
  auto result_require = NaiveSolve(state_require);

  int opt_orig = result_orig.value;
  int opt_forbid = result_forbid.value;
  int opt_require = result_require.value;

  auto log_perms = [&](const char* label, const ProblemState& s, const NaiveSolveResult& result) {
    RC_LOG() << label << " (opt=" << result.value << "):\n";
    showValue(s, RC_LOG());
    RC_LOG() << "\n  optimal permutations:\n";
    for (const auto& perm : result.optimal_permutations) {
      RC_LOG() << "    ";
      for (size_t i = 0; i < perm.stops.size(); ++i) {
        if (i > 0) RC_LOG() << " -> ";
        RC_LOG() << s.stop_names.at(perm.stops[i]);
      }
      RC_LOG() << " [" << perm.origin_time.ToString() << " -> " << perm.destination_time.ToString() << "]\n";
    }
    RC_LOG() << "\n";
  };
  log_perms("[state]", state, result_orig);
  RC_LOG() << "branch on " << state.StopName(a) << " -> " << state.StopName(b) << "\n";
  log_perms("[state_forbid]", state_forbid, result_forbid);
  log_perms("[state_require]", state_require, result_require);

  RC_ASSERT(opt_orig == std::min(opt_forbid, opt_require));
}

}  // namespace vats5
