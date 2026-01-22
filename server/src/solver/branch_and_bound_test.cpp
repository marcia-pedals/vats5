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

  ProblemState state_left = ApplyConstraints(state, {ConstraintForbidEdge(a, b)});
  ProblemState state_right = ApplyConstraints(state, {ConstraintRequireEdge(a, b)});

  int opt_orig = NaiveSolve(state);
  int opt_left = NaiveSolve(state_left);
  int opt_right = NaiveSolve(state_right);

  RC_ASSERT(opt_orig == std::min(opt_left, opt_right));
}

}  // namespace vats5
