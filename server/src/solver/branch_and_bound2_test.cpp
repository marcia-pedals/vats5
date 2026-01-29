#include "solver/branch_and_bound2.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>
#include <algorithm>
#include <sstream>
#include <vector>
#include "rapidcheck/Assertions.h"
#include "rapidcheck/Log.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {

TEST(BranchAndBound2Test, DummyTest) {
  EXPECT_EQ(DummyFunction(), 42);
}

RC_GTEST_PROP(BranchAndBound2Test, BranchPreservesSolutionSpace, ()) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state_orig = *GenProblemState(std::nullopt, rc::gen::construct<StepPartitionId>(rc::gen::inRange(0, num_partitions - 1)));

  // Helper to find optimal solution and log it.
  auto FindAndLogOptimal = [](const ProblemState& state, const std::string& label) -> int {
    std::vector<SolutionSpaceElement> solutions = EnumerateSolutionSpace(state);
    if (solutions.empty()) {
      RC_LOG() << label << ": no solution\n";
      return std::numeric_limits<int>::max();
    }

    auto best = std::min_element(solutions.begin(), solutions.end(),
      [](const SolutionSpaceElement& a, const SolutionSpaceElement& b) {
        return a.merged_step.DurationSeconds() < b.merged_step.DurationSeconds();
      });

    RC_LOG() << label << ": " << best->merged_step.DurationSeconds() << " [";
    for (size_t i = 0; i < best->actual_path.size(); ++i) {
      if (i > 0) RC_LOG() << " -> ";
      RC_LOG() << state.StopName(best->actual_path[i]);
    }
    RC_LOG() << "]\n";

    return best->merged_step.DurationSeconds();
  };

  // Get the required stops excluding START and END.
  std::vector<StopId> available_stops;
  for (StopId stop : state_orig.required_stops) {
    if (stop != state_orig.boundary.start && stop != state_orig.boundary.end) {
      available_stops.push_back(stop);
    }
  }

  RC_PRE(available_stops.size() >= 1);

  // Generate a random path of distinct stops.
  int n = static_cast<int>(available_stops.size());
  int path_length = *rc::gen::inRange(1, n + 1);
  std::vector<StopId> required_path = *rc::gen::unique<std::vector<StopId>>(path_length, rc::gen::elementOf(available_stops));

  RC_LOG() << "required_path: ";
  for (int i = 0; i < required_path.size(); ++i) {
    if (i > 0) RC_LOG() << " -> ";
    RC_LOG() << state_orig.StopName(required_path[i]);
  }
  RC_LOG() << "\n\n";

  // Build the constraints that partition the solution space:
  // - {required_path=[s1], forbid_next=s2}
  // - {required_path=[s1, s2], forbid_next=s3}
  // - ...
  // - {required_path=[s1, ..., s(n-1)], forbid_next=sn}
  // - {required_path=[s1, ..., sn], forbid_next=nullopt}
  std::vector<ElaborateConstraint> constraints;
  for (int i = 1; i <= required_path.size(); ++i) {
    ElaborateConstraint constraint;
    constraint.required_path = std::vector<StopId>(required_path.begin(), required_path.begin() + i);
    if (i < required_path.size()) {
      constraint.forbid_next = required_path[i];
    } else {
      constraint.forbid_next = std::nullopt;
    }
    constraints.push_back(constraint);
  }

  // Compute optimal for original state.
  int opt_orig = FindAndLogOptimal(state_orig, "original");

  // Apply each constraint and compute optimal for each branch.
  int min_branch_opt = std::numeric_limits<int>::max();
  for (int i = 0; i < constraints.size(); ++i) {
    const ElaborateConstraint& constraint = constraints[i];
    ProblemState state_branch = ApplyConstraint(state_orig, constraint);

    std::stringstream label;
    label << "branch " << i << " (path length " << constraint.required_path.size();
    if (constraint.forbid_next.has_value()) {
      label << ", forbid " << state_orig.StopName(constraint.forbid_next.value());
    }
    label << ")";

    int opt_branch = FindAndLogOptimal(state_branch, label.str());
    min_branch_opt = std::min(min_branch_opt, opt_branch);
  }

  RC_ASSERT(opt_orig == min_branch_opt);
}

}  // namespace vats5
