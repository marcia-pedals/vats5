#include "solver/tarel_graph.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <optional>
#include <unordered_map>

#include "rapidcheck/gen/Build.h"
#include "solver/concorde.h"

#include "rapidcheck/Assertions.h"
#include "rapidcheck/Classify.h"
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {
namespace {

// A problem with two stops a, b, and no steps between them is infeasible and
// the relaxation should have no solution.
TEST(TarelGraphTest, InfeasibleProblemNoSolution) {
  std::vector<Step> steps;
  std::unordered_set<StopId> stops;
  std::unordered_map<StopId, std::string> stop_names;

  stops.insert(StopId{0});
  stops.insert(StopId{1});
  stop_names[StopId{0}] = "a";
  stop_names[StopId{1}] = "b";

  ProblemBoundary boundary{
    .start=StopId{2},
    .end=StopId{3},
  };
  AddBoundary(steps, stops, stop_names, boundary);

  ProblemState state = MakeProblemState(MakeAdjacencyList(steps), boundary, stops, stop_names);

  std::optional<TspTourResult> result = ComputeTarelLowerBound(state);

  EXPECT_FALSE(result.has_value());
}

RC_GTEST_PROP(TarelGraphTest, LowerBoundRandomPartition, ()) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state = *GenProblemState(std::nullopt, rc::gen::construct<StepPartitionId>(rc::gen::inRange(0, num_partitions - 1)));

  std::optional<TspTourResult> result;
  try {
    result = ComputeTarelLowerBound(state);
  } catch (const InvalidTourStructure&) {
    RC_DISCARD("InvalidTourStructure");
  }
  RC_ASSERT(result.has_value());

  int lower_bound = result->optimal_value;
  int actual_value = BruteForceSolveOptimalDuration(state);
  RC_CLASSIFY(lower_bound == actual_value);
  RC_ASSERT(lower_bound <= actual_value);
}

// TODO: Figure out GenProblemState API allowing us to express this property.

// // If there are no flex steps and if each step is in a different partition, then
// // the lower bound should reach the optimal value.
// RC_GTEST_PROP(TarelGraphTest, LowerBoundMaxPartitioning, ()) {
//   ProblemState state = *GenProblemState(rc::gen::just(CycleIsFlex::kNo), [](std::vector<Step>& steps) {
//     for (int i = 0; i < steps.size(); ++i) {
//       steps[i].origin.partition = StepPartitionId{i};
//       steps[i].destination.partition = StepPartitionId{i};
//     }
//   });
//   std::optional<TspTourResult> result;
//   try {
//     result = ComputeTarelLowerBound(state);
//   } catch (const InvalidTourStructure&) {
//     RC_DISCARD("InvalidTourStructure");
//   }
//   RC_ASSERT(result.has_value());
//   int lower_bound = result->optimal_value;
//   int actual_value = BruteForceSolveOptimalDuration(state);
//   RC_ASSERT(lower_bound == actual_value);
// }

}  // namespace
}  // namespace vats5
