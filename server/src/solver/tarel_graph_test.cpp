#include "solver/tarel_graph.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <unordered_map>

#include "rapidcheck/Assertions.h"
#include "rapidcheck/Classify.h"
#include "solver/data.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {
namespace {

RC_GTEST_PROP(TarelGraphTest, LowerBoundRandomPartition, ()) {
  ProblemState state = *GenProblemState();

  // TODO: Maybe partition based on minimal steps to make it closer to how
  // partitioning works in the real solver.
  std::vector<Step> steps = state.completed.AllMergedSteps();
  int num_partitions = *rc::gen::inRange(1, static_cast<int>(steps.size()));
  // TODO: The printed value is gonna be incomprehensible on failure. Maybe
  // combine with the ProblemState printout somehow.
  std::vector<int> partition_vec = *rc::gen::container<std::vector<int>>(steps.size(), rc::gen::inRange(0, num_partitions - 1));
  std::unordered_map<Step, StepPartitionId> partition;
  for (int i = 0; i < steps.size(); ++i) {
    partition[steps[i]] = StepPartitionId{partition_vec[i]};
  }
  auto edges = MakeTarelEdges(state.completed, [&](const Step& s) -> StepPartitionId {
    return partition.at(s);
  });
  auto merged_edges = MergeEquivalentTarelStates(edges);
  auto graph = MakeTspGraphEdges(merged_edges, state.boundary);
  auto result = SolveTspAndExtractTour(merged_edges, graph, state.boundary);
  RC_ASSERT(result.has_value());

  int lower_bound = result->optimal_value;
  int actual_value = NaiveSolve(state).value;

  RC_CLASSIFY(lower_bound == actual_value);

  RC_ASSERT(lower_bound <= actual_value);
}

// If there are no flex steps and if each step is in a different partition, then
// the lower bound should reach the optimal value.
RC_GTEST_PROP(TarelGraphTest, LowerBoundMaxPartitioning, ()) {
  ProblemState state = *GenProblemState(rc::gen::just(CycleIsFlex::kNo));
  std::unordered_map<Step, StepPartitionId> partition;
  auto edges = MakeTarelEdges(state.completed, [&](const Step& s) -> StepPartitionId {
    const auto [it, _] = partition.try_emplace(s, partition.size());
    return it->second;
  });
  auto merged_edges = MergeEquivalentTarelStates(edges);
  auto graph = MakeTspGraphEdges(merged_edges, state.boundary);
  auto result = SolveTspAndExtractTour(merged_edges, graph, state.boundary);
  RC_ASSERT(result.has_value());
  int lower_bound = result->optimal_value;
  int actual_value = NaiveSolve(state).value;
  RC_ASSERT(lower_bound == actual_value);
}

}  // namespace
}  // namespace vats5
