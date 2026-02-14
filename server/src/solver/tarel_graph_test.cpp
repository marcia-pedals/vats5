#include "solver/tarel_graph.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <nlohmann/json.hpp>
#include <optional>
#include <unordered_map>

#include "rapidcheck/Assertions.h"
#include "rapidcheck/Classify.h"
#include "solver/concorde.h"
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/test_util/cached_test_data.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {
namespace {

// Validates that merged edges have contiguous partition IDs starting from 0 for
// each stop, and that all expected stops are present. Returns error messages
// (empty if validation passes).
std::vector<std::string> ValidateMergedEdgePartitions(
    const std::vector<TarelEdge>& merged,
    const std::unordered_set<StopId>& expected_stops
) {
  std::vector<std::string> errors;

  std::unordered_map<StopId, std::unordered_set<int>> partitions_by_stop;
  for (const TarelEdge& e : merged) {
    partitions_by_stop[e.origin.stop].insert(e.origin.partition.v);
    partitions_by_stop[e.destination.stop].insert(e.destination.partition.v);
  }

  // Each stop's partitions should be contiguous starting from 0.
  for (const auto& [stop, partitions] : partitions_by_stop) {
    if (partitions.empty()) continue;
    int max_partition = *std::max_element(partitions.begin(), partitions.end());
    if (partitions.size() != max_partition + 1) {
      errors.push_back(
          "Stop " + std::to_string(stop.v) + " has non-contiguous partitions"
      );
    }
    for (int i = 0; i <= max_partition; ++i) {
      if (!partitions.contains(i)) {
        errors.push_back(
            "Stop " + std::to_string(stop.v) + " missing partition " +
            std::to_string(i)
        );
      }
    }
  }

  // All expected stops should be present.
  for (StopId stop : expected_stops) {
    if (!partitions_by_stop.contains(stop)) {
      errors.push_back(
          "Expected stop " + std::to_string(stop.v) +
          " not found in merged edges"
      );
    }
  }

  return errors;
}

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
      .start = StopId{2},
      .end = StopId{3},
  };
  AddBoundary(steps, stops, stop_names, boundary);

  ProblemState state = MakeProblemState(
      MakeAdjacencyList(steps), boundary, stops, stop_names, {}, {}
  );

  std::optional<TspTourResult> result = ComputeTarelLowerBound(state);

  EXPECT_FALSE(result.has_value());
}

TEST(TarelGraphTest, TarelEdges_BART) {
  const auto test_data = GetCachedFilteredTestData(
      {"../data/raw_RG_202506", "20250718", {"BA:"}}
  );
  std::unordered_set<StopId> bart_stops = GetStopsForTripIdPrefix(
      test_data.gtfs_day, test_data.steps_from_gtfs.mapping, "BA:"
  );
  ProblemState state =
      InitializeProblemState(test_data.steps_from_gtfs, bart_stops);
  std::vector<TarelEdge> edges = MakeTarelEdges(state.completed);

  StopId warm_springs = state.StopIdFromName("Warm Springs South Fremont BART");
  StopId berryessa = state.StopIdFromName("Berryessa / North San Jose");

  struct ExpectedEdge {
    std::string arrival;
    std::string departure;
    int weight_seconds;
    auto operator<=>(const ExpectedEdge&) const = default;
  };

  std::vector<ExpectedEdge> actual_edges;
  for (const TarelEdge& e : edges) {
    if (e.origin.stop == warm_springs && e.destination.stop == berryessa) {
      actual_edges.push_back(
          ExpectedEdge{
              state.PartitionName(e.origin.partition),
              state.PartitionName(e.destination.partition),
              e.weight,
          }
      );
    }
  }
  std::ranges::sort(actual_edges);

  std::vector<ExpectedEdge> expected_edges{
      // Stay on line.
      {"Green-N North", "Green-N North", 12 * 60},

      // Get off and transfer line in same direction.
      {"Green-N North", "Orange-S South", 19 * 60},

      // Switch direction from Green.
      {"Green-S South", "Green-N North", 21 * 60},
      {"Green-S South", "Orange-S South", 28 * 60},

      // Switch direction from Orange.
      {"Orange-N North", "Green-N North", 29 * 60},
      {"Orange-N North", "Orange-S South", 16 * 60},

      // Get off and transfer line in same direction.
      {"Orange-S South", "Green-N North", 25 * 60},

      // Stay on line.
      {"Orange-S South", "Orange-S South", 12 * 60},

      // Arrive flex and ride to dest immediately.
      {"unnamed(-1)", "Green-N North", 12 * 60},
      {"unnamed(-1)", "Orange-S South", 12 * 60},
  };

  EXPECT_EQ(actual_edges, expected_edges);
}

RC_GTEST_PROP(TarelGraphTest, LowerBoundRandomPartition, ()) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state = *GenProblemState(
      std::nullopt,
      rc::gen::construct<StepPartitionId>(
          rc::gen::inRange(0, num_partitions - 1)
      )
  );

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

// If there are no flex steps and if each step is in a different partition, then
// the lower bound should reach the optimal value.
RC_GTEST_PROP(TarelGraphTest, LowerBoundMaxPartitioning, ()) {
  ProblemState state = *GenProblemState(
      rc::gen::just(CycleIsFlex::kNo), rc::gen::just(StepPartitionId::NONE)
  );

  // Give every step a distinct partition and regenerate the problem state.
  std::vector<Step> steps = state.minimal.AllSteps();
  for (int i = 0; i < steps.size(); ++i) {
    steps[i].origin.partition.v = i;
    steps[i].destination.partition.v = i;
  }
  state = MakeProblemState(
      MakeAdjacencyList(steps),
      state.boundary,
      state.required_stops,
      state.stop_names,
      state.step_partition_names,
      state.original_edges
  );

  std::optional<TspTourResult> result;
  try {
    result = ComputeTarelLowerBound(state);
  } catch (const InvalidTourStructure&) {
    RC_DISCARD("InvalidTourStructure");
  }
  RC_ASSERT(result.has_value());
  int lower_bound = result->optimal_value;
  int actual_value = BruteForceSolveOptimalDuration(state);
  RC_ASSERT(lower_bound == actual_value);
}

// Serialization round-trip: serialize to JSON and deserialize back should
// produce an equivalent ProblemState.
RC_GTEST_PROP(TarelGraphTest, SerializationRoundTrip, ()) {
  ProblemState original = *GenProblemState();

  // Serialize to JSON
  nlohmann::json j = original;

  // Deserialize back
  ProblemState deserialized = j.get<ProblemState>();

  // Check that the deserialized state matches the original
  RC_ASSERT(original.boundary.start == deserialized.boundary.start);
  RC_ASSERT(original.boundary.end == deserialized.boundary.end);
  RC_ASSERT(original.required_stops == deserialized.required_stops);
  RC_ASSERT(original.stop_names == deserialized.stop_names);
  RC_ASSERT(original.step_partition_names == deserialized.step_partition_names);
  RC_ASSERT(original.original_edges == deserialized.original_edges);

  // Check minimal steps match
  std::vector<Step> original_steps = original.minimal.AllSteps();
  std::vector<Step> deserialized_steps = deserialized.minimal.AllSteps();
  RC_ASSERT(original_steps.size() == deserialized_steps.size());

  // Sort steps for comparison since adjacency list ordering may differ
  auto step_sort_key = [](const Step& s) {
    return std::tuple(
        s.origin.stop.v,
        s.destination.stop.v,
        s.origin.time.seconds,
        s.destination.time.seconds,
        s.is_flex
    );
  };
  std::sort(
      original_steps.begin(),
      original_steps.end(),
      [&](const Step& a, const Step& b) {
        return step_sort_key(a) < step_sort_key(b);
      }
  );
  std::sort(
      deserialized_steps.begin(),
      deserialized_steps.end(),
      [&](const Step& a, const Step& b) {
        return step_sort_key(a) < step_sort_key(b);
      }
  );

  for (size_t i = 0; i < original_steps.size(); ++i) {
    RC_ASSERT(
        original_steps[i].origin.stop == deserialized_steps[i].origin.stop
    );
    RC_ASSERT(
        original_steps[i].destination.stop ==
        deserialized_steps[i].destination.stop
    );
    RC_ASSERT(
        original_steps[i].origin.time == deserialized_steps[i].origin.time
    );
    RC_ASSERT(
        original_steps[i].destination.time ==
        deserialized_steps[i].destination.time
    );
    RC_ASSERT(original_steps[i].is_flex == deserialized_steps[i].is_flex);
  }

  // Verify completed is recomputed correctly by comparing merged steps
  std::vector<Step> original_completed = original.completed.AllMergedSteps();
  std::vector<Step> deserialized_completed =
      deserialized.completed.AllMergedSteps();
  RC_ASSERT(original_completed.size() == deserialized_completed.size());

  std::sort(
      original_completed.begin(),
      original_completed.end(),
      [&](const Step& a, const Step& b) {
        return step_sort_key(a) < step_sort_key(b);
      }
  );
  std::sort(
      deserialized_completed.begin(),
      deserialized_completed.end(),
      [&](const Step& a, const Step& b) {
        return step_sort_key(a) < step_sort_key(b);
      }
  );

  for (size_t i = 0; i < original_completed.size(); ++i) {
    RC_ASSERT(
        original_completed[i].origin.stop ==
        deserialized_completed[i].origin.stop
    );
    RC_ASSERT(
        original_completed[i].destination.stop ==
        deserialized_completed[i].destination.stop
    );
    RC_ASSERT(
        original_completed[i].origin.time ==
        deserialized_completed[i].origin.time
    );
    RC_ASSERT(
        original_completed[i].destination.time ==
        deserialized_completed[i].destination.time
    );
    RC_ASSERT(
        original_completed[i].is_flex == deserialized_completed[i].is_flex
    );
  }
}

// Test that MergeEquivalentTarelStates handles destination-only states
// correctly. A destination-only state is one that appears only as an edge
// destination, never as an origin.
TEST(TarelGraphTest, MergeEquivalentTarelStates_DestinationOnlyStates) {
  // Create edges where state B only appears as a destination (never as an
  // origin). A -> B should still produce valid output after merging.
  TarelState stateA0{StopId{0}, StepPartitionId{0}};
  TarelState stateA1{StopId{0}, StepPartitionId{1}};
  TarelState stateB0{StopId{1}, StepPartitionId{0}};  // destination-only
  TarelState stateB1{StopId{1}, StepPartitionId{1}};  // destination-only

  std::vector<TarelEdge> edges = {
      {.origin = stateA0,
       .destination = stateB0,
       .weight = 100,
       .original_origins = {stateA0},
       .original_destinations = {stateB0}},
      {.origin = stateA1,
       .destination = stateB1,
       .weight = 200,
       .original_origins = {stateA1},
       .original_destinations = {stateB1}},
  };

  std::vector<TarelEdge> merged = MergeEquivalentTarelStates(edges);

  // Verify that all states have valid partition IDs and expected stops are
  // present.
  std::vector<std::string> errors =
      ValidateMergedEdgePartitions(merged, {StopId{0}, StopId{1}});
  EXPECT_TRUE(errors.empty())
      << "Validation errors: " << (errors.empty() ? "" : errors[0]);
}

// Property test: after merging, all states should have contiguous partition IDs
// starting from 0.
RC_GTEST_PROP(
    TarelGraphTest, MergeEquivalentTarelStates_AllDestinationsValid, ()
) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state = *GenProblemState(
      std::nullopt,
      rc::gen::construct<StepPartitionId>(
          rc::gen::inRange(0, num_partitions - 1)
      )
  );

  std::vector<TarelEdge> edges = MakeTarelEdges(state.completed);
  std::vector<TarelEdge> merged = MergeEquivalentTarelStates(edges);

  // Verify that all states have valid partition IDs and expected stops are
  // present.
  std::vector<std::string> errors =
      ValidateMergedEdgePartitions(merged, state.required_stops);
  RC_ASSERT(errors.empty());
}

}  // namespace
}  // namespace vats5
