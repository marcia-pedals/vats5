#pragma once

#include <rapidcheck.h>

#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "solver/branch_and_bound.h"
#include "solver/tarel_graph.h"

namespace vats5 {

enum class CycleIsFlex { kNo, kYes };

// Generates a random RequiredStops for `num_stops` stops (ids 0..n-1).
// Stops are randomly assigned to groups.
// Does NOT include boundary stops — callers must add those.
rc::Gen<RequiredStops> GenRequiredStops(int num_stops);

rc::Gen<ProblemState> GenProblemState(
    std::optional<rc::Gen<CycleIsFlex>> cycle_is_flex_gen = std::nullopt,
    std::optional<rc::Gen<StepPartitionId>> step_partition_gen = std::nullopt
);

// Generates a ProblemState where all steps are flex (complete bidirectional
// graph with random durations). Also generates alternate stops. With all-flex
// and distinct partitions per step, the tarel lower bound should be tight.
rc::Gen<ProblemState> GenFlexProblemState();

struct NamedBranchEdge {
  BranchEdge edge;
  std::string a_name;
  std::string b_name;
};

void showValue(const NamedBranchEdge& e, std::ostream& os);

rc::Gen<NamedBranchEdge> GenBranchEdge(const ProblemState& state);

}  // namespace vats5
