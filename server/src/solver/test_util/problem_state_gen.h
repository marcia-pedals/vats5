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

// Generates a ProblemState whose only steps are flex and which has a flex step
// between every pair in both directions.
rc::Gen<ProblemState> GenFlexProblemState();

struct NamedBranchEdge {
  BranchEdge edge;
  std::string a_name;
  std::string b_name;
};

void showValue(const NamedBranchEdge& e, std::ostream& os);

rc::Gen<NamedBranchEdge> GenBranchEdge(const ProblemState& state);

}  // namespace vats5
