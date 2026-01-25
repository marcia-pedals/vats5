#pragma once

#include <rapidcheck.h>

#include <iostream>
#include <unordered_set>

#include "solver/branch_and_bound.h"
#include "solver/tarel_graph.h"

namespace vats5 {

enum class CycleIsFlex { kNo, kYes };

rc::Gen<ProblemState> GenProblemState(
  std::optional<rc::Gen<CycleIsFlex>> cycle_is_flex_gen = std::nullopt,
  std::optional<rc::Gen<StepPartitionId>> step_partition_gen = std::nullopt
);

struct NamedBranchEdge {
  BranchEdge edge;
  std::string a_name;
  std::string b_name;
};

void showValue(const NamedBranchEdge& e, std::ostream& os);

rc::Gen<NamedBranchEdge> GenBranchEdge(const ProblemState& state);

}  // namespace vats5
