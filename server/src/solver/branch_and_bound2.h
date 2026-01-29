#pragma once


#include <vector>
#include "solver/data.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct ElaborateConstraint {
  std::vector<StopId> required_path;
  std::optional<StopId> forbid_next;
};

ProblemState ApplyConstraint(
  const ProblemState& state,
  const ElaborateConstraint& constraint
);

int DummyFunction();

}  // namespace vats5
