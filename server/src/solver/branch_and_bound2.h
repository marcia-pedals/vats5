#pragma once

#include <optional>
#include <string>
#include <vector>
#include "solver/data.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct ElaborateConstraint {
  std::vector<StopId> required_path;
  std::optional<StopId> forbid_next;
};

std::vector<Path> FindSubpaths(const ProblemState& state, const std::vector<TarelEdge>& tour);

Path FindWorstJump(const ProblemState& state, const std::vector<TarelEdge>& tour);

ProblemState ApplyConstraint(
  const ProblemState& state,
  const ElaborateConstraint& constraint
);

int BranchAndBoundSolve2(
  const ProblemState& initial_state,
  std::ostream* search_log,
  std::optional<std::string> run_dir = std::nullopt,
  int max_iter = -1
);

int DummyFunction();

}  // namespace vats5
