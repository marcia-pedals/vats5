#pragma once

#include <optional>
#include <ostream>
#include <vector>

#include "solver/data.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct TwoOptOptions {
  // Number of hill-climbing starts.
  int restarts = 250;

  int seed = 1;

  // Wall-clock budget. Unset means no limit.
  std::optional<double> time_limit_seconds;
};

struct TwoOptResult {
  // Duration of the best tour, or INT_MAX if no feasible tour was found (in
  // which case best_tour is empty).
  int best_val;

  // The visited stops in visit order, from the boundary START to the boundary
  // END, in the ProblemState's stop ids.
  std::vector<StopId> best_tour;

  // Number of tour orders scored.
  long long evaluations = 0;

  int restarts_completed = 0;

  // Wall-clock seconds each completed restart took, in restart order.
  std::vector<double> restart_seconds;
};

// Heuristically solves the tour problem by multi-start 2-opt local search
// over visit orders of the required groups (plus group-member swaps for
// groups with more than one stop).
TwoOptResult TwoOptSolve(
    const ProblemState& state,
    const TwoOptOptions& options = {},
    std::ostream* search_log = nullptr
);

}  // namespace vats5
