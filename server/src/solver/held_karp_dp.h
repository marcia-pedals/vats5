#pragma once

#include <ostream>
#include <vector>

#include "solver/data.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct HeldKarpDPResult {
  // Duration of the best tour, or INT_MAX if the problem is infeasible (in
  // which case best_tour is empty).
  int best_val;

  // The visited stops in visit order, from the boundary START to the boundary
  // END.
  std::vector<StopId> best_tour;
};

HeldKarpDPResult HeldKarpDPSolve(
    const ProblemState& state, int known_lb, std::ostream* search_log = nullptr
);

}  // namespace vats5
