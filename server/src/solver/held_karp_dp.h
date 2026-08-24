#pragma once

#include <ostream>
#include <vector>

#include "solver/data.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct HeldKarpDPPathPoint {
  StopId stop;
  TimeSinceServiceStart arrival;
  TimeSinceServiceStart departure;
};

struct HeldKarpDPResult {
  // Duration of the best tour, or INT_MAX if the problem is infeasible (in
  // which case best_path is empty).
  int best_val;

  std::vector<HeldKarpDPPathPoint> best_path;
};

HeldKarpDPResult HeldKarpDPSolve(
    const ProblemState& state, std::ostream* search_log = nullptr
);

}  // namespace vats5
