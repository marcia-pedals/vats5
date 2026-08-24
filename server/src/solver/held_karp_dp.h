#pragma once

#include <ostream>
#include <vector>

#include "solver/data.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct HeldKarpDPPathPoint {
  // Stop id in the ProblemState's (uncompacted) id space.
  StopId stop;
  TimeSinceServiceStart arrival;
  TimeSinceServiceStart departure;
};

struct HeldKarpDPResult {
  // Duration of the best tour, or INT_MAX if the problem is infeasible (in
  // which case best_path is empty).
  int best_val;

  // One point per visited group, in visit order. The first point is the
  // boundary START and the last is the boundary END. The first point's
  // arrival equals its departure (the tour starts there) and the last point's
  // departure equals its arrival (the tour ends there). Each departure is as
  // late as possible given the later points, which may be later than the
  // departure the dp itself propagated.
  std::vector<HeldKarpDPPathPoint> best_path;
};

// Exactly solves for the minimum-duration tour that starts at the boundary
// START, ends at the boundary END, and visits at least one stop of every
// required group in between. All steps are priced as they appear in
// `state.minimal`, so a boundary that is a combined stop costs its real rides.
//
// Precondition: START and END are required and in distinct groups.
HeldKarpDPResult HeldKarpDPSolve(
    const ProblemState& state, std::ostream* search_log = nullptr
);

}  // namespace vats5
