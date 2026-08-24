#pragma once

#include <optional>
#include <ostream>

#include "solver/tarel_graph.h"

namespace vats5 {

struct SubsetDpResult {
  int best_duration_seconds;
  TimeSinceServiceStart start_time;
  TimeSinceServiceStart end_time;
  // The sequence of stops visited (boundary start, one member per required
  // group in visit order, boundary end).
  std::vector<StopId> stop_sequence;
};

// Exact schedule-aware solve by dynamic programming over
// (subset of visited required groups, last stop).
//
// Each DP state keeps a Pareto frontier of labels (departure-from-start,
// arrival-at-stop); a label dominates another if it departs at-or-after and
// arrives at-or-before (the same dominance rule as MakeMinimalCover). This
// covers every possible start time in a single pass, so the result is the
// true minimum duration over all departure times.
//
// Returns nullopt if no feasible tour exists. Throws if the problem shape is
// unsupported (more than 24 required groups, or a boundary stop sharing a
// group with another stop).
std::optional<SubsetDpResult> SubsetDpSolve(
    const ProblemState& state, std::ostream* log = nullptr
);

}  // namespace vats5
