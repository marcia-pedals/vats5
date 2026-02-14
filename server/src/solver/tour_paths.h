#pragma once

#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

// Compute the minimum-duration feasible paths along a stop sequence.
//
// `stop_sequence` is the ordered list of stops to visit (e.g. [START, A, B, C,
// END]). Consecutive pairs form the edges. The first element
// (stop_sequence[0]) is used as the start point.
//
// `completed` provides the merged steps between consecutive stops.
std::vector<Path> ComputeMinDurationFeasiblePaths(
    const std::vector<StopId>& stop_sequence,
    const StepPathsAdjacencyList& completed);

}  // namespace vats5
