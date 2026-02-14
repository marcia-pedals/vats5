#pragma once

#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

// Returns a sorted and minimal (in the sense of CheckSortedAndMinimal) sequence
// of paths that visit `stop_sequence` in order.
std::vector<Path> ComputeMinimalFeasiblePathsAlong(
    const std::vector<StopId>& stop_sequence,
    const StepPathsAdjacencyList& completed);

}  // namespace vats5
