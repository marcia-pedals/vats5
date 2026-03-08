#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

struct StopIdPairHash {
  std::size_t operator()(const std::pair<StopId, StopId>& v) const {
    std::size_t seed = std::hash<StopId>{}(v.first);
    seed ^=
        std::hash<StopId>{}(v.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

// Cache for on-demand path lookups: (origin, destination) -> paths.
using PathCache = std::
    unordered_map<std::pair<StopId, StopId>, std::vector<Path>, StopIdPairHash>;

// Returns a sorted and minimal (in the sense of CheckSortedAndMinimal) sequence
// of paths that visit `stop_sequence` in order.
// Uses pre-computed paths from `completed`.
std::vector<Path> ComputeMinimalFeasiblePathsAlong(
    const std::vector<StopId>& stop_sequence,
    const StepPathsAdjacencyList& completed
);

// Same as above, but computes paths on-demand from `minimal` using
// FindMinimalPathSet instead of requiring a pre-computed completed graph.
std::vector<Path> ComputeMinimalFeasiblePathsAlong(
    const std::vector<StopId>& stop_sequence, const StepsAdjacencyList& minimal
);

// Same as above, but caches FindMinimalPathSet results in `cache` so that
// repeated calls with overlapping edges avoid redundant shortest-path
// computations.
std::vector<Path> ComputeMinimalFeasiblePathsAlong(
    const std::vector<StopId>& stop_sequence,
    const StepsAdjacencyList& minimal,
    PathCache& cache
);

}  // namespace vats5
