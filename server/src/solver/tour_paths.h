#pragma once

#include <span>
#include <unordered_map>
#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

// Returns the sorted and minimal (in the sense of CheckSortedAndMinimal) set of
// paths made of a path from `ab_paths` followed by a path from `bc_paths`.
//
// Unlike ComputeMinimalFeasiblePathsAlong, the results are not normalized and
// paths departing before 00:00:00 are not removed; do both once the sequence is
// complete.
std::vector<Path> ExtendMinimalFeasiblePaths(
    std::span<const Path> ab_paths, std::span<const Path> bc_paths
);

// The minimal path sets between pairs of stops, computed on demand and kept.
class MinimalPathSetCache {
 public:
  explicit MinimalPathSetCache(const StepsAdjacencyList& minimal)
      : minimal_(minimal) {}

  // The minimal set of paths from `origin` to `destination`, empty if there are
  // none. Remains valid for the life of the cache: later calls can rehash the
  // map, which invalidates its iterators but not its elements.
  std::span<const Path> Between(StopId origin, StopId destination);

 private:
  const StepsAdjacencyList& minimal_;
  std::unordered_map<PlainEdge, std::vector<Path>> paths_;
};

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

// Same as above, but reusing whatever `cache` has already computed.
std::vector<Path> ComputeMinimalFeasiblePathsAlong(
    const std::vector<StopId>& stop_sequence, MinimalPathSetCache& cache
);

}  // namespace vats5
