#include "solver/tour_paths.h"

#include <algorithm>
#include <cassert>

#include "solver/step_merge.h"

namespace vats5 {

// Returns a sorted and minimal (in the sense of CheckSortedAndMinimal) sequence
// of paths that visit `stop_sequence` in order.
std::vector<Path> ComputeMinimalFeasiblePathsAlong(
  const std::vector<StopId>& stop_sequence,
  const StepPathsAdjacencyList& completed
) {
  if (stop_sequence.size() < 2) {
    return {};
  }

  // Initialize with the paths for the first edge.
  std::span<const Path> first_paths =
      completed.PathsBetween(stop_sequence[0], stop_sequence[1]);
  std::vector<Path> paths(first_paths.begin(), first_paths.end());

  for (size_t i = 1; i + 1 < stop_sequence.size(); ++i) {
    std::span<const Path> next_paths =
        completed.PathsBetween(stop_sequence[i], stop_sequence[i + 1]);

    // Extract merged steps from current and next paths.
    std::vector<Step> ab_steps;
    ab_steps.reserve(paths.size());
    for (const Path& p : paths) {
      ab_steps.push_back(p.merged_step);
    }

    std::vector<Step> bc_steps;
    bc_steps.reserve(next_paths.size());
    for (const Path& p : next_paths) {
      bc_steps.push_back(p.merged_step);
    }

    // Merge steps with provenance tracking.
    std::vector<StepProvenance> provenance;
    std::vector<Step> merged =
        PairwiseMergedSteps(ab_steps, bc_steps, &provenance);

    // Build new paths by combining the constituent paths from each side.
    std::vector<Path> new_paths;
    new_paths.reserve(merged.size());
    for (size_t j = 0; j < merged.size(); ++j) {
      Path path{.merged_step = merged[j],
                .steps = paths[provenance[j].ab_index].steps};
      const std::vector<Step>& next_steps =
          next_paths[provenance[j].bc_index].steps;
      path.steps.insert(path.steps.end(), next_steps.begin(), next_steps.end());
      new_paths.push_back(std::move(path));
    }

    paths = std::move(new_paths);
  }

  // TODO: Reference thing about 00:00:00.
  std::erase_if(paths, [](const Path& path) {
    return path.merged_step.origin.time < TimeSinceServiceStart{0};
  });

  for (Path& path : paths) {
    NormalizeConsecutiveSteps(path.steps);
    assert(ConsecutiveMergedSteps(path.steps) == path.merged_step);
  }

  return paths;
}

}  // namespace vats5
