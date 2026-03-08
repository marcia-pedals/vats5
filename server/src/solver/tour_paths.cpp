#include "solver/tour_paths.h"

#include <algorithm>
#include <cassert>

#include "solver/step_merge.h"
#include "solver/steps_shortest_path.h"

namespace vats5 {

namespace {

// Common implementation for ComputeMinimalFeasiblePathsAlong.
// GetEdgePaths(origin, destination) must return the paths for that edge,
// either as a vector or span of Path. Returns empty optional if no paths exist.
template <typename GetEdgePaths>
std::vector<Path> ComputeMinimalFeasiblePathsAlongImpl(
    const std::vector<StopId>& stop_sequence, GetEdgePaths&& get_edge_paths
) {
  if (stop_sequence.size() < 2) {
    return {};
  }

  auto first_edge = get_edge_paths(stop_sequence[0], stop_sequence[1]);
  if (!first_edge.has_value() || first_edge->empty()) {
    return {};
  }
  std::vector<Path> paths(first_edge->begin(), first_edge->end());

  for (size_t i = 1; i + 1 < stop_sequence.size(); ++i) {
    auto next_edge = get_edge_paths(stop_sequence[i], stop_sequence[i + 1]);
    if (!next_edge.has_value() || next_edge->empty()) {
      return {};
    }
    const auto& next_paths = *next_edge;

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
      Path path{
          .merged_step = merged[j], .steps = paths[provenance[j].ab_index].steps
      };
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

}  // namespace

std::vector<Path> ComputeMinimalFeasiblePathsAlong(
    const std::vector<StopId>& stop_sequence,
    const StepPathsAdjacencyList& completed
) {
  return ComputeMinimalFeasiblePathsAlongImpl(
      stop_sequence,
      [&](StopId a, StopId b) -> std::optional<std::span<const Path>> {
        std::span<const Path> paths = completed.PathsBetween(a, b);
        if (paths.empty()) {
          return std::nullopt;
        }
        return paths;
      }
  );
}

std::vector<Path> ComputeMinimalFeasiblePathsAlong(
    const std::vector<StopId>& stop_sequence, const StepsAdjacencyList& minimal
) {
  return ComputeMinimalFeasiblePathsAlongImpl(
      stop_sequence,
      [&](StopId a, StopId b) -> std::optional<std::vector<Path>> {
        auto path_map = FindMinimalPathSet(minimal, a, {b});
        auto it = path_map.find(b);
        if (it == path_map.end()) {
          return std::nullopt;
        }
        return std::move(it->second);
      }
  );
}

}  // namespace vats5
