#include "solver/graph_util.h"
#include <queue>
#include <unordered_set>

namespace {

using namespace vats5;

// Returns true if there exists a path from `start` to `end` in `g` that does
// not visit `avoid`.
//
// Paths that go through `block_paths_through` are not considered. (Paths
// starting or ending at it are considered though.)
bool HasPathAvoiding(
  const StepPathsAdjacencyList& g,
  StopId start,
  StopId end,
  StopId avoid,
  StopId block_paths_through
) {
  if (start == avoid || end == avoid) {
    return false;
  }

  std::unordered_set<StopId> visited;
  std::queue<StopId> queue;
  queue.push(start);
  visited.insert(start);

  while (!queue.empty()) {
    StopId current = queue.front();
    queue.pop();

    if (start != block_paths_through && current == block_paths_through) {
      continue;
    }

    if (current == end) {
      return true;
    }

    auto path_groups_it = g.adjacent.find(current);
    if (path_groups_it == g.adjacent.end()) {
      continue;
    }

    for (const auto& path_group : path_groups_it->second) {
      for (const Path& path : path_group) {
        StopId dest = path.merged_step.destination.stop;

        // Skip if we've already visited dest.
        if (visited.contains(dest)) {
          continue;
        }

        // Skip if this path goes to or through the avoid stop.
        if (dest == avoid) {
          continue;
        }
        bool goes_through_avoid = false;
        path.VisitIntermediateStops([&](StopId intermediate) {
          if (intermediate == avoid) {
            goes_through_avoid = true;
          }
        });
        if (goes_through_avoid) {
          continue;
        }

        // This path is usable; add destination to queue.
        visited.insert(dest);
        queue.push(dest);
      }
    }
  }

  return false;
}

// Intersects `stops` in-place with the stops in `path`.
void IntersectWithIntermediateStops(std::unordered_set<StopId>& stops, const Path& path) {
  std::unordered_set<StopId> intersection;
  intersection.reserve(std::min(static_cast<int>(stops.size()), path.IntermediateStopCount()));
  path.VisitIntermediateStops([&](auto x) {
    if (stops.contains(x)) {
      intersection.insert(x);
    }
  });
  stops = std::move(intersection);
}

}  // namespace

namespace vats5 {

std::unordered_set<StopId> ComputeExtremeStops(
  const StepPathsAdjacencyList& g,
  const std::unordered_set<StopId>& stops,
  StopId block_paths_through
) {
  std::vector<StopId> stops_vec(stops.begin(), stops.end());

  std::unordered_set<StopId> inner_stops;
  for (int i = 0; i < stops_vec.size(); ++i) {
    for (int j = i + 1; j < stops_vec.size(); ++j) {
      StopId a = stops_vec[i];
      StopId b = stops_vec[j];

      auto paths_ab = g.PathsBetween(a, b);
      auto paths_ba = g.PathsBetween(b, a);

      if (paths_ab.size() == 0 || paths_ba.size() == 0) {
        continue;
      }

      // Get candidate inner stops from minimal paths (intersection of all
      // intermediate stops across all minimal paths in both directions).
      std::unordered_set<StopId> candidate_inner_stops;
      candidate_inner_stops.reserve(paths_ab[0].IntermediateStopCount());
      paths_ab[0].VisitIntermediateStops([&](auto x) { candidate_inner_stops.insert(x); });
      for (const Path& path : paths_ab) {
        IntersectWithIntermediateStops(candidate_inner_stops, path);
      }
      for (const Path& path : paths_ba) {
        IntersectWithIntermediateStops(candidate_inner_stops, path);
      }

      // Verify each candidate is truly on ALL paths (not just minimal paths).
      // A candidate X is only an inner stop if there's no way to get from a to b
      // (or b to a) without going through X.
      for (StopId candidate : candidate_inner_stops) {
        if (inner_stops.contains(candidate)) continue;
        if (!HasPathAvoiding(g, a, b, candidate, block_paths_through) &&
            !HasPathAvoiding(g, b, a, candidate, block_paths_through)) {
          inner_stops.insert(candidate);
        }
      }
    }
  }

  std::unordered_set<StopId> extreme_stops = stops;
  for (StopId inner_stop : inner_stops) {
    extreme_stops.erase(inner_stop);
  }
  return extreme_stops;
}

}  // namespace vats5
