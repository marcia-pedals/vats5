#include "solver/graph_util.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

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
  const StepsAdjacencyList& g_steps,
  const StepPathsAdjacencyList& g,
  const std::unordered_set<StopId>& stops,
  StopId block_paths_through,
  const ProblemState* state
) {
  std::vector<StopId> stops_vec(stops.begin(), stops.end());

  // A shortest path from a->b or b->a that avoids x, with a score.
  struct AvoidPath {
    StopId a;
    StopId b;
    StopId x;

    // The shortest path a->b or b->a that avoids x, with a score.
    std::optional<Path> path;

    // The duration of `path` minus the duration of the shortest out of all paths from a->b or b->a.
    // If `path` does not exist, std::numeric_limits<int>::max(), because that is very good.
    int score = std::numeric_limits<int>::min();

    auto operator<=>(const AvoidPath& other) const {
      return score <=> other.score;
    }
  };

  // best_avoid[x] is the AvoidPath avoiding x with the best score.
  std::unordered_map<StopId, AvoidPath> best_avoid;

  for (int i = 0; i < stops_vec.size(); ++i) {
    for (int j = i + 1; j < stops_vec.size(); ++j) {
      StopId a = stops_vec[i];
      StopId b = stops_vec[j];

      std::vector<Path> paths;
      auto paths_ab = g.PathsBetween(a, b);
      paths.insert(paths.end(), paths_ab.begin(), paths_ab.end());
      auto paths_ba = g.PathsBetween(b, a);
      paths.insert(paths.end(), paths_ba.begin(), paths_ba.end());

      int shortest_dur_thru = std::numeric_limits<int>::max();
      std::unordered_map<StopId, std::optional<Path>> shortest_avoiding;
      for (const Path& path : paths) {
        shortest_dur_thru = std::min(shortest_dur_thru, path.DurationSeconds());

        std::unordered_set<StopId> intermediate_stops;
        path.VisitIntermediateStops([&](StopId x) { intermediate_stops.insert(x); });
        for (StopId x : stops_vec) {
          if (!intermediate_stops.contains(x)) {
            std::optional<Path>& cur_shortest = shortest_avoiding[x];
            if (!cur_shortest.has_value() || path.DurationSeconds() < cur_shortest->DurationSeconds()) {
              cur_shortest = path;
            }
          }
        }
      }

      if (shortest_dur_thru == std::numeric_limits<int>::max()) {
        // There is no path a->b or b->a.
        continue;
      }

      // Ok, we have found shortest_avoiding[x] considering only shortest paths
      // a->b and b->a.
      //
      // This means that shortest_avoiding[x] is correct for any x that we have
      // actually computed it for: if there were a shorter path avoiding x a->b
      // or b->a, it would have been included in the paths a->b or b->a that we
      // considered.
      //
      // But for keys x that we have not inserted, there may be a shortest path
      // a->b or b->a avoiding x that we haven't considered because it's not a
      // shortest path a->b or b->a. So consider those now.
      for (StopId x : stops_vec) {
        if (x == a || x == b) {
          continue;
        }
        std::optional<Path>& cur_shortest = shortest_avoiding[x];
        if (cur_shortest.has_value()) {
          // Already inserted. See above comment about why it's already correct.
          continue;
        }

        // TODO: Justify origin_lb/origin_ub and probably make them configurabler.
        TimeSinceServiceStart origin_lb{0};
        TimeSinceServiceStart origin_ub{36 * 3600};
        std::vector<Path> ab_avoiding = FindMinimalPathSet(g_steps, a, {b}, origin_lb, origin_ub, nullptr, false, {x, block_paths_through})[b];
        std::vector<Path> ba_avoiding = FindMinimalPathSet(g_steps, b, {a}, origin_lb, origin_ub, nullptr, false, {x, block_paths_through})[a];
        std::vector<Path> avoiding;
        avoiding.insert(avoiding.end(), ab_avoiding.begin(), ab_avoiding.end());
        avoiding.insert(avoiding.end(), ba_avoiding.begin(), ba_avoiding.end());
        for (const Path& path : avoiding) {
          if (!cur_shortest.has_value() || path.DurationSeconds() < cur_shortest->DurationSeconds()) {
            cur_shortest = path;
          }
        }
      }

      for (const auto& [x, shortest_avoiding_x] : shortest_avoiding) {
        AvoidPath avoid_path{
          .a=a,
          .b=b,
          .x=x,
          .path=shortest_avoiding_x,
          .score=shortest_avoiding_x.has_value() ? shortest_avoiding_x->DurationSeconds() - shortest_dur_thru : std::numeric_limits<int>::max(),
        };
        best_avoid[x] = std::max(best_avoid[x], avoid_path);
      }
    }
  }

  std::unordered_set<StopId> inner_stops;
  for (const auto& [x, best_avoid_x] : best_avoid) {
    if (!best_avoid_x.path.has_value()) {
      inner_stops.insert(x);
    }
  }

  std::unordered_set<StopId> extreme_stops = stops;
  for (StopId inner_stop : inner_stops) {
    extreme_stops.erase(inner_stop);
  }

  if (state) {
    std::vector<std::pair<StopId, AvoidPath>> best_avoid_sorted(best_avoid.begin(), best_avoid.end());
    std::sort(best_avoid_sorted.begin(), best_avoid_sorted.end(), [](const auto& a, const auto& b) {
      return a.second.score > b.second.score;
    });
    std::cout << "best_avoid:\n";
    for (const auto& [x, best_avoid_x] : best_avoid_sorted) {
      if (!best_avoid_x.path.has_value()) continue;
      std::cout << "  x=" << state->StopName(x)
                << " a=" << state->StopName(best_avoid_x.a)
                << " b=" << state->StopName(best_avoid_x.b)
                << " score=" << TimeSinceServiceStart{best_avoid_x.score}.ToString()
                << "\n    path=";
      bool first = true;
      best_avoid_x.path->VisitIntermediateStops([&](StopId s) {
        if (!first) std::cout << " -> ";
        std::cout << state->StopName(s);
        first = false;
      });
      std::cout << "\n";
    }
  }

  return extreme_stops;
}

}  // namespace vats5
