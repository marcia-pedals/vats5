#include "solver/graph_util.h"
#include <unordered_set>

namespace {

using namespace vats5;

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
  const std::unordered_set<StopId>& stops
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

      std::unordered_set<StopId> ab_inner_stops;
      ab_inner_stops.reserve(paths_ab[0].IntermediateStopCount());
      paths_ab[0].VisitIntermediateStops([&](auto x) { ab_inner_stops.insert(x); });
      for (const Path& path : paths_ab) {
        IntersectWithIntermediateStops(ab_inner_stops, path);
      }
      for (const Path& path : paths_ba) {
        IntersectWithIntermediateStops(ab_inner_stops, path);
      }

      for (const StopId stop : ab_inner_stops) {
        inner_stops.insert(stop);
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
