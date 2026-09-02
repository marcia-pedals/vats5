#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "solver/steps_adjacency_list.h"

namespace vats5 {

inline constexpr int kPairStepsUnreachable = std::numeric_limits<int>::max();

// Scheduled (non-flex) steps of one directed stop pair, plus its optional
// flex step. Departure and arrival times are both strictly increasing.
struct PairSteps {
  std::vector<int> deps;
  std::vector<int> arrs;
  int flex_seconds = -1;  // -1 if the pair has no flex step.

  // Earliest scheduled arrival departing at or after `t`;
  // kPairStepsUnreachable if none.
  int EarliestScheduledArrival(int t) const {
    auto it = std::lower_bound(deps.begin(), deps.end(), t);
    if (it == deps.end()) {
      return kPairStepsUnreachable;
    }
    return arrs[it - deps.begin()];
  }

  // Earliest arrival departing at or after `t`, flex included.
  int EarliestArrival(int t) const {
    int best = EarliestScheduledArrival(t);
    if (flex_seconds >= 0) {
      best = std::min(best, t + flex_seconds);
    }
    return best;
  }
};

// Dense table of PairSteps for all directed stop pairs of `list`, indexed
// a.v * NumStops() + b.v.
inline std::vector<PairSteps> BuildPairStepsTable(
    const StepsAdjacencyList& list
) {
  int n = list.NumStops();
  std::vector<PairSteps> pairs(static_cast<size_t>(n) * n);
  for (StopId a{0}; a.v < n; ++a.v) {
    for (const StepGroup& group : list.GetGroups(a)) {
      PairSteps& pair =
          pairs[static_cast<size_t>(a.v) * n + group.destination_stop.v];
      if (group.flex_step.has_value()) {
        pair.flex_seconds = group.flex_step->FlexDurationSeconds();
      }
      for (const AdjacencyListStep& step : list.GetSteps(group)) {
        pair.deps.push_back(step.origin_time.seconds);
        pair.arrs.push_back(step.destination_time.seconds);
      }
    }
  }
  return pairs;
}

}  // namespace vats5
