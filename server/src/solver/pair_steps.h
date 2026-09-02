#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

inline constexpr int kPairStepsUnreachable = std::numeric_limits<int>::max();

// The sorted minimal cover of ways to travel from one stop to another: the
// scheduled steps as (departure, arrival) pairs sorted by departure, with no
// step dominated by another (later or equal departure and earlier or equal
// arrival), so both times are strictly increasing; and the optional flex step
// (takeable at any time). Built for a single directed stop pair by
// BuildPairStepsTable, and for a section of a tour by composing the pairs of
// its legs (see Compose), so a section is scored like a single leg.
struct PairSteps {
  std::vector<int> deps;
  std::vector<int> arrs;
  int flex_seconds = -1;  // -1 if the pair has no flex step.

  // No way to travel at all.
  bool Empty() const { return deps.empty() && flex_seconds < 0; }

  void Clear() {
    deps.clear();
    arrs.clear();
    flex_seconds = -1;
  }

  // The shortest duration of any step departing at or after
  // `earliest_departure` (the flex step always can);
  // kPairStepsUnreachable if none.
  int MinDuration(int earliest_departure) const {
    int best = flex_seconds >= 0 ? flex_seconds : kPairStepsUnreachable;
    auto it = std::lower_bound(deps.begin(), deps.end(), earliest_departure);
    for (size_t i = it - deps.begin(); i < deps.size(); ++i) {
      best = std::min(best, arrs[i] - deps[i]);
    }
    return best;
  }

  // The cover view of MergeCovers (see step_merge.h).
  bool HasFlex() const { return flex_seconds >= 0; }
  int FlexSeconds() const { return flex_seconds; }
  size_t NumScheduled() const { return deps.size(); }
  int Dep(size_t i) const { return deps[i]; }
  int Arr(size_t i) const { return arrs[i]; }

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
static_assert(CoverView<PairSteps>);

// Sets `out` to the cover of the two legs `cur` then `next` taken in turn
// (`out` must be neither). See MergeCovers for the semantics.
inline void Compose(
    const PairSteps& cur, const PairSteps& next, PairSteps& out
) {
  struct Sink {
    PairSteps& out;
    void Flex(int seconds) { out.flex_seconds = seconds; }
    void Scheduled(int dep, int arr, size_t, size_t) {
      out.deps.push_back(dep);
      out.arrs.push_back(arr);
    }
  };
  out.Clear();
  Sink sink{out};
  MergeCovers(cur, next, sink);
  std::reverse(out.deps.begin(), out.deps.end());
  std::reverse(out.arrs.begin(), out.arrs.end());
}

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
