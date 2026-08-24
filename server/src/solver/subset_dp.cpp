#include "solver/subset_dp.h"

#include <algorithm>
#include <bit>
#include <climits>
#include <stdexcept>
#include <vector>

namespace vats5 {

namespace {

// A DP label: one Pareto-optimal way of reaching the state's stop having
// visited the state's group set.
//
// Scheduled label (flex == false): departs the boundary start at `dep`,
// arrives at the stop at `arr`.
// Flex label (flex == true): the whole prefix is flexible; it takes
// `arr` seconds starting at any time (dep == 0).
struct Label {
  int dep;
  int arr;
  bool flex;
  // Predecessor state: member-stop index, or -1 for the boundary start.
  int pred_stop;
  // Index of the predecessor label within its (pruned) state.
  int pred_label;

  int DurationSeconds() const { return arr - dep; }
};

// A leg between two stops, extracted from the completed graph's minimal path
// group: an optional flex duration plus scheduled (departure, arrival) pairs
// sorted by departure, with a suffix-min over arrivals so "earliest arrival
// departing at or after t" is one binary search.
struct Leg {
  bool exists = false;
  std::optional<int> flex_duration;
  std::vector<int> dep_times;
  std::vector<int> arr_times;
  std::vector<int> arr_min_suffix;
  // Index minimizing (arr - dep), for closing flex-prefix tours.
  int best_duration_index = -1;

  static Leg FromPaths(std::span<const Path> paths) {
    Leg leg;
    if (paths.empty()) {
      return leg;
    }
    leg.exists = true;
    std::vector<std::pair<int, int>> dep_arr;
    for (const Path& p : paths) {
      const Step& s = p.merged_step;
      if (s.is_flex) {
        int d = s.DurationSeconds();
        if (!leg.flex_duration.has_value() || d < *leg.flex_duration) {
          leg.flex_duration = d;
        }
      } else {
        dep_arr.emplace_back(s.origin.time.seconds, s.destination.time.seconds);
      }
    }
    std::sort(dep_arr.begin(), dep_arr.end());
    leg.dep_times.reserve(dep_arr.size());
    leg.arr_times.reserve(dep_arr.size());
    for (const auto& [d, a] : dep_arr) {
      leg.dep_times.push_back(d);
      leg.arr_times.push_back(a);
    }
    leg.arr_min_suffix.assign(dep_arr.size(), INT_MAX);
    int min_arr = INT_MAX;
    int best_duration = INT_MAX;
    for (int i = static_cast<int>(dep_arr.size()) - 1; i >= 0; --i) {
      min_arr = std::min(min_arr, leg.arr_times[i]);
      leg.arr_min_suffix[i] = min_arr;
      int duration = leg.arr_times[i] - leg.dep_times[i];
      if (duration < best_duration) {
        best_duration = duration;
        leg.best_duration_index = i;
      }
    }
    return leg;
  }

  // Earliest arrival for a departure at or after `t`, with the matching label
  // arrival value. Returns nullopt if no scheduled step departs at or after t.
  std::optional<int> EarliestArrivalAfter(int t) const {
    auto it = std::lower_bound(dep_times.begin(), dep_times.end(), t);
    if (it == dep_times.end()) {
      return std::nullopt;
    }
    return arr_min_suffix[it - dep_times.begin()];
  }
};

// Prune a label set to the non-dominated frontier, mirroring the dominance
// rule of MakeMinimalCover: a label is dominated if another departs
// at-or-after and arrives at-or-before; at most one flex label survives, and
// scheduled labels lasting at least the flex duration are dominated by it.
void PruneLabels(std::vector<Label>& labels) {
  std::vector<Label> scheduled;
  std::optional<Label> best_flex;
  for (const Label& l : labels) {
    if (l.flex) {
      if (!best_flex.has_value() || l.arr < best_flex->arr) {
        best_flex = l;
      }
    } else {
      scheduled.push_back(l);
    }
  }
  int flex_duration = best_flex.has_value() ? best_flex->arr : INT_MAX;
  std::sort(
      scheduled.begin(), scheduled.end(), [](const Label& a, const Label& b) {
        if (a.dep != b.dep) {
          return a.dep > b.dep;
        }
        return a.arr < b.arr;
      }
  );
  labels.clear();
  if (best_flex.has_value()) {
    labels.push_back(*best_flex);
  }
  int best_arr = INT_MAX;
  for (const Label& l : scheduled) {
    if (l.arr < best_arr && l.DurationSeconds() < flex_duration) {
      labels.push_back(l);
      best_arr = l.arr;
    }
  }
}

}  // namespace

std::optional<SubsetDpResult> SubsetDpSolve(
    const ProblemState& state, std::ostream* log
) {
  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();

  StopId start = state.boundary.start;
  StopId end = state.boundary.end;

  // Mid-tour groups: all required groups except the boundary ones. The DP
  // assumes the boundary stops form singleton groups.
  std::vector<StopId> member_stops;
  std::vector<int> group_of_stop;
  int num_groups = 0;
  for (const std::vector<StopId>& group : state.required.Groups()) {
    bool has_boundary = false;
    for (StopId s : group) {
      if (s == start || s == end) {
        has_boundary = true;
      }
    }
    if (has_boundary) {
      if (group.size() != 1) {
        throw std::runtime_error(
            "SubsetDpSolve: boundary stop shares a required group"
        );
      }
      continue;
    }
    for (StopId s : group) {
      member_stops.push_back(s);
      group_of_stop.push_back(num_groups);
    }
    num_groups += 1;
  }
  if (num_groups > 24) {
    throw std::runtime_error(
        "SubsetDpSolve: too many required groups (" +
        std::to_string(num_groups) + ") for subset DP"
    );
  }
  int m = static_cast<int>(member_stops.size());
  int full_mask = (1 << num_groups) - 1;

  if (log != nullptr) {
    *log << "SubsetDpSolve: " << num_groups << " groups, " << m
         << " member stops\n";
  }

  // Legs. from: 0..m-1 = member stops, m = START. to: 0..m-1, m = END.
  std::vector<std::vector<Leg>> legs(m + 1, std::vector<Leg>(m + 1));
  for (int u = 0; u <= m; ++u) {
    StopId from = (u == m) ? start : member_stops[u];
    for (int w = 0; w <= m; ++w) {
      if (u == m && w == m) {
        continue;
      }
      StopId to = (w == m) ? end : member_stops[w];
      if (u < m && w < m && group_of_stop[u] == group_of_stop[w]) {
        continue;
      }
      legs[u][w] = Leg::FromPaths(completed.PathsBetween(from, to));
    }
  }

  // state_labels[mask * m + stop]: labels (unpruned until their level is
  // processed).
  std::vector<std::vector<Label>> state_labels(
      static_cast<size_t>(full_mask + 1) * m
  );
  auto StateIndex = [m](int mask, int stop) {
    return static_cast<size_t>(mask) * m + stop;
  };

  // Transition `label` (at member stop u, or the virtual start label for
  // u == -1) over `leg`, appending the produced candidate labels to `out`.
  auto Transition = [](const Label& label,
                       const Leg& leg,
                       int pred_stop,
                       int pred_label,
                       std::vector<Label>& out) {
    if (!leg.exists) {
      return;
    }
    if (label.flex) {
      int d = label.arr;
      if (leg.flex_duration.has_value()) {
        out.push_back(
            Label{0, d + *leg.flex_duration, true, pred_stop, pred_label}
        );
      }
      for (size_t i = 0; i < leg.dep_times.size(); ++i) {
        out.push_back(
            Label{
                leg.dep_times[i] - d,
                leg.arr_min_suffix[i],
                false,
                pred_stop,
                pred_label
            }
        );
      }
    } else {
      if (leg.flex_duration.has_value()) {
        out.push_back(
            Label{
                label.dep,
                label.arr + *leg.flex_duration,
                false,
                pred_stop,
                pred_label
            }
        );
      }
      std::optional<int> arr = leg.EarliestArrivalAfter(label.arr);
      if (arr.has_value()) {
        out.push_back(Label{label.dep, *arr, false, pred_stop, pred_label});
      }
    }
  };

  // Initial labels: the virtual start label is flex with duration 0.
  Label start_label{0, 0, true, -1, -1};
  for (int w = 0; w < m; ++w) {
    Transition(
        start_label,
        legs[m][w],
        -1,
        -1,
        state_labels[StateIndex(1 << group_of_stop[w], w)]
    );
  }

  // Masks by popcount so every state is complete before it is pruned and
  // expanded.
  std::vector<std::vector<int>> masks_by_popcount(num_groups + 1);
  for (int mask = 1; mask <= full_mask; ++mask) {
    masks_by_popcount[std::popcount(static_cast<unsigned>(mask))].push_back(
        mask
    );
  }

  int best_duration = INT_MAX;
  int best_stop = -1;
  int best_label = -1;
  int best_start_time = 0;
  int best_end_time = 0;

  size_t total_labels = 0;
  for (int k = 1; k <= num_groups; ++k) {
    size_t level_labels = 0;
    for (int mask : masks_by_popcount[k]) {
      for (int s = 0; s < m; ++s) {
        if ((mask & (1 << group_of_stop[s])) == 0) {
          continue;
        }
        std::vector<Label>& labels = state_labels[StateIndex(mask, s)];
        if (labels.empty()) {
          continue;
        }
        PruneLabels(labels);
        level_labels += labels.size();

        if (mask == full_mask) {
          // Close the tour via the leg to the boundary end.
          const Leg& leg = legs[s][m];
          if (!leg.exists) {
            continue;
          }
          for (int li = 0; li < static_cast<int>(labels.size()); ++li) {
            const Label& l = labels[li];
            if (l.flex) {
              if (leg.flex_duration.has_value()) {
                int duration = l.arr + *leg.flex_duration;
                if (duration < best_duration) {
                  best_duration = duration;
                  best_stop = s;
                  best_label = li;
                  best_start_time = 0;
                  best_end_time = duration;
                }
              }
              if (leg.best_duration_index >= 0) {
                int i = leg.best_duration_index;
                int duration = l.arr + leg.arr_times[i] - leg.dep_times[i];
                if (duration < best_duration) {
                  best_duration = duration;
                  best_stop = s;
                  best_label = li;
                  best_start_time = leg.dep_times[i] - l.arr;
                  best_end_time = leg.arr_times[i];
                }
              }
            } else {
              if (leg.flex_duration.has_value()) {
                int duration = l.arr + *leg.flex_duration - l.dep;
                if (duration < best_duration) {
                  best_duration = duration;
                  best_stop = s;
                  best_label = li;
                  best_start_time = l.dep;
                  best_end_time = l.arr + *leg.flex_duration;
                }
              }
              std::optional<int> arr = leg.EarliestArrivalAfter(l.arr);
              if (arr.has_value() && *arr - l.dep < best_duration) {
                best_duration = *arr - l.dep;
                best_stop = s;
                best_label = li;
                best_start_time = l.dep;
                best_end_time = *arr;
              }
            }
          }
          continue;
        }

        for (int w = 0; w < m; ++w) {
          int g = group_of_stop[w];
          if ((mask & (1 << g)) != 0) {
            continue;
          }
          std::vector<Label>& out =
              state_labels[StateIndex(mask | (1 << g), w)];
          for (int li = 0; li < static_cast<int>(labels.size()); ++li) {
            Transition(labels[li], legs[s][w], s, li, out);
          }
        }
      }
    }
    total_labels += level_labels;
    if (log != nullptr) {
      *log << "  level " << k << ": " << level_labels << " labels\n";
    }
  }
  if (log != nullptr) {
    *log << "  total labels: " << total_labels << "\n";
  }

  if (best_stop < 0) {
    return std::nullopt;
  }

  // Reconstruct the visit order by walking predecessor pointers.
  std::vector<StopId> sequence;
  int mask = full_mask;
  int s = best_stop;
  int li = best_label;
  while (s >= 0) {
    sequence.push_back(member_stops[s]);
    const Label& l = state_labels[StateIndex(mask, s)][li];
    mask &= ~(1 << group_of_stop[s]);
    s = l.pred_stop;
    li = l.pred_label;
  }
  std::reverse(sequence.begin(), sequence.end());
  sequence.insert(sequence.begin(), start);
  sequence.push_back(end);

  return SubsetDpResult{
      best_duration,
      TimeSinceServiceStart{best_start_time},
      TimeSinceServiceStart{best_end_time},
      std::move(sequence)
  };
}

}  // namespace vats5
