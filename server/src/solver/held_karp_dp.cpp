#include "solver/held_karp_dp.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"

namespace {

using namespace vats5;

struct HKDPGraph {
  RequiredStops required;
  CompactStopIdsResult compact;
};

// The subset of `required` whose stops appear in some step of `steps`, with
// each group's representative re-elected among the stops kept. Returns nullopt
// if a whole group disappears: with two or more groups every visit needs a leg
// in or out, so a group none of whose stops touches any step can never be
// visited and the problem is infeasible.
std::optional<RequiredStops> RequiredWithSteps(
    const std::vector<Step>& steps, const RequiredStops& required
) {
  std::unordered_set<StopId> present;
  for (const Step& step : steps) {
    present.insert(step.origin.stop);
    present.insert(step.destination.stop);
  }
  RequiredStops result;
  for (const std::vector<StopId>& group : required.Groups()) {
    std::vector<StopId> kept;
    for (StopId stop : group) {
      if (present.contains(stop)) {
        kept.push_back(stop);
      }
    }
    if (kept.empty()) {
      return std::nullopt;
    }
    for (StopId stop : kept) {
      result.representative[stop] = kept[0];
    }
  }
  return result;
}

// Returns nullopt if the problem is infeasible: some required group is
// disconnected from the other required stops. `required` must hold at least
// two groups.
std::optional<HKDPGraph> MakeHKDPGraph(
    const ProblemState& state, const RequiredStops& required
) {
  // A required stop in no step at all would fall outside the graphs the
  // completion below builds, so settle its fate first.
  std::optional<RequiredStops> required_with_steps =
      RequiredWithSteps(state.minimal.AllSteps(), required);
  if (!required_with_steps.has_value()) {
    return std::nullopt;
  }

  // Complete the graph for two reasons:
  // - The DP repeatedly queries for shortest paths, so this does all the work
  // upfront instead of repeating it.
  // - Stop ids are part of the index into dp state, so we want to compact them,
  // and to compact them we need to get rid of any non-required stops.
  std::vector<Step> completed_steps =
      CompleteShortestPathsGraph(
          state.minimal,
          required_with_steps->AllFlat(),
          HorizonCoveringAllDepartures(state.minimal)
      )
          .AllMergedSteps();

  // A required stop can also drop out here: one whose steps all lead to or
  // from non-required stops reaches no other required stop, so the completion
  // has no steps for it either.
  std::optional<RequiredStops> required_completed =
      RequiredWithSteps(completed_steps, *required_with_steps);
  if (!required_completed.has_value()) {
    return std::nullopt;
  }

  StepsAdjacencyList completed = MakeAdjacencyList(std::move(completed_steps));
  CompactStopIdsResult compact = CompactStopIds(completed);

  RequiredStops required_compacted;
  for (const auto& [stop, rep] : required_completed->representative) {
    required_compacted
        .representative[StopId{compact.mapping.original_to_new[stop.v]}] =
        StopId{compact.mapping.original_to_new[rep.v]};
  }

  return HKDPGraph{
      .required = required_compacted,
      .compact = compact,
  };
}

constexpr TimeSinceServiceStart kUnreachable{std::numeric_limits<int>::max()};

// O(1) replacement for the binary search in FindDepartureAtOrAfter, built once
// per solve for the compacted graph. For each group and each packed-time
// bucket in the group's departure span, stores the first step index whose
// packed departure is at or after that bucket. Queries share
// FindDepartureAtOrAfter's contract: the bucket lookup may undershoot (packing
// is lossy), and the forward scan on exact times fixes it up.
struct NextDepartureTable {
  struct GroupTable {
    int offset;  // Start of this group's entries in `entries`.
    int16_t first_bucket;
    int16_t last_bucket;  // Inclusive; less than first_bucket iff no steps.
  };
  std::vector<GroupTable> group_tables;  // Parallel to adj_list.groups.
  std::vector<int32_t> entries;          // Step indexes relative to the group.

  static NextDepartureTable Build(const StepsAdjacencyList& list) {
    NextDepartureTable table;
    table.group_tables.resize(list.groups.size());
    for (size_t gi = 0; gi < list.groups.size(); ++gi) {
      std::span<const int16_t> packed = list.GetDepartureTimes(list.groups[gi]);
      GroupTable& gt = table.group_tables[gi];
      if (packed.empty()) {
        gt = GroupTable{0, 0, -1};
        continue;
      }
      gt.offset = static_cast<int>(table.entries.size());
      gt.first_bucket = packed.front();
      gt.last_bucket = packed.back();
      size_t step_idx = 0;
      for (int bucket = gt.first_bucket; bucket <= gt.last_bucket; ++bucket) {
        while (packed[step_idx] < bucket) {
          ++step_idx;
        }
        table.entries.push_back(static_cast<int32_t>(step_idx));
      }
    }
    return table;
  }

  // Mirrors FindDepartureAtOrAfter: index of the first step in the group
  // departing at or after `t`, or steps.size() if none.
  size_t Find(
      std::span<const AdjacencyListStep> steps,
      size_t group_index,
      TimeSinceServiceStart t
  ) const {
    if (steps.empty()) {
      return steps.size();
    }
    const GroupTable& gt = group_tables[group_index];
    int packed_target = PackDepartureTimeForSearch(t);
    size_t idx;
    if (packed_target <= gt.first_bucket) {
      idx = 0;
    } else if (packed_target > gt.last_bucket) {
      return steps.size();
    } else {
      idx = static_cast<size_t>(
          entries[gt.offset + (packed_target - gt.first_bucket)]
      );
    }
    while (idx < steps.size() && steps[idx].origin_time.seconds < t.seconds) {
      ++idx;
    }
    return idx;
  }
};

}  // end namespace

namespace vats5 {

HeldKarpDPResult HeldKarpDPSolve(
    const ProblemState& state, std::ostream* search_log
) {
  HeldKarpDPResult infeasible{
      .best_val = kUnreachable.seconds,
      .best_tour = {},
  };

  StopId start = state.boundary.start;
  StopId end = state.boundary.end;

  // The boundary stops need not be required (constraints can erase their
  // groups), but they must be in the graph, so add each as its own group when
  // it is not required.
  RequiredStops graph_required = state.required;
  if (!graph_required.Contains(start)) {
    graph_required.representative[start] = start;
  }
  if (!graph_required.Contains(end)) {
    graph_required.representative[end] = end;
  }

  std::optional<HKDPGraph> maybe_graph = MakeHKDPGraph(state, graph_required);
  if (!maybe_graph.has_value()) {
    return infeasible;
  }
  HKDPGraph& graph = *maybe_graph;

  // A tour must start at START and end at END; if either dropped out of the
  // graph (it reaches no other required stop), the problem is infeasible.
  const std::vector<StopId>& original_to_new =
      graph.compact.mapping.original_to_new;
  if (start.v >= static_cast<int>(original_to_new.size()) ||
      original_to_new[start.v] == StopId{-1} ||
      end.v >= static_cast<int>(original_to_new.size()) ||
      original_to_new[end.v] == StopId{-1}) {
    return infeasible;
  }
  StopId compact_start = original_to_new[start.v];
  StopId compact_end = original_to_new[end.v];

  // The groups (rather than the stops) index the mask, and only the "middle"
  // groups get a mask bit: a group containing START or END is satisfied by the
  // endpoints themselves, so the tour's structure (start at START, end at END)
  // already guarantees it is visited.
  StopId start_rep = graph.required.Representative(compact_start);
  StopId end_rep = graph.required.Representative(compact_end);
  std::vector<int> stop_id_to_mask_index(graph.required.size(), -1);
  size_t n_mid_groups = 0;
  for (const StopId& rep_stop_id : graph.required.GroupRepresentatives()) {
    if (rep_stop_id == start_rep || rep_stop_id == end_rep) {
      continue;
    }
    stop_id_to_mask_index[rep_stop_id.v] = n_mid_groups;
    ++n_mid_groups;
  }
  for (StopId stop_id{0}; stop_id.v < stop_id_to_mask_index.size();
       ++stop_id.v) {
    StopId rep_stop_id = graph.required.Representative(stop_id);
    stop_id_to_mask_index[stop_id.v] = stop_id_to_mask_index[rep_stop_id.v];
  }

  size_t n_stops = graph.required.size();

  HeldKarpDPResult result{
      .best_val = kUnreachable.seconds,
      .best_tour = {},
  };

  TimeSinceServiceStart t_latest_dep{0};
  for (const Step& step : graph.compact.list.AllSteps()) {
    if (step.origin.time > t_latest_dep) {
      t_latest_dep = step.origin.time;
    }
  }

  // `dp[mask * n_stops + j]` is the earliest time we can get to stop `j` from
  // START, having visited all middle groups in `mask`.
  size_t dp_state_size = (size_t{1} << n_mid_groups) * n_stops;
  std::vector<TimeSinceServiceStart> dp(dp_state_size);

  // `back[s]` is the stop the transition that set `dp[s]` came from (START for
  // the base case). Kept separate from `dp` so the hot loop only touches `dp`.
  // One byte per state, which bounds the stop ids it can hold.
  if (n_stops > 256) {
    throw std::invalid_argument(
        "HeldKarpDPSolve: back-pointers require at most 256 stops in the "
        "compacted graph, got " +
        std::to_string(n_stops)
    );
  }
  // Never re-filled between sweep iterations: `back[s]` is only read where
  // `dp[s]` is reachable, and every `dp` write pairs with a `back` write.
  std::vector<uint8_t> back(dp_state_size);

  const StepsAdjacencyList& adj_list = graph.compact.list;

  NextDepartureTable next_departure_table = NextDepartureTable::Build(adj_list);

  // Earliest arrival at the destination of `adj_list.groups[group_index]` when
  // leaving its origin at `ready_time` or later.
  auto EarliestArrivalVia = [&](
                                size_t group_index,
                                TimeSinceServiceStart ready_time
                            ) -> std::optional<TimeSinceServiceStart> {
    const StepGroup& group = adj_list.groups[group_index];
    std::optional<TimeSinceServiceStart> best;
    if (group.flex_step.has_value()) {
      best = TimeSinceServiceStart{
          ready_time.seconds + group.flex_step->FlexDurationSeconds()
      };
    }
    std::span<const AdjacencyListStep> steps = adj_list.GetSteps(group);
    size_t step_idx = next_departure_table.Find(steps, group_index, ready_time);
    if (step_idx < steps.size() &&
        (!best.has_value() || steps[step_idx].destination_time < *best)) {
      best = steps[step_idx].destination_time;
    }
    return best;
  };

  // Group index (into `adj_list.groups`) of each stop's group of steps to END,
  // or -1 if it has none.
  std::vector<int> to_end_group_index(n_stops, -1);
  for (StopId a{0}; a.v < n_stops; ++a.v) {
    for (const StepGroup& group : adj_list.GetGroups(a)) {
      if (group.destination_stop == compact_end) {
        to_end_group_index[a.v] =
            static_cast<int>(&group - adj_list.groups.data());
      }
    }
  }

  MinimalPathSetCache path_cache(adj_list);

  TimeSinceServiceStart t_start{0};
  while (t_start <= t_latest_dep) {
    std::ranges::fill(dp, kUnreachable);

    // Base case: the tour leaves START at `t_start` or later, so seed each
    // middle stop reachable directly from START.
    for (const StepGroup& group : adj_list.GetGroups(compact_start)) {
      StopId b = group.destination_stop;
      int b_mask_index = stop_id_to_mask_index[b.v];
      if (b_mask_index == -1) {
        continue;
      }
      std::optional<TimeSinceServiceStart> arrival = EarliestArrivalVia(
          static_cast<size_t>(&group - adj_list.groups.data()), t_start
      );
      size_t dest_index = (size_t{1} << b_mask_index) * n_stops + b.v;
      if (arrival.has_value() && *arrival < dp[dest_index]) {
        dp[dest_index] = *arrival;
        back[dest_index] = static_cast<uint8_t>(compact_start.v);
      }
    }

    // Run DP.
    // Iterating masks in increasing numeric order guarantees a mask is fully
    // settled before any of its supersets (which are numerically larger) reads
    // from it, because this loop only propagates (mask, *) forwards.
    for (size_t mask = 1; mask < (size_t{1} << n_mid_groups); ++mask) {
      // Propagate (mask, *) forwards to all possible next stops.
      for (StopId a{0}; a.v < n_stops; ++a.v) {
        TimeSinceServiceStart a_time = dp[mask * n_stops + a.v];
        if (a_time == kUnreachable) {
          // Can't have `mask` ending up at `a`.
          continue;
        }
        for (const StepGroup& ab_group : adj_list.GetGroups(a)) {
          StopId b = ab_group.destination_stop;
          int b_mask_index = stop_id_to_mask_index[b.v];
          if (b_mask_index == -1) {
            // `b` is in START's or END's group: never a middle visit.
            continue;
          }
          size_t mask_with_b = mask | (size_t{1} << b_mask_index);
          if (mask_with_b == mask) {
            // `b` is already in `mask`, so don't revisit.
            continue;
          }
          size_t dest_index = mask_with_b * n_stops + b.v;
          std::optional<TimeSinceServiceStart> arrival = EarliestArrivalVia(
              static_cast<size_t>(&ab_group - adj_list.groups.data()), a_time
          );
          if (arrival.has_value() && *arrival < dp[dest_index]) {
            dp[dest_index] = *arrival;
            back[dest_index] = static_cast<uint8_t>(a.v);
          }
        }
      }
    }

    // Collect the earliest arrival at END over all final middle stops with
    // every middle group visited (or directly from START when there are none).
    size_t full_mask = (size_t{1} << n_mid_groups) - 1;
    TimeSinceServiceStart best_arrival = kUnreachable;
    StopId best_final_stop = compact_start;
    if (n_mid_groups == 0) {
      if (to_end_group_index[compact_start.v] != -1) {
        std::optional<TimeSinceServiceStart> arrival =
            EarliestArrivalVia(to_end_group_index[compact_start.v], t_start);
        if (arrival.has_value()) {
          best_arrival = *arrival;
        }
      }
    } else {
      for (StopId j{0}; j.v < n_stops; ++j.v) {
        TimeSinceServiceStart j_time = dp[full_mask * n_stops + j.v];
        if (j_time == kUnreachable || to_end_group_index[j.v] == -1) {
          continue;
        }
        std::optional<TimeSinceServiceStart> arrival =
            EarliestArrivalVia(to_end_group_index[j.v], j_time);
        if (arrival.has_value() && *arrival < best_arrival) {
          best_arrival = *arrival;
          best_final_stop = j;
        }
      }
    }
    if (best_arrival == kUnreachable) {
      // Waiting at a stop is always allowed, so any tour feasible from a later
      // start is also feasible from an earlier one. If no tour completes from
      // this start, none will from any later start either.
      break;
    }

    // Backtrack the visit order by following the back-pointers. END has no dp
    // state, so its predecessor is the final stop the collection above chose
    // (START directly when there are no middle groups); a middle stop's
    // predecessor is its back-pointer (START for seeds).
    size_t n_points = n_mid_groups + 2;
    std::vector<StopId> compact_tour(n_points);
    compact_tour[n_points - 1] = compact_end;
    {
      size_t cur_mask = full_mask;
      StopId cur_stop = n_mid_groups == 0 ? compact_start : best_final_stop;
      for (size_t i = n_points - 2;; --i) {
        compact_tour[i] = cur_stop;
        if (i == 0) {
          break;
        }
        StopId prev_stop{back[cur_mask * n_stops + cur_stop.v]};
        cur_mask &= ~(size_t{1} << stop_id_to_mask_index[cur_stop.v]);
        cur_stop = prev_stop;
      }
      assert(cur_mask == 0);
      assert(cur_stop == compact_start);
    }

    // Step forwards along the tour for its departure and duration: from the
    // minimal cover of whole-tour paths, pick the one leaving at or after
    // `t_start` that arrives earliest, leaving as late as possible. A flex
    // path leaves exactly at `t_start`.
    std::vector<Path> tour_paths =
        ComputeMinimalFeasiblePathsAlong(compact_tour, path_cache);
    TimeSinceServiceStart departure{-1};
    TimeSinceServiceStart arrival = kUnreachable;
    for (const Path& path : tour_paths) {
      TimeSinceServiceStart path_departure, path_arrival;
      if (path.merged_step.is_flex) {
        path_departure = t_start;
        path_arrival =
            TimeSinceServiceStart{t_start.seconds + path.DurationSeconds()};
      } else {
        path_departure = path.merged_step.origin.time;
        if (path_departure < t_start) {
          continue;
        }
        path_arrival = path.merged_step.destination.time;
      }
      if (path_arrival < arrival ||
          (path_arrival == arrival && path_departure > departure)) {
        arrival = path_arrival;
        departure = path_departure;
      }
    }
    if (arrival == kUnreachable) {
      // The dp found a chain along this tour departing at or after `t_start`,
      // so a path always exists.
      throw std::logic_error(
          "HeldKarpDPSolve: no path along the backtracked tour"
      );
    }
    assert(arrival == best_arrival);

    int dur = arrival.seconds - departure.seconds;
    t_start.seconds = departure.seconds + 1;
    if (dur < result.best_val) {
      result.best_val = dur;
      result.best_tour.clear();
      for (StopId stop : compact_tour) {
        result.best_tour.push_back(
            graph.compact.mapping.new_to_original[stop.v]
        );
      }
    }
    if (result.best_val == 0) {
      // A zero-duration tour can't be beaten, so skip the rest of the sweep.
      // TODO: This is a workaround to handle a common property-test case.
      // There is a more general problem: If there is a flex-only optimal tour,
      // then we'll step forwards one second at a time until we find an optimal
      // tour that is not flex-only. To fix this in general, we need to find a
      // way to correctly jump to the next departure time, accounting for the
      // possibility of flex prefixes. I think this won't happen much or ever in
      // practice.
      break;
    }
  }

  return result;
}

}  // namespace vats5
