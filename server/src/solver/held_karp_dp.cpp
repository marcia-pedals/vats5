#include "solver/held_karp_dp.h"

#include <algorithm>
#include <bit>
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
  ProblemBoundary boundary;
  RequiredStops required;
  CompactStopIdsResult compact;
};

// May return nullopt if the problem is infeasible.
std::optional<HKDPGraph> MakeHKDPGraph(const ProblemState& state) {
  // Complete the graph for two reasons:
  // - The DP repeatedly queries for shortest paths, so this does all the work
  // upfront instead of repeating it.
  // - Stop ids are part of the index into dp state, so we want to compact them,
  // and to compact them we need to get rid of any non-required stops.
  std::vector<Step> completed_steps =
      CompleteShortestPathsGraph(
          state.minimal,
          state.required.AllFlat(),
          HorizonCoveringAllDepartures(state.minimal)
      )
          .AllMergedSteps();

  StepsAdjacencyList completed = MakeAdjacencyList(std::move(completed_steps));
  CompactStopIdsResult compact = CompactStopIds(completed);
  std::optional<RequiredStops> required_mapped =
      RemapRequiredStops(compact.mapping, state.required);
  if (!required_mapped.has_value()) {
    return std::nullopt;
  }

  std::optional<StopId> start_mapped =
      compact.mapping.MapToNew(state.boundary.start);
  std::optional<StopId> end_mapped =
      compact.mapping.MapToNew(state.boundary.end);
  assert(start_mapped.has_value());
  assert(end_mapped.has_value());

  return HKDPGraph{
      .boundary = ProblemBoundary{.start = *start_mapped, .end = *end_mapped},
      .required = *required_mapped,
      .compact = compact,
  };
}

constexpr TimeSinceServiceStart kUnreachable{std::numeric_limits<int>::max()};

// Everything the DP's hot loop needs from the compacted complete graph, packed
// into flat arrays indexed by directed stop pair `a * n_stops + b` so the loop
// never touches the StepsAdjacencyList. Densifying over pairs is affordable
// because the completed graph is dense anyways. Queries keep the
// FindDepartureAtOrAfter contract: the bucket lookup may undershoot (buckets
// are coarser than seconds), and the forward scan on exact times fixes it up.
struct DensePairTable {
  static constexpr int32_t kNoStep = std::numeric_limits<int32_t>::max();

  // Timed steps slimmed to the two times the DP needs, grouped by pair and
  // sorted by departure, with a {kNoStep, kNoStep} sentinel terminating each
  // pair's run so queries need no end-of-pair bound: a query that runs off the
  // real steps reads the sentinel's kNoStep arrival, which never improves
  // anything.
  struct SlimStep {
    int32_t dep;
    int32_t arr;
  };
  std::vector<SlimStep> steps;

  // Index into `steps` of the first step of pair P departing at or after each
  // bucket's start (the pair's sentinel if none), indexed
  // bucket * n_pairs + P. Bucket-major because the DP's inner loop queries one
  // ready time (one bucket) against a contiguous run of pairs, so its lookups
  // all land in one row. Buckets are uniform over the whole graph's departure
  // span, so a query is a single multiply-add.
  std::vector<int32_t> bucket_to_step;
  int n_buckets;
  int bucket_size_seconds;
  int bucket0_seconds;  // Departure time at the start of bucket 0.

  // Flex duration of each pair, kNoStep if the pair has no flex step.
  std::vector<int32_t> flex_seconds;

  int n_stops;

  static DensePairTable Build(const StepsAdjacencyList& list, int n_stops) {
    DensePairTable table;
    table.n_stops = n_stops;
    size_t n_pairs = static_cast<size_t>(n_stops) * n_stops;
    table.flex_seconds.assign(n_pairs, kNoStep);

    // MakeAdjacencyList merges all steps of a directed pair into one group, so
    // each pair's steps are a single sorted run.
    std::vector<std::span<const AdjacencyListStep>> pair_steps(n_pairs);
    int min_dep = std::numeric_limits<int>::max();
    int max_dep = std::numeric_limits<int>::min();
    size_t n_timed = 0;
    for (StopId a{0}; a.v < n_stops; ++a.v) {
      for (const StepGroup& group : list.GetGroups(a)) {
        size_t pair =
            static_cast<size_t>(a.v) * n_stops + group.destination_stop.v;
        if (group.flex_step.has_value()) {
          table.flex_seconds[pair] = group.flex_step->FlexDurationSeconds();
        }
        std::span<const AdjacencyListStep> steps = list.GetSteps(group);
        pair_steps[pair] = steps;
        n_timed += steps.size();
        if (!steps.empty()) {
          min_dep = std::min(min_dep, steps.front().origin_time.seconds);
          max_dep = std::max(max_dep, steps.back().origin_time.seconds);
        }
      }
    }

    // Start at the packing resolution the adjacency list uses and coarsen
    // until the bucket table is reasonably small; coarser buckets only
    // lengthen the fix-up scan.
    constexpr size_t kMaxBucketEntries = size_t{1} << 22;  // 16 MB of int32.
    table.bucket_size_seconds = kDepartureTimeResolutionSeconds;
    table.bucket0_seconds = n_timed == 0 ? 0 : min_dep;
    if (n_timed == 0) {
      table.n_buckets = 1;
    } else {
      while (true) {
        size_t n_buckets =
            static_cast<size_t>(max_dep - min_dep) / table.bucket_size_seconds +
            1;
        if (n_pairs * n_buckets <= kMaxBucketEntries) {
          table.n_buckets = static_cast<int>(n_buckets);
          break;
        }
        table.bucket_size_seconds *= 2;
      }
    }

    table.steps.reserve(n_timed + n_pairs);
    table.bucket_to_step.resize(n_pairs * table.n_buckets);
    for (size_t pair = 0; pair < n_pairs; ++pair) {
      size_t idx = table.steps.size();
      for (const AdjacencyListStep& step : pair_steps[pair]) {
        table.steps.push_back(
            SlimStep{step.origin_time.seconds, step.destination_time.seconds}
        );
      }
      table.steps.push_back(SlimStep{kNoStep, kNoStep});
      for (int bucket = 0; bucket < table.n_buckets; ++bucket) {
        int bucket_start =
            table.bucket0_seconds + bucket * table.bucket_size_seconds;
        while (table.steps[idx].dep < bucket_start) {
          ++idx;
        }
        table.bucket_to_step[static_cast<size_t>(bucket) * n_pairs + pair] =
            static_cast<int32_t>(idx);
      }
    }
    return table;
  }

  // Start of `ready_seconds`'s row in `bucket_to_step`. Hoistable out of any
  // loop that queries many pairs at one ready time.
  size_t BucketRowBase(int32_t ready_seconds) const {
    int64_t offset = static_cast<int64_t>(ready_seconds) - bucket0_seconds;
    int bucket = offset <= 0 ? 0
                             : static_cast<int>(std::min<int64_t>(
                                   offset / bucket_size_seconds, n_buckets - 1
                               ));
    return static_cast<size_t>(bucket) * n_stops * n_stops;
  }

  // Earliest arrival at `b` leaving `a` at `ready_seconds` or later, over the
  // pair's flex step and timed steps; kNoStep if unreachable. Correct because
  // each pair's steps are a minimal cover: the first departure at or after
  // `ready_seconds` also has the earliest arrival. `ready_seconds` must be a
  // real time, not the kUnreachable sentinel, and `row_base` must be
  // `BucketRowBase(ready_seconds)`.
  int32_t EarliestArrival(
      size_t pair, int32_t ready_seconds, size_t row_base
  ) const {
    int32_t best = flex_seconds[pair];
    if (best != kNoStep) {
      best += ready_seconds;
    }
    size_t idx = bucket_to_step[row_base + pair];
    while (steps[idx].dep < ready_seconds) {
      ++idx;
    }
    return std::min(best, steps[idx].arr);
  }

  int32_t EarliestArrival(size_t pair, int32_t ready_seconds) const {
    return EarliestArrival(pair, ready_seconds, BucketRowBase(ready_seconds));
  }
};

// Binomial coefficients binom[n][k] for 0 <= n, k <= n_max, with
// binom[n][k] == 0 for k > n.
std::vector<std::vector<size_t>> MakeBinomialTable(int n_max) {
  std::vector<std::vector<size_t>> binom(
      n_max + 1, std::vector<size_t>(n_max + 1, 0)
  );
  for (int n = 0; n <= n_max; ++n) {
    binom[n][0] = 1;
    for (int k = 1; k <= n; ++k) {
      binom[n][k] = binom[n - 1][k - 1] + (k <= n - 1 ? binom[n - 1][k] : 0);
    }
  }
  return binom;
}

}  // end namespace

namespace vats5 {

HeldKarpDPResult HeldKarpDPSolve(
    const ProblemState& state, int known_lb, std::ostream* search_log
) {
  HeldKarpDPResult result{
      .best_val = kUnreachable.seconds,
      .best_tour = {},
  };

  std::optional<HKDPGraph> maybe_graph = MakeHKDPGraph(state);
  if (!maybe_graph.has_value()) {
    return result;
  }
  const HKDPGraph& graph = *maybe_graph;

  // The groups (rather than the stops) index the mask, and only the "middle"
  // groups get a mask bit: a group containing START or END is satisfied by the
  // endpoints themselves, so the tour's structure (start at START, end at END)
  // already guarantees it is visited.
  StopId start_rep = graph.required.Representative(graph.boundary.start);
  StopId end_rep = graph.required.Representative(graph.boundary.end);
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

  TimeSinceServiceStart t_latest_dep{0};
  for (const Step& step : graph.compact.list.AllSteps()) {
    if (step.origin.time > t_latest_dep) {
      t_latest_dep = step.origin.time;
    }
  }

  // The dp state is layered by mask popcount: every transition goes from a
  // mask to a superset with exactly one more bit, so only the current and next
  // popcount layers are ever live. Layer `p` lives in `dp_layers[p % 2]`,
  // where `dp_layers[p % 2][rank * n_stops + j]` is the earliest time we can
  // get to stop `j` from START, having visited exactly the middle groups in
  // the layer's rank-th mask of popcount `p` — masks rank within a layer by
  // increasing numeric value (the combinatorial number system: the i-th
  // smallest set bit, 1-based, at position `pos` contributes binom[pos][i]).
  // This caps dp memory at two layers of C(n_mid_groups, n_mid_groups / 2)
  // masks instead of all 2^n_mid_groups.
  // At least 2x2 so the seed fill below may read binom[0][1] (= 0) when there
  // are no middle groups.
  std::vector<std::vector<size_t>> binom =
      MakeBinomialTable(std::max<int>(n_mid_groups, 1));
  size_t max_layer_masks = binom[n_mid_groups][n_mid_groups / 2];
  std::vector<TimeSinceServiceStart> dp_layers[2];
  dp_layers[0].resize(max_layer_masks * n_stops);
  dp_layers[1].resize(max_layer_masks * n_stops);

  // `back[mask * n_stops + j]` is the stop the transition that set the dp
  // state for (mask, j) came from (START for the base case). Unlike the dp
  // values it spans all masks, because the backtrack needs the whole chain.
  // One byte per state, which bounds the stop ids it can hold.
  if (n_stops > 256) {
    throw std::invalid_argument(
        "HeldKarpDPSolve: back-pointers require at most 256 stops in the "
        "compacted graph, got " +
        std::to_string(n_stops)
    );
  }
  // Never re-filled between sweep iterations: `back[s]` is only read where
  // the dp state `s` is reachable, and every dp write pairs with a `back`
  // write.
  std::vector<uint8_t> back((size_t{1} << n_mid_groups) * n_stops);

  const StepsAdjacencyList& adj_list = graph.compact.list;

  DensePairTable table =
      DensePairTable::Build(adj_list, static_cast<int>(n_stops));

  // Each stop's mask bit, 0 for boundary-group stops (which have no mask
  // index and are never transitioned into). Precomputed so the DP hot loop
  // skips and applies a stop's bit with a single lookup.
  std::vector<size_t> stop_bit(n_stops);
  for (StopId b{0}; b.v < n_stops; ++b.v) {
    int mask_index = stop_id_to_mask_index[b.v];
    stop_bit[b.v] = mask_index == -1 ? 0 : size_t{1} << mask_index;
  }

  MinimalPathSetCache path_cache(adj_list);

  // Per-mask candidate stops: the middle stops whose group is not yet in the
  // mask, with their destination indices into the next dp layer and into
  // `back`. All depend only on the mask, so they are computed once per mask
  // (lazily, on its first reachable source stop) and iterated as small
  // contiguous arrays in the hot loop instead of re-deriving them from
  // `stop_bit` for every (source, destination) pair.
  std::vector<uint8_t> cand_b(n_stops);
  std::vector<size_t> cand_dest(n_stops);
  std::vector<size_t> cand_back(n_stops);

  // `next_rank[q]` is the rank of `mask | (1 << q)` in the next layer, for
  // each group bit `q` not in the current `mask`; scratch for the per-mask
  // candidate computation. `binom_flat` is `binom` flattened to a single
  // array (binom[q][k] at q * binom_stride + k) so that computation reads it
  // without a second indirection.
  std::vector<size_t> next_rank(n_mid_groups);
  size_t binom_stride = binom.size();
  std::vector<size_t> binom_flat;
  binom_flat.reserve(binom_stride * binom_stride);
  for (const std::vector<size_t>& row : binom) {
    binom_flat.insert(binom_flat.end(), row.begin(), row.end());
  }

  TimeSinceServiceStart t_start{0};
  while (t_start <= t_latest_dep) {
    // Base case (layer 1): the tour leaves START at `t_start` or later, so
    // seed each middle stop reachable directly from START. The single-bit
    // mask `1 << i` has rank `i` in its layer.
    std::vector<TimeSinceServiceStart>& dp_seed = dp_layers[1 % 2];
    std::fill_n(
        dp_seed.begin(), binom[n_mid_groups][1] * n_stops, kUnreachable
    );
    for (StopId b{0}; b.v < n_stops; ++b.v) {
      size_t bit = stop_bit[b.v];
      if (bit == 0) {
        continue;
      }
      int32_t arrival = table.EarliestArrival(
          static_cast<size_t>(graph.boundary.start.v) * n_stops + b.v,
          t_start.seconds
      );
      size_t dest_index =
          static_cast<size_t>(stop_id_to_mask_index[b.v]) * n_stops + b.v;
      if (arrival < dp_seed[dest_index].seconds) {
        dp_seed[dest_index] = TimeSinceServiceStart{arrival};
        back[bit * n_stops + b.v] =
            static_cast<uint8_t>(graph.boundary.start.v);
      }
    }

    // Run DP layer by layer: propagate every mask of popcount `p` into the
    // masks of popcount `p + 1`, so a layer is fully settled before anything
    // reads from it and only two layers are live at once.
    for (int p = 1; p < n_mid_groups; ++p) {
      const std::vector<TimeSinceServiceStart>& dp_cur = dp_layers[p % 2];
      std::vector<TimeSinceServiceStart>& dp_next = dp_layers[(p + 1) % 2];
      std::fill_n(
          dp_next.begin(), binom[n_mid_groups][p + 1] * n_stops, kUnreachable
      );
      // Gosper's hack enumerates the layer's masks in increasing numeric
      // order, which is exactly their order in the combinatorial number
      // system, so the running loop index is each mask's rank in the layer.
      size_t mask = (size_t{1} << p) - 1;
      size_t layer_masks = binom[n_mid_groups][p];
      for (size_t rank = 0; rank < layer_masks; ++rank) {
        int n_cand = -1;
        size_t cur_base = rank * n_stops;
        // Propagate (mask, *) forwards to all possible next stops.
        for (StopId a{0}; a.v < n_stops; ++a.v) {
          TimeSinceServiceStart a_time = dp_cur[cur_base + a.v];
          if (a_time == kUnreachable) {
            // Can't have `mask` ending up at `a`.
            continue;
          }
          if (n_cand < 0) {
            n_cand = 0;
            // Rank every one-more-bit superset of `mask` as a delta off the
            // mask's own rank (the Gosper loop counter): inserting a bit at
            // clear position `q` leaves set bits below `q` (1-based index
            // `j`) contributing binom[q'][j] as before, adds binom[q][j + 1]
            // for the new bit, and shifts each set bit above `q` up one index
            // so it contributes binom[q'][j + 1] instead of binom[q'][j].
            // One descending pass accumulates those shift deltas; `suffix` is
            // modular (deltas may be negative) but every final rank is exact.
            {
              size_t suffix = 0;
              int above = 0;
              for (int q = n_mid_groups - 1; q >= 0; --q) {
                const size_t* binom_q = &binom_flat[q * binom_stride];
                if ((mask >> q) & 1) {
                  int j = p - above;
                  suffix += binom_q[j + 1] - binom_q[j];
                  ++above;
                } else {
                  next_rank[q] = rank + suffix + binom_q[p - above + 1];
                }
              }
            }
            for (StopId b{0}; b.v < n_stops; ++b.v) {
              size_t bit = stop_bit[b.v];
              if (bit == 0 || (mask & bit) != 0) {
                // `b` is a boundary-group stop, or its group is already in
                // `mask`, so don't (re)visit.
                continue;
              }
              cand_b[n_cand] = static_cast<uint8_t>(b.v);
              cand_dest[n_cand] =
                  next_rank[stop_id_to_mask_index[b.v]] * n_stops + b.v;
              cand_back[n_cand] = (mask | bit) * n_stops + b.v;
              ++n_cand;
            }
          }
          size_t pair_base = static_cast<size_t>(a.v) * n_stops;
          size_t row_base = table.BucketRowBase(a_time.seconds);
          for (int k = 0; k < n_cand; ++k) {
            size_t dest_index = cand_dest[k];
            if (dp_next[dest_index] <= a_time) {
              // No step goes backwards in time, so an arrival from `a_time`
              // can't improve on this.
              continue;
            }
            int32_t arrival = table.EarliestArrival(
                pair_base + cand_b[k], a_time.seconds, row_base
            );
            if (arrival < dp_next[dest_index].seconds) {
              dp_next[dest_index] = TimeSinceServiceStart{arrival};
              back[cand_back[k]] = static_cast<uint8_t>(a.v);
            }
          }
        }
        size_t carry = mask + (mask & (~mask + 1));
        mask = (((mask ^ carry) >> (std::countr_zero(mask) + 2))) | carry;
      }
    }

    // Collect the earliest arrival at END over all final middle stops with
    // every middle group visited (or directly from START when there are none).
    size_t full_mask = (size_t{1} << n_mid_groups) - 1;
    TimeSinceServiceStart best_arrival = kUnreachable;
    StopId best_final_stop = graph.boundary.start;
    if (n_mid_groups == 0) {
      best_arrival = TimeSinceServiceStart{table.EarliestArrival(
          static_cast<size_t>(graph.boundary.start.v) * n_stops +
              graph.boundary.end.v,
          t_start.seconds
      )};
    } else {
      // The full mask is its layer's only member, at rank 0 of the layer the
      // propagation loop wrote last.
      const std::vector<TimeSinceServiceStart>& dp_full =
          dp_layers[n_mid_groups % 2];
      for (StopId j{0}; j.v < n_stops; ++j.v) {
        TimeSinceServiceStart j_time = dp_full[j.v];
        if (j_time == kUnreachable) {
          continue;
        }
        TimeSinceServiceStart arrival{table.EarliestArrival(
            static_cast<size_t>(j.v) * n_stops + graph.boundary.end.v,
            j_time.seconds
        )};
        if (arrival < best_arrival) {
          best_arrival = arrival;
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
    compact_tour[n_points - 1] = graph.boundary.end;
    {
      size_t cur_mask = full_mask;
      StopId cur_stop =
          n_mid_groups == 0 ? graph.boundary.start : best_final_stop;
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
      assert(cur_stop == graph.boundary.start);
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

    // Separately (!), set the best val to the shortest duration path. This
    // might be a different one from the one departing earliest after t_start,
    // but that's fine, if we find a better path, might as well use that one to
    // speed up the sweep.
    for (const Path& path : tour_paths) {
      if (path.DurationSeconds() < result.best_val) {
        result.best_val = path.DurationSeconds();
        result.best_tour.clear();
        for (StopId stop : compact_tour) {
          result.best_tour.push_back(
              graph.compact.mapping.new_to_original[stop.v]
          );
        }
      }
    }

    if (search_log != nullptr) {
      *search_log << "t_start " << t_start << ", current opt "
                  << TimeSinceServiceStart{result.best_val} << "\n";
    }

    // We can jump forwards this much without missing any better paths, because
    // if there is a path better than `result.best_val` departing before
    // `arrival.seconds - result.best_val`, then:
    //
    // (1) it arrives before `arrival.seconds`,
    //
    // (2) it departs at or after `arrival.seconds - result.best_val >=
    // arrival.seconds - (arrival.seconds - departure.seconds) =
    // departure.seconds >= t_start`.
    //
    // So it would have been a better path for the current iteration. and we
    // would have found it in the current iteration.
    t_start.seconds = arrival.seconds - result.best_val + 1;

    if (result.best_val <= known_lb) {
      // The best tour achieves a duration the caller knows can't be beaten,
      // so skip the rest of the sweep.
      if (search_log != nullptr) {
        *search_log << "Sweep terminated: reached known_lb ("
                    << TimeSinceServiceStart{known_lb} << ")\n";
      }
      break;
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
