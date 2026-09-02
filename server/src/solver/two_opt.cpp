#include "solver/two_opt.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <climits>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "solver/pair_steps.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"

namespace vats5 {
namespace {

constexpr int kUnreachable = std::numeric_limits<int>::max();

// A PairSteps doubles as the Pareto front of ways to travel a section of the
// tour: scheduled elements are (departure from the section's first stop,
// arrival at its last), sorted by departure with no element dominated by
// another (later or equal departure and earlier or equal arrival), and
// flex_seconds is the duration of the best pure-flex chain (takeable at any
// time), or -1 if none. A single leg's PairSteps is the base case; Compose
// joins two adjacent sections into one, so a tour can be scored from
// precomputed sections instead of leg by leg.

bool Empty(const PairSteps& p) { return p.deps.empty() && p.flex_seconds < 0; }

int MinDuration(const PairSteps& p) {
  int best = p.flex_seconds >= 0 ? p.flex_seconds : kUnreachable;
  for (size_t i = 0; i < p.deps.size(); ++i) {
    best = std::min(best, p.arrs[i] - p.deps[i]);
  }
  return best;
}

// The empty section: ready to leave at any time, at no cost.
PairSteps Identity() {
  PairSteps p;
  p.flex_seconds = 0;
  return p;
}

void Clear(PairSteps& p) {
  p.deps.clear();
  p.arrs.clear();
  p.flex_seconds = -1;
}

// Sets `out` to the join of `cur` with the section `next` that starts where
// `cur` ends (`out` must be neither). Mirrors the path-merging semantics of
// ComputeMinimalFeasiblePathsAlong: waiting at a stop is free (the next
// scheduled departure at or after the arrival is taken), a flex leg departs
// exactly on arrival, and chains that would depart before 00:00:00 are
// dropped (departures only ever move earlier along a chain, so dropping them
// eagerly loses nothing).
void Compose(const PairSteps& cur, const PairSteps& next, PairSteps& out) {
  Clear(out);
  if (cur.flex_seconds >= 0 && next.flex_seconds >= 0) {
    out.flex_seconds = cur.flex_seconds + next.flex_seconds;
  }

  // The candidate elements come from two streams, each already sorted by
  // departure: each element of `cur` continued by the earliest arrival of
  // `next` after it, and, if `cur` has a pure-flex chain, each scheduled
  // element of `next` reached "just in time" by that chain. Merge the two
  // from the latest departure down, keeping elements that strictly improve
  // the arrival, which leaves exactly the Pareto front.
  size_t i = cur.deps.size();
  size_t j = cur.flex_seconds >= 0 ? next.deps.size() : 0;
  int min_arr = kUnreachable;
  while (i > 0 || j > 0) {
    int dep;
    int arr;
    // On equal departures, take the earlier arrival first so that the other
    // is dropped as dominated.
    bool take_j =
        j > 0 &&
        (i == 0 ||
         next.deps[j - 1] - cur.flex_seconds > cur.deps[i - 1] ||
         (next.deps[j - 1] - cur.flex_seconds == cur.deps[i - 1] &&
          next.arrs[j - 1] < next.EarliestArrival(cur.arrs[i - 1])));
    if (take_j) {
      --j;
      dep = next.deps[j] - cur.flex_seconds;
      if (dep < 0) {
        // Every remaining departure of this stream is earlier still.
        j = 0;
        continue;
      }
      arr = next.arrs[j];
    } else {
      --i;
      dep = cur.deps[i];
      arr = next.EarliestArrival(cur.arrs[i]);
      if (arr == kPairStepsUnreachable) {
        continue;
      }
    }
    if (arr < min_arr) {
      min_arr = arr;
      out.deps.push_back(dep);
      out.arrs.push_back(arr);
    }
  }
  std::reverse(out.deps.begin(), out.deps.end());
  std::reverse(out.arrs.begin(), out.arrs.end());
}

struct TwoOptGraph {
  ProblemBoundary boundary;  // In compacted stop ids.
  RequiredStops required;
  CompactStopIdsResult compact;
  std::vector<PairSteps> pairs;
  int n_stops;

  const PairSteps& Pair(StopId a, StopId b) const {
    return pairs[static_cast<size_t>(a.v) * n_stops + b.v];
  }
};

// May return nullopt if the problem is infeasible.
std::optional<TwoOptGraph> MakeTwoOptGraph(const ProblemState& state) {
  std::vector<Step> completed_steps =
      CompleteShortestPathsGraph(
          state.minimal,
          state.required.AllFlat(),
          HorizonCoveringAllDepartures(state.minimal)
      )
          .AllMergedSteps();
  StepsAdjacencyList completed = MakeAdjacencyList(std::move(completed_steps));
  CompactStopIdsResult compact = CompactStopIds(completed);

  std::optional<RequiredStops> required =
      RemapRequiredStops(compact.mapping, state.required);
  if (!required.has_value()) {
    return std::nullopt;
  }

  std::optional<StopId> start = compact.mapping.MapToNew(state.boundary.start);
  std::optional<StopId> end = compact.mapping.MapToNew(state.boundary.end);
  assert(start.has_value());
  assert(end.has_value());

  TwoOptGraph graph;
  graph.boundary = ProblemBoundary{.start = *start, .end = *end};
  graph.required = std::move(*required);
  graph.pairs = BuildPairStepsTable(compact.list);
  graph.n_stops = compact.list.NumStops();
  graph.compact = std::move(compact);
  return graph;
}

// The search state: which stop of each middle group is visited, and in which
// order the groups are visited.
struct Candidate {
  std::vector<int> order;   // Permutation of middle group indices.
  std::vector<int> chosen;  // Per middle group, index into its stop list.
};

// Scores the neighbors of a base candidate. A neighbor differs from the base
// in one section of the tour (a reversed segment, or one replaced stop), so
// its value is composed from precomputed sections of the base: the prefix
// before the change and the suffix after it. For a reversal, the reversed
// segment's cover is carried along as the segment grows by one leg at a time
// (see EvaluateReversal), so only the two legs joining the changed section
// to the rest are composed leg by leg for each move.
class Evaluator {
 public:
  Evaluator(
      const TwoOptGraph& graph,
      const std::vector<std::vector<StopId>>& mid_groups
  )
      : graph_(graph), mid_groups_(mid_groups) {}

  // Makes `c` the base candidate and returns the exact optimal duration of a
  // tour visiting its stops in its order, or kUnreachable if none exists.
  int SetBase(const Candidate& c) {
    seq_.clear();
    seq_.push_back(graph_.boundary.start);
    for (int g : c.order) {
      seq_.push_back(mid_groups_[g][c.chosen[g]]);
    }
    seq_.push_back(graph_.boundary.end);
    n_ = static_cast<int>(seq_.size());

    // The tables are resized (not reassigned) so their buffers are reused
    // across bases; every entry is cleared or recomputed below.
    // prefix_[b]: the section from seq_[0] to seq_[b].
    prefix_.resize(n_);
    ClearAll(prefix_);
    prefix_[0] = Identity();
    for (int b = 1; b < n_ && !Empty(prefix_[b - 1]); ++b) {
      Compose(prefix_[b - 1], graph_.Pair(seq_[b - 1], seq_[b]), prefix_[b]);
    }
    // suffix_[a]: the section from seq_[a] to seq_[n_ - 1].
    suffix_.resize(n_);
    ClearAll(suffix_);
    suffix_[n_ - 1] = Identity();
    for (int a = n_ - 2; a >= 0 && !Empty(suffix_[a + 1]); --a) {
      Compose(graph_.Pair(seq_[a], seq_[a + 1]), suffix_[a + 1], suffix_[a]);
    }

    ++evaluations_;
    return MinDuration(prefix_[n_ - 1]);
  }

  // The value of the base with order positions i..j (i < j) reversed. For a
  // given j, must be called with i = j - 1, j - 2, ... in that order: the
  // cover of the reversed segment (from seq_[j] back to seq_[i]) is kept
  // across calls and extended by one leg at its end per call. (Extending at
  // the end, rather than the start, keeps the composition in its cheap
  // orientation: a small cover followed by a leg with many departures.)
  int EvaluateReversal(int i, int j) {
    ++evaluations_;
    int si = i + 1;  // seq_ index of order position i.
    int sj = j + 1;
    if (i == j - 1) {
      Clear(reversed_);
      reversed_.flex_seconds = 0;  // Identity: the segment is just seq_[sj].
    } else {
      assert(j == reversed_j_ && i == reversed_i_ - 1);
    }
    reversed_i_ = i;
    reversed_j_ = j;
    Compose(reversed_, graph_.Pair(seq_[si + 1], seq_[si]), scratch_a_);
    std::swap(reversed_, scratch_a_);
    if (Empty(reversed_)) {
      return kUnreachable;
    }

    Compose(prefix_[si - 1], graph_.Pair(seq_[si - 1], seq_[sj]), scratch_a_);
    if (Empty(scratch_a_)) {
      return kUnreachable;
    }
    Compose(scratch_a_, reversed_, scratch_b_);
    if (Empty(scratch_b_)) {
      return kUnreachable;
    }
    Compose(scratch_b_, graph_.Pair(seq_[si], seq_[sj + 1]), scratch_a_);
    if (Empty(scratch_a_)) {
      return kUnreachable;
    }
    Compose(scratch_a_, suffix_[sj + 1], scratch_b_);
    return MinDuration(scratch_b_);
  }

  // The value of the base with the stop at order position p replaced by x.
  int EvaluateSwap(int p, StopId x) {
    ++evaluations_;
    int s = p + 1;  // seq_ index of order position p.
    Compose(prefix_[s - 1], graph_.Pair(seq_[s - 1], x), scratch_a_);
    if (Empty(scratch_a_)) {
      return kUnreachable;
    }
    Compose(scratch_a_, graph_.Pair(x, seq_[s + 1]), scratch_b_);
    if (Empty(scratch_b_)) {
      return kUnreachable;
    }
    Compose(scratch_b_, suffix_[s + 1], scratch_a_);
    return MinDuration(scratch_a_);
  }

  long long NumEvaluations() const { return evaluations_; }

 private:
  static void ClearAll(std::vector<PairSteps>& table) {
    for (PairSteps& p : table) {
      Clear(p);
    }
  }

  const TwoOptGraph& graph_;
  const std::vector<std::vector<StopId>>& mid_groups_;
  std::vector<StopId> seq_;
  int n_ = 0;
  std::vector<PairSteps> prefix_;
  std::vector<PairSteps> suffix_;
  // The reversed segment of the last EvaluateReversal call, and its i and j.
  PairSteps reversed_;
  int reversed_i_ = -1;
  int reversed_j_ = -1;
  PairSteps scratch_a_;
  PairSteps scratch_b_;
  long long evaluations_ = 0;
};

}  // namespace

TwoOptResult TwoOptSolve(
    const ProblemState& state,
    const TwoOptOptions& options,
    std::ostream* search_log
) {
  TwoOptResult result{.best_val = kUnreachable, .best_tour = {}};

  std::optional<TwoOptGraph> maybe_graph = MakeTwoOptGraph(state);
  if (!maybe_graph.has_value()) {
    return result;
  }
  const TwoOptGraph& graph = *maybe_graph;

  // Middle groups: required groups not containing START or END.
  StopId start_rep = graph.required.Representative(graph.boundary.start);
  StopId end_rep = graph.required.Representative(graph.boundary.end);
  std::vector<std::vector<StopId>> mid_groups;
  for (const std::vector<StopId>& group : graph.required.Groups()) {
    StopId rep = graph.required.Representative(group[0]);
    if (rep != start_rep && rep != end_rep) {
      mid_groups.push_back(group);
    }
  }
  int k = static_cast<int>(mid_groups.size());

  Evaluator evaluator(graph, mid_groups);
  auto deadline_passed = [&, start_time = std::chrono::steady_clock::now()]() {
    if (!options.time_limit_seconds.has_value()) {
      return false;
    }
    std::chrono::duration<double> elapsed =
        std::chrono::steady_clock::now() - start_time;
    return elapsed.count() >= *options.time_limit_seconds;
  };

  Candidate best_candidate;
  std::mt19937 rng(options.seed);

  for (int restart = 0; restart < options.restarts; ++restart) {
    if (deadline_passed()) {
      break;
    }

    auto restart_start = std::chrono::steady_clock::now();

    // Build a random initial candidate.
    Candidate c;
    c.chosen.assign(k, 0);
    c.order.resize(k);
    for (int g = 0; g < k; ++g) {
      c.order[g] = g;
    }
    std::shuffle(c.order.begin(), c.order.end(), rng);
    for (int g = 0; g < k; ++g) {
      if (mid_groups[g].size() > 1) {
        c.chosen[g] = std::uniform_int_distribution<
            int>(0, static_cast<int>(mid_groups[g].size()) - 1)(rng);
      }
    }

    int cur_val = evaluator.SetBase(c);

    // Best-improvement hill climbing over 2-opt segment reversals and
    // group-member swaps.
    // TODO: The evaluator could simply DP over group-member selection so that
    // we don't have to evaluate swap moves.
    bool improved = true;
    while (improved && !deadline_passed()) {
      improved = false;
      int best_move_val = cur_val;
      // The best move: a reversal of order positions i..j if best_j >= 0,
      // otherwise a swap of group best_g's stop to best_m.
      int best_i = -1;
      int best_j = -1;
      int best_g = -1;
      int best_m = -1;

      for (int j = 1; j < k; ++j) {
        for (int i = j - 1; i >= 0; --i) {
          int val = evaluator.EvaluateReversal(i, j);
          if (val < best_move_val) {
            best_move_val = val;
            best_i = i;
            best_j = j;
          }
        }
      }
      for (int p = 0; p < k; ++p) {
        int g = c.order[p];
        for (size_t m = 0; m < mid_groups[g].size(); ++m) {
          if (static_cast<int>(m) == c.chosen[g]) {
            continue;
          }
          int val = evaluator.EvaluateSwap(p, mid_groups[g][m]);
          if (val < best_move_val) {
            best_move_val = val;
            best_j = -1;
            best_g = g;
            best_m = static_cast<int>(m);
          }
        }
      }

      if (best_move_val < cur_val) {
        if (best_j >= 0) {
          std::reverse(
              c.order.begin() + best_i, c.order.begin() + best_j + 1
          );
        } else {
          c.chosen[best_g] = best_m;
        }
        cur_val = evaluator.SetBase(c);
        assert(cur_val == best_move_val);
        improved = true;
      }
    }

    ++result.restarts_completed;
    result.restart_seconds.push_back(
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - restart_start
        )
            .count()
    );
    if (cur_val < result.best_val) {
      result.best_val = cur_val;
      best_candidate = c;
    }
    if (search_log != nullptr) {
      *search_log << "restart " << restart << ": local opt "
                  << (cur_val == kUnreachable
                          ? "infeasible"
                          : TimeSinceServiceStart{cur_val}.ToString())
                  << ", global best "
                  << (result.best_val == kUnreachable
                          ? "infeasible"
                          : TimeSinceServiceStart{result.best_val}.ToString())
                  << "\n";
    }
  }

  result.evaluations = evaluator.NumEvaluations();
  if (result.best_val == kUnreachable) {
    return result;
  }

  result.best_tour.push_back(
      graph.compact.mapping.new_to_original[graph.boundary.start.v]
  );
  for (int g : best_candidate.order) {
    StopId stop = mid_groups[g][best_candidate.chosen[g]];
    result.best_tour.push_back(graph.compact.mapping.new_to_original[stop.v]);
  }
  result.best_tour.push_back(
      graph.compact.mapping.new_to_original[graph.boundary.end.v]
  );
  return result;
}

}  // namespace vats5
