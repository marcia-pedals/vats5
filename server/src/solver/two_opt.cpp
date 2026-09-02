#include "solver/two_opt.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <climits>
#include <limits>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "solver/pair_steps.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"

namespace vats5 {
namespace {

constexpr int kUnreachable = std::numeric_limits<int>::max();

// The Pareto front of ways to travel from START through a prefix of the tour:
// scheduled elements are (departure from the first stop, arrival at the
// current stop), sorted by departure with no element dominated by another
// (later or equal departure and earlier or equal arrival). `flex` is the
// duration of the best pure-flex chain (takeable at any time), or -1 if none.
struct Cover {
  std::vector<std::pair<int, int>> sched;
  int flex = -1;

  bool Empty() const { return sched.empty() && flex < 0; }

  int MinDuration() const {
    int best = flex >= 0 ? flex : kUnreachable;
    for (const auto& [dep, arr] : sched) {
      best = std::min(best, arr - dep);
    }
    return best;
  }
};

// Extends `cur` by one leg. Mirrors the path-merging semantics of
// ComputeMinimalFeasiblePathsAlong: waiting at a stop is free (the next
// scheduled departure at or after the arrival is taken), a flex leg departs
// exactly on arrival, and chains that would depart before 00:00:00 are
// dropped (departures only ever move earlier along a chain, so dropping them
// eagerly loses nothing).
Cover ComposeLeg(const Cover& cur, const PairSteps& pair) {
  Cover out;
  if (cur.flex >= 0 && pair.flex_seconds >= 0) {
    out.flex = cur.flex + pair.flex_seconds;
  }

  std::vector<std::pair<int, int>> cand;
  cand.reserve(cur.sched.size() + pair.deps.size());
  for (const auto& [dep, arr] : cur.sched) {
    int best = pair.EarliestArrival(arr);
    if (best != kPairStepsUnreachable) {
      cand.emplace_back(dep, best);
    }
  }
  if (cur.flex >= 0) {
    // A pure-flex prefix arrives "just in time" for every scheduled
    // departure of this leg.
    for (size_t i = 0; i < pair.deps.size(); ++i) {
      int dep = pair.deps[i] - cur.flex;
      if (dep >= 0) {
        cand.emplace_back(dep, pair.arrs[i]);
      }
    }
  }

  // Keep the Pareto front: sort by departure and sweep from the latest
  // departure, keeping elements that strictly improve the arrival.
  std::sort(cand.begin(), cand.end());
  int min_arr = kUnreachable;
  for (size_t i = cand.size(); i-- > 0;) {
    if (cand[i].second < min_arr) {
      min_arr = cand[i].second;
      out.sched.push_back(cand[i]);
    }
  }
  std::reverse(out.sched.begin(), out.sched.end());
  return out;
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

class Evaluator {
 public:
  Evaluator(
      const TwoOptGraph& graph,
      const std::vector<std::vector<StopId>>& mid_groups
  )
      : graph_(graph), mid_groups_(mid_groups) {}

  // The exact optimal duration of a tour visiting the candidate's stops in
  // its order, or kUnreachable if no such tour exists.
  int Evaluate(const Candidate& c) {
    seq_.clear();
    seq_.push_back(graph_.boundary.start);
    for (int g : c.order) {
      seq_.push_back(mid_groups_[g][c.chosen[g]]);
    }
    seq_.push_back(graph_.boundary.end);

    auto it = cache_.find(seq_);
    if (it != cache_.end()) {
      return it->second;
    }

    Cover cover;
    cover.flex = 0;  // At START, ready to leave at any time.
    for (size_t i = 0; i + 1 < seq_.size() && !cover.Empty(); ++i) {
      cover = ComposeLeg(cover, graph_.Pair(seq_[i], seq_[i + 1]));
    }
    int val = cover.MinDuration();
    cache_.emplace(seq_, val);
    ++evaluations_;
    return val;
  }

  long long NumEvaluations() const { return evaluations_; }

 private:
  const TwoOptGraph& graph_;
  const std::vector<std::vector<StopId>>& mid_groups_;
  std::vector<StopId> seq_;
  std::unordered_map<std::vector<StopId>, int, StopIdVectorHash> cache_;
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

    int cur_val = evaluator.Evaluate(c);

    // Best-improvement hill climbing over 2-opt segment reversals and
    // group-member swaps.
    // TODO: The evaluator could simply DP over group-member selection so that
    // we don't have to evaluate swap moves.
    bool improved = true;
    while (improved && !deadline_passed()) {
      improved = false;
      Candidate best_move;
      int best_move_val = cur_val;

      for (int i = 0; i < k; ++i) {
        for (int j = i + 1; j < k; ++j) {
          Candidate cand = c;
          std::reverse(cand.order.begin() + i, cand.order.begin() + j + 1);
          int val = evaluator.Evaluate(cand);
          if (val < best_move_val) {
            best_move_val = val;
            best_move = std::move(cand);
          }
        }
      }
      for (int p = 0; p < k; ++p) {
        int g = c.order[p];
        for (size_t m = 0; m < mid_groups[g].size(); ++m) {
          if (static_cast<int>(m) == c.chosen[g]) {
            continue;
          }
          Candidate cand = c;
          cand.chosen[g] = static_cast<int>(m);
          int val = evaluator.Evaluate(cand);
          if (val < best_move_val) {
            best_move_val = val;
            best_move = std::move(cand);
          }
        }
      }

      if (best_move_val < cur_val) {
        c = std::move(best_move);
        cur_val = best_move_val;
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
