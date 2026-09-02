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

constexpr int kUnreachable = kPairStepsUnreachable;

// A tour cannot depart before 00:00:00. Chains that do are carried through
// the covers (they never dominate one that does not) and excluded only when
// a tour is scored, as ComputeMinimalFeasiblePathsAlong does.
constexpr int kEarliestDeparture = 0;

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

using MidGroups = std::vector<std::vector<StopId>>;

// The search state: which stop of each middle group is visited and in which
// order the groups are visited, with its value and the tables that score its
// neighbors. A neighbor differs from it in one section of the tour (a
// reversed segment, or one replaced stop), so a neighbor's value is composed
// from the precomputed covers of the prefix before the change and the suffix
// after it; only the changed section is composed leg by leg.
class Candidate {
 public:
  // `order` is a permutation of the middle group indices; `chosen[g]` is the
  // index of the visited stop in mid_groups[g].
  Candidate(
      const TwoOptGraph& graph,
      const MidGroups& mid_groups,
      std::vector<int> order,
      std::vector<int> chosen
  )
      : order_(std::move(order)), chosen_(std::move(chosen)) {
    int k = static_cast<int>(order_.size());
    position_of_.resize(k);
    for (int p = 0; p < k; ++p) {
      position_of_[order_[p]] = p;
    }

    seq_.reserve(k + 2);
    seq_.push_back(graph.boundary.start);
    for (int g : order_) {
      seq_.push_back(mid_groups[g][chosen_[g]]);
    }
    seq_.push_back(graph.boundary.end);
    int n = static_cast<int>(seq_.size());

    prefix_.resize(n);
    prefix_[0].flex_seconds = 0;  // Identity: ready to leave START any time.
    suffix_.resize(n);
    suffix_[n - 1].flex_seconds = 0;
    Recompute(graph, 1, n - 2);
  }

  const std::vector<int>& order() const { return order_; }
  const std::vector<int>& chosen() const { return chosen_; }
  int PositionOf(int g) const { return position_of_[g]; }

  // The exact optimal duration of a tour visiting the candidate's stops in
  // its order, or kUnreachable if there is none.
  int value() const { return value_; }

  // Reverses order positions i..j.
  void Reverse(const TwoOptGraph& graph, int i, int j) {
    std::reverse(order_.begin() + i, order_.begin() + j + 1);
    for (int p = i; p <= j; ++p) {
      position_of_[order_[p]] = p;
    }
    std::reverse(seq_.begin() + i + 1, seq_.begin() + j + 2);
    Recompute(graph, i + 1, j + 1);
  }

  // Visits member m of group g instead.
  void SetMember(
      const TwoOptGraph& graph, const MidGroups& mid_groups, int g, int m
  ) {
    chosen_[g] = m;
    int s = position_of_[g] + 1;
    seq_[s] = mid_groups[g][m];
    Recompute(graph, s, s);
  }

  struct Reversal {
    int i;      // -1 if no reversal ending at j is feasible.
    int value;  // kUnreachable in that case.
  };

  // The best of the reversals of order positions i..j over all i < j, with
  // its value. The cover of the reversed segment (from seq_[j] back to
  // seq_[i]) is extended by one leg at its end as i walks down, so each
  // reversal costs a constant number of compositions. (Extending at the end,
  // rather than the start, keeps the composition in its cheap orientation: a
  // small cover followed by a leg with many departures.)
  Reversal BestReversalEndingAt(const TwoOptGraph& graph, int j) const {
    Reversal best{.i = -1, .value = kUnreachable};
    int sj = j + 1;  // seq_ index of order position j.
    PairSteps reversed;
    reversed.flex_seconds = 0;  // Identity: the segment is just seq_[sj].
    PairSteps scratch_a;
    PairSteps scratch_b;
    for (int i = j - 1; i >= 0; --i) {
      int si = i + 1;
      Compose(reversed, graph.Pair(seq_[si + 1], seq_[si]), scratch_a);
      std::swap(reversed, scratch_a);
      if (reversed.Empty()) {
        break;
      }
      const PairSteps* sections[] = {
          &graph.Pair(seq_[si - 1], seq_[sj]),
          &reversed,
          &graph.Pair(seq_[si], seq_[sj + 1]),
      };
      int val = ValueOf(si - 1, sections, sj + 1, scratch_a, scratch_b);
      if (val < best.value) {
        best = Reversal{.i = i, .value = val};
      }
    }
    return best;
  }

  // The value of the candidate visiting member m of group g instead.
  int MemberValue(
      const TwoOptGraph& graph, const MidGroups& mid_groups, int g, int m
  ) const {
    int s = position_of_[g] + 1;  // seq_ index of the group's stop.
    StopId x = mid_groups[g][m];
    const PairSteps* sections[] = {
        &graph.Pair(seq_[s - 1], x),
        &graph.Pair(x, seq_[s + 1]),
    };
    PairSteps scratch_a;
    PairSteps scratch_b;
    return ValueOf(s - 1, sections, s + 1, scratch_a, scratch_b);
  }

 private:
  // Recomputes the tables and the value after seq_[first..last] changed:
  // prefix_[b], the section from seq_[0] to seq_[b], for b >= first, and
  // suffix_[a], the section from seq_[a] to seq_[n - 1], for a <= last.
  void Recompute(const TwoOptGraph& graph, int first, int last) {
    int n = static_cast<int>(seq_.size());
    for (int b = first; b < n; ++b) {
      if (prefix_[b - 1].Empty()) {
        prefix_[b].Clear();
      } else {
        Compose(prefix_[b - 1], graph.Pair(seq_[b - 1], seq_[b]), prefix_[b]);
      }
    }
    for (int a = last; a >= 0; --a) {
      if (suffix_[a + 1].Empty()) {
        suffix_[a].Clear();
      } else {
        Compose(graph.Pair(seq_[a], seq_[a + 1]), suffix_[a + 1], suffix_[a]);
      }
    }
    value_ = prefix_[n - 1].MinDuration(kEarliestDeparture);
  }

  // The value of the tour made of prefix_[p], then `sections` in order, then
  // suffix_[s].
  template <size_t N>
  int ValueOf(
      int p,
      const PairSteps* (&sections)[N],
      int s,
      PairSteps& scratch_a,
      PairSteps& scratch_b
  ) const {
    const PairSteps* cur = &prefix_[p];
    for (const PairSteps* section : sections) {
      Compose(*cur, *section, scratch_a);
      if (scratch_a.Empty()) {
        return kUnreachable;
      }
      std::swap(scratch_a, scratch_b);
      cur = &scratch_b;
    }
    Compose(*cur, suffix_[s], scratch_a);
    return scratch_a.MinDuration(kEarliestDeparture);
  }

  std::vector<int> order_;        // Permutation of middle group indices.
  std::vector<int> chosen_;       // Per middle group, index into its stops.
  std::vector<int> position_of_;  // Per middle group, its position in order_.
  std::vector<StopId> seq_;       // The tour's stops, START to END.
  std::vector<PairSteps> prefix_;
  std::vector<PairSteps> suffix_;
  int value_;
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
  MidGroups mid_groups;
  for (const std::vector<StopId>& group : graph.required.Groups()) {
    StopId rep = graph.required.Representative(group[0]);
    if (rep != start_rep && rep != end_rep) {
      mid_groups.push_back(group);
    }
  }
  int k = static_cast<int>(mid_groups.size());

  auto deadline_passed = [&, start_time = std::chrono::steady_clock::now()]() {
    if (!options.time_limit_seconds.has_value()) {
      return false;
    }
    std::chrono::duration<double> elapsed =
        std::chrono::steady_clock::now() - start_time;
    return elapsed.count() >= *options.time_limit_seconds;
  };

  // A unit of the neighborhood scan: the reversals ending at order position
  // j if j >= 0, otherwise the swap of group g's stop to its member m.
  struct Unit {
    int j = -1;
    int g = -1;
    int m = -1;
  };
  std::vector<Unit> units;
  for (int j = 1; j < k; ++j) {
    units.push_back(Unit{.j = j});
  }
  for (int g = 0; g < k; ++g) {
    for (size_t m = 0; m < mid_groups[g].size(); ++m) {
      units.push_back(Unit{.g = g, .m = static_cast<int>(m)});
    }
  }

  std::optional<Candidate> best_candidate;
  std::mt19937 rng(options.seed);
  long long evaluations = 0;

  for (int restart = 0; restart < options.restarts; ++restart) {
    if (deadline_passed()) {
      break;
    }

    auto restart_start = std::chrono::steady_clock::now();

    // Build a random initial candidate.
    std::vector<int> order(k);
    for (int g = 0; g < k; ++g) {
      order[g] = g;
    }
    std::shuffle(order.begin(), order.end(), rng);
    std::vector<int> chosen(k, 0);
    for (int g = 0; g < k; ++g) {
      if (mid_groups[g].size() > 1) {
        chosen[g] = std::uniform_int_distribution<
            int>(0, static_cast<int>(mid_groups[g].size()) - 1)(rng);
      }
    }
    Candidate c(graph, mid_groups, std::move(order), std::move(chosen));
    ++evaluations;

    // First-improvement hill climbing over 2-opt segment reversals and
    // group-member swaps: each step scans the neighborhood in a fresh random
    // order of its units (all reversals ending at a given position, or one
    // group-member swap) and moves to the first improving candidate found.
    // TODO: Candidate could simply DP over group-member selection so that we
    // don't have to evaluate swap moves.
    while (!deadline_passed()) {
      std::shuffle(units.begin(), units.end(), rng);
      // The improving move found: a reversal of order positions i..j if
      // move_j >= 0, otherwise a swap of group move_g's stop to member move_m.
      int move_val = c.value();
      int move_i = -1;
      int move_j = -1;
      int move_g = -1;
      int move_m = -1;
      for (const Unit& unit : units) {
        if (unit.j >= 0) {
          Candidate::Reversal reversal = c.BestReversalEndingAt(graph, unit.j);
          evaluations += unit.j;
          if (reversal.value < c.value()) {
            move_val = reversal.value;
            move_i = reversal.i;
            move_j = unit.j;
          }
        } else {
          if (unit.m == c.chosen()[unit.g]) {
            continue;
          }
          int val = c.MemberValue(graph, mid_groups, unit.g, unit.m);
          ++evaluations;
          if (val < c.value()) {
            move_val = val;
            move_g = unit.g;
            move_m = unit.m;
          }
        }
        if (move_val < c.value()) {
          break;
        }
      }
      if (move_val == c.value()) {
        break;  // Local optimum.
      }
      if (move_j >= 0) {
        c.Reverse(graph, move_i, move_j);
      } else {
        c.SetMember(graph, mid_groups, move_g, move_m);
      }
      assert(c.value() == move_val);
      ++evaluations;
    }

    ++result.restarts_completed;
    result.restart_seconds.push_back(
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - restart_start
        )
            .count()
    );
    int local_val = c.value();
    if (local_val < result.best_val) {
      result.best_val = local_val;
      best_candidate = c;
    }
    if (search_log != nullptr) {
      *search_log << "restart " << restart << ": local opt "
                  << (local_val == kUnreachable
                          ? "infeasible"
                          : TimeSinceServiceStart{local_val}.ToString())
                  << ", global best "
                  << (result.best_val == kUnreachable
                          ? "infeasible"
                          : TimeSinceServiceStart{result.best_val}.ToString())
                  << "\n";
    }
  }

  result.evaluations = evaluations;
  if (result.best_val == kUnreachable) {
    return result;
  }

  result.best_tour.push_back(
      graph.compact.mapping.new_to_original[graph.boundary.start.v]
  );
  for (int g : best_candidate->order()) {
    StopId stop = mid_groups[g][best_candidate->chosen()[g]];
    result.best_tour.push_back(graph.compact.mapping.new_to_original[stop.v]);
  }
  result.best_tour.push_back(
      graph.compact.mapping.new_to_original[graph.boundary.end.v]
  );
  return result;
}

}  // namespace vats5
