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

  // Per required group representative, the group's stops.
  std::unordered_map<StopId, std::vector<StopId>> group_by_rep;

  const PairSteps& Pair(StopId a, StopId b) const {
    return pairs[static_cast<size_t>(a.v) * n_stops + b.v];
  }

  // The stops in the same required group as `stop` (including itself).
  const std::vector<StopId>& GroupOf(StopId stop) const {
    return group_by_rep.at(required.Representative(stop));
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
  for (std::vector<StopId>& group : graph.required.Groups()) {
    StopId rep = graph.required.Representative(group[0]);
    graph.group_by_rep[rep] = std::move(group);
  }
  return graph;
}

// The search state: the tour's stops in visit order, from START to END,
// with its value and the tables that score its neighbors.
class Candidate {
 public:
  Candidate(const TwoOptGraph& graph, std::vector<StopId> path)
      : path_(std::move(path)) {
    assert(path_.front() == graph.boundary.start);
    assert(path_.back() == graph.boundary.end);
    int n = static_cast<int>(path_.size());
    prefix_.resize(n);
    prefix_[0].flex_seconds = 0;  // Identity: ready to leave START any time.
    suffix_.resize(n);
    suffix_[n - 1].flex_seconds = 0;
    Recompute(graph, 1, n - 2);
  }

  const std::vector<StopId>& path() const { return path_; }

  // The exact optimal duration of a tour visiting the candidate's stops in
  // its order, or kUnreachable if there is none.
  int value() const { return value_; }

  // Reverses path_[i..j].
  void Reverse(const TwoOptGraph& graph, int i, int j) {
    std::reverse(path_.begin() + i, path_.begin() + j + 1);
    Recompute(graph, i, j);
  }

  // Visits `stop` at path index p instead.
  void SetStop(const TwoOptGraph& graph, int p, StopId stop) {
    path_[p] = stop;
    Recompute(graph, p, p);
  }

  struct Reversal {
    int i;      // -1 if no reversal ending at j is feasible.
    int value;  // kUnreachable in that case.
  };

  // The best of the reversals of path_[i..j] over all 0 < i < j, with its
  // value.
  Reversal BestReversalEndingAt(const TwoOptGraph& graph, int j) const {
    Reversal best{.i = -1, .value = kUnreachable};
    PairSteps reversed;
    reversed.flex_seconds = 0;  // Identity: the segment is just path_[j].
    PairSteps scratch_a;
    PairSteps scratch_b;
    for (int i = j - 1; i >= 1; --i) {
      Compose(reversed, graph.Pair(path_[i + 1], path_[i]), scratch_a);
      std::swap(reversed, scratch_a);
      if (reversed.Empty()) {
        break;
      }
      const PairSteps* sections[] = {
          &graph.Pair(path_[i - 1], path_[j]),
          &reversed,
          &graph.Pair(path_[i], path_[j + 1]),
      };
      int val = ValueOf(i - 1, sections, j + 1, scratch_a, scratch_b);
      if (val < best.value) {
        best = Reversal{.i = i, .value = val};
      }
    }
    return best;
  }

  struct Switch {
    StopId stop;  // path_[p] itself if no switch at p is feasible.
    int value;    // kUnreachable in that case.
  };

  // The best of the switches of path_[p] to another member of its required
  // group, with its value.
  Switch BestSwitchAt(const TwoOptGraph& graph, int p) const {
    Switch best{.stop = path_[p], .value = kUnreachable};
    PairSteps scratch_a;
    PairSteps scratch_b;
    for (StopId stop : graph.GroupOf(path_[p])) {
      if (stop == path_[p]) {
        continue;
      }
      const PairSteps* sections[] = {
          &graph.Pair(path_[p - 1], stop),
          &graph.Pair(stop, path_[p + 1]),
      };
      int val = ValueOf(p - 1, sections, p + 1, scratch_a, scratch_b);
      if (val < best.value) {
        best = Switch{.stop = stop, .value = val};
      }
    }
    return best;
  }

 private:
  // Recomputes the tables and the value after path_[first..last] changed:
  // prefix_[b], the section from path_[0] to path_[b], for b >= first, and
  // suffix_[a], the section from path_[a] to path_[n - 1], for a <= last.
  void Recompute(const TwoOptGraph& graph, int first, int last) {
    int n = static_cast<int>(path_.size());
    for (int b = first; b < n; ++b) {
      if (prefix_[b - 1].Empty()) {
        prefix_[b].Clear();
      } else {
        Compose(prefix_[b - 1], graph.Pair(path_[b - 1], path_[b]), prefix_[b]);
      }
    }
    for (int a = last; a >= 0; --a) {
      if (suffix_[a + 1].Empty()) {
        suffix_[a].Clear();
      } else {
        Compose(graph.Pair(path_[a], path_[a + 1]), suffix_[a + 1], suffix_[a]);
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

  std::vector<StopId> path_;  // The tour's stops, START to END.
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
  std::vector<std::vector<StopId>> mid_groups;
  for (std::vector<StopId>& group : graph.required.Groups()) {
    StopId rep = graph.required.Representative(group[0]);
    if (rep != start_rep && rep != end_rep) {
      mid_groups.push_back(std::move(group));
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

  // A unit of the neighborhood scan: the reversals ending at path index j if
  // j >= 0, otherwise the switches of the stop at path index p to the other
  // members of its group.
  struct Unit {
    int j = -1;
    int p = -1;
  };
  std::vector<Unit> units;
  for (int j = 2; j <= k; ++j) {
    units.push_back(Unit{.j = j});
  }
  for (int p = 1; p <= k; ++p) {
    units.push_back(Unit{.p = p});
  }

  std::optional<Candidate> best_candidate;
  std::mt19937 rng(options.seed);
  long long evaluations = 0;

  for (int restart = 0; restart < options.restarts; ++restart) {
    if (deadline_passed()) {
      break;
    }

    auto restart_start = std::chrono::steady_clock::now();

    // Build a random initial candidate: a random member of each middle group
    // in a random order.
    std::vector<StopId> path;
    path.reserve(k + 2);
    path.push_back(graph.boundary.start);
    for (const std::vector<StopId>& group : mid_groups) {
      path.push_back(
          group[std::uniform_int_distribution<size_t>(0, group.size() - 1)(rng)]
      );
    }
    std::shuffle(path.begin() + 1, path.end(), rng);
    path.push_back(graph.boundary.end);
    Candidate c(graph, std::move(path));
    ++evaluations;

    // First-improvement hill climbing over 2-opt segment reversals and
    // group-member switches: each step scans the neighborhood in a fresh
    // random order of its units (all reversals ending at a given index, or
    // all switches at a given index) and moves to the first improving
    // candidate found.
    // TODO: Candidate could simply DP over group-member selection so that we
    // don't have to evaluate switch moves.
    while (!deadline_passed()) {
      std::shuffle(units.begin(), units.end(), rng);
      // The improving move found: a reversal of path_[move_i..move_j] if
      // move_j >= 0, otherwise a switch of path_[move_p] to move_stop.
      int move_val = c.value();
      int move_i = -1;
      int move_j = -1;
      int move_p = -1;
      StopId move_stop{};
      for (const Unit& unit : units) {
        if (unit.j >= 0) {
          Candidate::Reversal reversal = c.BestReversalEndingAt(graph, unit.j);
          evaluations += unit.j - 1;
          if (reversal.value < c.value()) {
            move_val = reversal.value;
            move_i = reversal.i;
            move_j = unit.j;
          }
        } else {
          Candidate::Switch sw = c.BestSwitchAt(graph, unit.p);
          evaluations += graph.GroupOf(c.path()[unit.p]).size() - 1;
          if (sw.value < c.value()) {
            move_val = sw.value;
            move_p = unit.p;
            move_stop = sw.stop;
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
        c.SetStop(graph, move_p, move_stop);
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

  for (StopId stop : best_candidate->path()) {
    result.best_tour.push_back(graph.compact.mapping.new_to_original[stop.v]);
  }
  return result;
}

}  // namespace vats5
