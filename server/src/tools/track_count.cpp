#include <CLI/CLI.hpp>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <numbers>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

StopId FindStopByGtfsId(
    const ProblemState& problem, const std::string& gtfs_stop_id_str
) {
  const GtfsStopId target_gtfs_id{gtfs_stop_id_str};
  for (const auto& [stop_id, info] : problem.stop_infos) {
    if (info.gtfs_stop_id == target_gtfs_id) {
      return stop_id;
    }
  }
  throw std::runtime_error(
      "Stop " + gtfs_stop_id_str + " not found in stop_infos"
  );
}

void ListStops(const ProblemState& problem) {
  for (const auto& [stop_id, info] : problem.stop_infos) {
    if (stop_id == problem.boundary.start || stop_id == problem.boundary.end) {
      continue;
    }
    std::cout << info.gtfs_stop_id.v << " - " << info.stop_name << "\n";
  }
}

void PrintDepartures(const ProblemState& problem, StopId stop) {
  struct Departure {
    TimeSinceServiceStart time;
    std::string dest_name;
  };
  std::vector<Departure> departures;

  for (const auto& group : problem.minimal.GetGroups(stop)) {
    const auto& dest_name =
        problem.stop_infos.at(group.destination_stop).stop_name;
    for (const auto& step : problem.minimal.GetSteps(group)) {
      departures.push_back({step.origin_time, dest_name});
    }
  }

  std::ranges::sort(departures, {}, &Departure::time);

  std::cout << "Departure times from " << problem.stop_infos.at(stop).stop_name
            << ":\n";
  for (size_t i = 0; i < departures.size();) {
    size_t j = i;
    while (j < departures.size() && departures[j].time == departures[i].time) {
      ++j;
    }
    std::cout << "  " << departures[i].time.ToString() << " -> ";
    for (size_t k = i; k < j; ++k) {
      if (k > i) std::cout << ", ";
      std::cout << departures[k].dest_name;
    }
    std::cout << "\n";
    i = j;
  }
}

struct StopExitStats {
  int min_exit_seconds;
  int max_exit_seconds;
};

struct ExitStats {
  std::unordered_map<StopId, StopExitStats> individual;
  std::vector<int> sorted_min_exit_seconds;
  std::vector<int> sorted_max_exit_seconds;
};

ExitStats ComputeExitStats(
    const StepsAdjacencyList& completed, const ProblemBoundary& boundary
) {
  ExitStats result;
  for (StopId s{0}; s.v < completed.NumStops(); ++s.v) {
    auto groups = completed.GetGroups(s);
    if (s == boundary.start || s == boundary.end || groups.size() == 0) {
      continue;
    }
    StopExitStats& stats = result.individual[s];
    stats.min_exit_seconds = std::numeric_limits<int>::max();
    stats.max_exit_seconds = 0;
    auto VisitStep = [&](const AdjacencyListStep& step) {
      int duration = step.destination_time.seconds - step.origin_time.seconds;
      stats.min_exit_seconds = std::min(stats.min_exit_seconds, duration);
      stats.max_exit_seconds = std::max(stats.max_exit_seconds, duration);
    };
    for (const StepGroup& g : groups) {
      if (g.destination_stop == boundary.start ||
          g.destination_stop == boundary.end) {
        continue;
      }
      if (g.flex_step.has_value()) {
        VisitStep(*g.flex_step);
      }
      for (const AdjacencyListStep& step : completed.GetSteps(g)) {
        VisitStep(step);
      }
    }
    result.sorted_min_exit_seconds.push_back(stats.min_exit_seconds);
    result.sorted_max_exit_seconds.push_back(stats.max_exit_seconds);
  }

  std::ranges::sort(result.sorted_min_exit_seconds);
  std::ranges::sort(result.sorted_max_exit_seconds, std::greater{});

  for (int i = 0; i < result.sorted_min_exit_seconds.size(); ++i) {
    std::cout << TimeSinceServiceStart{result.sorted_min_exit_seconds[i]}
              << "  "
              << TimeSinceServiceStart{result.sorted_max_exit_seconds[i]}
              << "\n";
  }

  return result;
}

struct OnwardsStep {
  StopId destination;
  int duration;
};

struct ArrivalTimeState {
  TimeSinceServiceStart arrival_time;
  std::vector<OnwardsStep> onwards;
};

struct StepState {
  std::vector<ArrivalTimeState> states;

  void SortAndDedupe() {
    std::sort(states.begin(), states.end(), [](const auto& a, const auto& b) {
      return a.arrival_time < b.arrival_time;
    });
    auto it = std::unique(
        states.begin(), states.end(), [](const auto& a, const auto& b) {
          return a.arrival_time == b.arrival_time;
        }
    );
    states.erase(it, states.end());
  }
};

struct BnbState {
  std::vector<std::unordered_map<StopId, StepState>> step_states;
  ProblemBoundary boundary;
};

TimeSinceServiceStart GetTNext(
    const StepsAdjacencyList& completed,
    const StepGroup& g_next,
    TimeSinceServiceStart t_cur
) {
  TimeSinceServiceStart t_next_flex{std::numeric_limits<int>::max()};
  if (g_next.flex_step.has_value()) {
    t_next_flex.seconds =
        t_cur.seconds + g_next.flex_step->FlexDurationSeconds();
  }

  TimeSinceServiceStart t_next_sched{std::numeric_limits<int>::max()};
  std::span<const AdjacencyListStep> group_steps = completed.GetSteps(g_next);
  size_t t_next_i = FindDepartureAtOrAfter(completed, g_next, t_cur);
  if (t_next_i < group_steps.size()) {
    t_next_sched = group_steps[t_next_i].destination_time;
  }

  return std::min(t_next_flex, t_next_sched);
}

// Propagates `state.step_states[k]` into `state.step_states[k + 1]`.
// Also adds `.onwards` entries for `state.step_states[k]`.
// Assumes these things are all empty to start with.
void PropagateStepStatesForwards(
    const StepsAdjacencyList& completed,
    const TimeSinceServiceStart t_ub,
    BnbState& state,
    StopId s0,
    int k
) {
  assert(k >= 0);
  assert(k < state.step_states.size());

  for (auto& [s_cur, step_state_cur] : state.step_states[k]) {
    for (const StepGroup& g_next : completed.GetGroups(s_cur)) {
      StopId s_next = g_next.destination_stop;

      // We've got a few "onwards" constraints:
      // 0->1 is START->s0
      // Don't step to s0 in any intermediate step.
      // Don't step to END in any other.
      // (n-2)->(n-1) is *->END.
      if ((k == 0 && s_next != s0) || (k != 0 && s_next == s0) ||
          (k < state.step_states.size() - 2 && s_next == state.boundary.end) ||
          (k == state.step_states.size() - 2 && s_next != state.boundary.end)) {
        continue;
      }

      for (ArrivalTimeState& ats : step_state_cur.states) {
        TimeSinceServiceStart t_next =
            GetTNext(completed, g_next, ats.arrival_time);
        if (t_next > t_ub) {
          continue;
        }
        ats.onwards.push_back(
            OnwardsStep{
                .destination = s_next,
                .duration = t_next.seconds - ats.arrival_time.seconds,
            }
        );
        if (k + 1 < state.step_states.size()) {
          state.step_states[k + 1][s_next].states.push_back(
              {.arrival_time = t_next}
          );
        }
      }
    }
  }

  if (k + 1 < state.step_states.size()) {
    for (auto& [_, step_state] : state.step_states[k + 1]) {
      step_state.SortAndDedupe();
    }
  }
}

// Deletes states from `state.step_states[k + 1]` that do not come from
// anything in `state.step_states[k]`.
void FilterStepStatesForwards(BnbState& state, int k) {
  assert(k >= 0);
  assert(k + 1 < state.step_states.size());

  // arrival_times[s] is all the arrival times to s that come from something in
  // state.step_states[k].
  std::unordered_map<StopId, std::unordered_set<TimeSinceServiceStart>>
      arrival_times;
  for (const auto& [s_cur, step_state_cur] : state.step_states[k]) {
    for (const ArrivalTimeState& ats : step_state_cur.states) {
      for (const OnwardsStep& onwards : ats.onwards) {
        arrival_times[onwards.destination].insert(
            TimeSinceServiceStart{ats.arrival_time.seconds + onwards.duration}
        );
      }
    }
  }

  for (auto& [s_next, step_state_next] : state.step_states[k + 1]) {
    const std::unordered_set<TimeSinceServiceStart>& arrival_times_next =
        arrival_times[s_next];
    std::erase_if(step_state_next.states, [&](const ArrivalTimeState& ats) {
      return !arrival_times_next.contains(ats.arrival_time);
    });
  }

  std::erase_if(state.step_states[k + 1], [](const auto& pair) {
    return pair.second.states.size() == 0;
  });
}

// Deletes states from `state.step_states[k]` that do not lead to anything in
// `state.step_states[k + 1]`.
void FilterStepStatesBackwards(BnbState& state, int k) {
  assert(k >= 0);
  assert(k + 1 < state.step_states.size());

  // arrival_times[s] is all the arrival times to s in state.step_states[k + 1].
  std::unordered_map<StopId, std::unordered_set<TimeSinceServiceStart>>
      arrival_times;
  for (const auto& [s_next, step_state_next] : state.step_states[k + 1]) {
    for (const ArrivalTimeState& ats : step_state_next.states) {
      arrival_times[s_next].insert(ats.arrival_time);
    }
  }

  for (auto& [s_cur, step_state_cur] : state.step_states[k]) {
    for (ArrivalTimeState& ats : step_state_cur.states) {
      std::erase_if(ats.onwards, [&](const OnwardsStep& onwards) {
        return !arrival_times[onwards.destination].contains(
            TimeSinceServiceStart{ats.arrival_time.seconds + onwards.duration}
        );
      });
    }
    std::erase_if(step_state_cur.states, [](const ArrivalTimeState& ats) {
      return ats.onwards.size() == 0;
    });
  }

  std::erase_if(state.step_states[k], [](const auto& pair) {
    return pair.second.states.size() == 0;
  });
}

// TODO(Sunday): Proceed with implementing this.
//
// Then start messing with branching strategies
// - require stops at start and at end or near start near end
// - constrain time ranges at start and at end or near start near end
//
// my guess is that there are huge improvements available constraining things
// near the end cuz we have such a large range of possible times around the end
// if we constrain to subsets of the times:
// - low times can't actually be reached so these branches get quickly pruned
// (but how do they get pruned? need to make sure they get pruned well!)
// - mid times are quite realistic and require a lot of exploration, but having
// a small time range around the end makes the LB tighter around the end
// - high times at the end increase the LB by virtue of being high
//
// then also there are improvements available by constraining stops near the
// start and end, because for some reason it is "obvious" which ones these
// should be
//
// but are these enough? We will see, but I am guessing that no it will not be
// enough because as we start working towards the middle we don't get enough
// tightening to avoid basically looking at "all reasonable tours" and there are
// a lot of reasonable tours.
//
// i have this other idea of tightening the step time ranges using the tsp
// solver to answer "what is the least possible amount of time you can spend
// making k steps"
//
// if we can answer this then we can tighten the min time at each step forwards
// from the beginning and we can also tighten the max time at each step
// backwards from the end.
//
// can we answer this?
// well, up to a smallish k (<10ish), yes: build up a graph like we normally do
// up to k, and then have a final partition where the weights between everything
// is 0.
// also for a k near the end (within 10ish), we can build up a normal smeared
// graph for the first j<k steps, then switch to a countdown and have the
// weights become 0 somewhere in the countdown
//
// hmmmm, not convinced this will give us much improvement on existing bounds
// because with small and with large k you can really save a lot of time by
// ending yourself up in an unreasonable spot
//
// so perhaps we are gonna need to rely on near-start/near-end constraints
// constraining things enough that we can make progress
//
// oh also I have the "min error DP" idea that might actually tighten the LB in
// this context. so I should try that.
// this might even make it possible to have fruitful branches in the smear
// region:
// - e.g. if there's a stop that always has a good txfer time, constrain it to a
// particular range of steps so that the "min error DP" can't use it over and
// over
// - or if there's a time range when all txfers are good, constrain that you
// only spend M steps in that time range so the "min error DP" can't
// unrealistically spend all its time in that range

void CombineForcedSteps(BnbState& state) {
  std::vector<std::unordered_map<StopId, StepState>> result_states;
  result_states.reserve(state.step_states.size());

  // Sweep forwards pushing states into `result_states`, combining with prev
  // when it is a forced stop state.
  for (int i = 0; i < state.step_states.size(); ++i) {
    if (
      // First state can't be combined with prev because there is no prev.
      i == 0 ||
      // Don't want to combine last state with prev so that we always have a
      // 1-state END at the end.
      i + 1 == state.step_states.size() ||
      // Don't combine >1 size states.
      state.step_states[i].size() != 1
    ) {
      result_states.push_back(std::move(state.step_states[i]));
      continue;
    }

    // Ok we have a size==1 state and it's not at the beginning or the end.
    const auto& [forced_s, forced_state] = *state.step_states[i].begin();

    // So we want to modify the prev state to represent arriving at forced_s.
    for (auto& [prev_s, prev_state] : result_states.back()) {
      int forced_ats_i = 0;
      for (ArrivalTimeState& prev_ats : prev_state.states) {
        assert(prev_ats.onwards.size() == 1);
        OnwardsStep prev_onwards = prev_ats.onwards.front();
        assert(prev_onwards.destination == forced_s);
        prev_ats.onwards.clear();

        while (forced_ats_i < forced_state.states.size() &&
               forced_state.states[forced_ats_i].arrival_time.seconds <
                   prev_ats.arrival_time.seconds + prev_onwards.duration) {
          forced_ats_i += 1;
        }
        assert(forced_ats_i < forced_state.states.size());
        assert(
            forced_state.states[forced_ats_i].arrival_time.seconds ==
            prev_ats.arrival_time.seconds + prev_onwards.duration
        );

        prev_ats.onwards.reserve(
            forced_state.states[forced_ats_i].onwards.size()
        );
        for (const OnwardsStep& forced_onwards :
             forced_state.states[forced_ats_i].onwards) {
          prev_ats.onwards.push_back(
              OnwardsStep{
                  .destination = forced_onwards.destination,
                  .duration = prev_onwards.duration + forced_onwards.duration,
              }
          );
        }
      }
    }
  }

  state.step_states = std::move(result_states);
}

// Sweeps a forwards filter from k0 to the end and then a backwards filter to
// the start.
void FilterStepStatesSweep(BnbState& state, int k0) {
  for (int k = k0; k + 1 < state.step_states.size(); ++k) {
    FilterStepStatesForwards(state, k);
  }
  for (int k = state.step_states.size() - 2; k >= 0; --k) {
    FilterStepStatesBackwards(state, k);
  }

  std::unordered_set<StopId> known_step_stops;
  for (int i = 0; i < state.step_states.size(); ++i) {
    if (state.step_states[i].size() == 1) {
      known_step_stops.insert(state.step_states[i].begin()->first);
    }
  }
  bool changed_anything = false;
  for (int i = 0; i < state.step_states.size(); ++i) {
    if (state.step_states[i].size() == 1) {
      continue;
    }
    auto erased_n = std::erase_if(state.step_states[i], [&](const auto& pair) {
      return known_step_stops.contains(pair.first);
    });
    changed_anything |= erased_n > 0;
  }

  // TODO: Loop until everything stops changing?
  if (changed_anything) {
    for (int k = 0; k + 1 < state.step_states.size(); ++k) {
      FilterStepStatesForwards(state, k);
    }
    for (int k = state.step_states.size() - 2; k >= 0; --k) {
      FilterStepStatesBackwards(state, k);
    }
  }

  // TODO: We can also find known step stops by noticing if there are any stops
  // that appear in only one step.
}

BnbState ComputeStepStates(
    const StepsAdjacencyList& completed,
    const RequiredStops& required,
    const ProblemBoundary& boundary,
    StopId s0,
    TimeSinceServiceStart t0,
    int lb_rel,
    int ub_rel
) {
  assert(required.size() >= 4);

  // TODO: Consider whether we can tighten these up enough to be useful.
  // ExitStats exit_stats = ComputeExitStats(graph, problem.boundary);
  // // We use all the exits except for 1 to do all the steps.
  // assert(exit_stats.sorted_max_exit_seconds.size() - 1 ==
  // problem.required.size() - 3);
  // assert(exit_stats.sorted_max_exit_seconds.size() ==
  // exit_stats.sorted_min_exit_seconds.size());

  TimeSinceServiceStart t_lb = t0;
  if (lb_rel >= 0) {
    t_lb.seconds = t0.seconds + lb_rel;
  }

  TimeSinceServiceStart t_ub;
  if (ub_rel < 0) {
    for (const Step& step : completed.AllSteps()) {
      t_ub = std::max(t_ub, step.destination.time);
    }
  } else {
    t_ub.seconds = t0.seconds + ub_rel;
  }

  BnbState state;
  state.boundary = boundary;

  // state.step_states[0]: Base case, START@t0.
  //
  // state.step_states[1]: s0@t0.
  //
  // state.step_states[k]: All possible states after making k steps (visited k+1
  // stops counting boundary stops).
  //
  // state.step_states[n-1]: END@{tf1, tf2, ..., tfx}.
  state.step_states.resize(required.size());

  state.step_states[0][boundary.start].states.push_back({.arrival_time = t0});
  for (int k = 0; k < state.step_states.size(); ++k) {
    PropagateStepStatesForwards(completed, t_ub, state, s0, k);
  }

  // Anything arriving at the end before t_lb is too good to be true.
  for (auto& [s, step_state] : state.step_states.back()) {
    std::erase_if(step_state.states, [&](const ArrivalTimeState& ats) {
      return ats.arrival_time < t_lb;
    });
  }

  // Now go backwardsly and filter any arrival times that do not lead to actual
  // arrival times that we have.
  for (int k = static_cast<int>(state.step_states.size()) - 2; k >= 0; --k) {
    FilterStepStatesBackwards(state, k);
  }

  // for (int k = 0; k < state.step_states.size(); ++k) {
  //   const std::unordered_map<StopId, StepState>& step_state_k =
  //   state.step_states[k]; int max_smear = 0; TimeSinceServiceStart
  //   min_t{std::numeric_limits<int>::max()}, max_t{0}; for (const auto& [s,
  //   step_state] : step_state_k) {
  //     if (step_state.arrival_times.size() == 0) {
  //       continue;
  //     }
  //     int smear = step_state.arrival_times.back().seconds -
  //                 step_state.arrival_times[0].seconds;
  //     max_smear = std::max(max_smear, smear);
  //     min_t = std::min(min_t, step_state.arrival_times[0]);
  //     max_t = std::max(max_t, step_state.arrival_times.back());
  //   }
  //   std::cout << "k = " << k << "\n"
  //             << "  [" << min_t << ", " << max_t << "] (difference "
  //             << TimeSinceServiceStart{max_t.seconds - min_t.seconds} <<
  //             ")\n"
  //             << "  max smear " << TimeSinceServiceStart{max_smear} << "\n";
  // }

  return state;
}

struct TarelStatePairHash {
  size_t operator()(const std::pair<TarelState, TarelState>& p) const {
    size_t h1 = std::hash<TarelState>{}(p.first);
    size_t h2 = std::hash<TarelState>{}(p.second);
    return h1 ^
           (h2 * 0x9e3779b97f4a7c15ULL + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
  }
};

void DeduplicateEdges(std::vector<TarelEdge>& edges) {
  std::unordered_map<std::pair<TarelState, TarelState>, int, TarelStatePairHash>
      best;
  for (const TarelEdge& e : edges) {
    auto [it, inserted] =
        best.try_emplace(std::make_pair(e.origin, e.destination), e.weight);
    if (!inserted) {
      it->second = std::min(it->second, e.weight);
    }
  }
  edges.clear();
  for (const auto& [key, weight] : best) {
    edges.push_back(
        {.origin = key.first, .destination = key.second, .weight = weight}
    );
  }
}

constexpr int kMaxStep = 6;

std::optional<TspTourResult> DoTSP(
    const StepsAdjacencyList& completed,
    const RequiredStops& required,
    const BnbState& state,
    int ub_rel
) {
  const auto& boundary = state.boundary;
  const auto& step_states = state.step_states;

  for (const std::unordered_map<StopId, StepState>& step_state : step_states) {
    // TODO: Think about whether this is an expected condition and whether we
    // can/should detect it earlier.
    if (step_state.size() == 0) {
      std::cout << "  empty step_state\n";
      return std::nullopt;
    }
  }

  int n = step_states.size();

  auto ClampedPartition = [&](int step) -> StepPartitionId {
    if (step <= kMaxStep || step >= n - kMaxStep) {
      return StepPartitionId{step};
    } else {
      return StepPartitionId{kMaxStep};
    }
  };

  // Convention:
  // step_states[k] does a step from StepPartitionId{k} -> StepPartitionId{k +
  // 1}.
  //
  // Since step_states[0] is a step from START, the start_state is thusly in
  // StepPartitionId{0}. Since step_states[n-1] is a step to END, the end_state
  // is thusly in StepPartitionId{n}.
  TarelState start_state{boundary.start, ClampedPartition(0)};
  TarelState end_state{boundary.end, ClampedPartition(n - 1)};
  std::vector<TarelEdge> edges;

  // END->START edge.
  edges.push_back({
      .origin = end_state,
      .destination = start_state,
      .weight = 0,
  });

  // Insert all the edges!! How edgy.
  for (int k = 0; k + 1 < n; ++k) {
    for (const auto& [s_cur, step_state_cur] : step_states[k]) {
      // Collect best duration per destination across all arrival times.
      std::unordered_map<StopId, int> best_dur_by_dest;
      for (const ArrivalTimeState& ats : step_state_cur.states) {
        for (const OnwardsStep& onwards : ats.onwards) {
          auto [it, inserted] = best_dur_by_dest.try_emplace(
              onwards.destination, onwards.duration
          );
          if (!inserted) {
            it->second = std::min(it->second, onwards.duration);
          }
        }
      }

      for (const auto& [s_next, best_dur] : best_dur_by_dest) {
        edges.push_back({
            .origin = TarelState{s_cur, ClampedPartition(k)},
            .destination = TarelState{s_next, ClampedPartition(k + 1)},
            .weight = best_dur,
        });
      }
    }
  }

  DeduplicateEdges(edges);

  // SOLVE!!!
  TarelStateRemapResult remap = RemapTarelStates(edges, required);
  TspGraphData graph = MakeTspGraphEdges(remap.edges, boundary);

  // Check that at least one representative from each group of required stops
  // appears in `graph`.
  //
  // This is necessary for correctness because the above construction can omit
  // stops from `graph`, and if it does, then the TSP on `graph` will give a
  // solution that does not reach all the stops. (Specifically, stops that don't
  // appear as both origins and destinations are omitted).
  std::unordered_set<StopId> representatives_in_graph;
  for (const TarelState& tarel_state : graph.state_by_id) {
    representatives_in_graph.insert(required.Representative(tarel_state.stop));
  }
  for (StopId rep : required.GroupRepresentatives()) {
    if (!representatives_in_graph.contains(rep)) {
      std::cout << "  missing required, no solution\n";
      return std::nullopt;
    }
  }

  std::optional<TspTourResult> result = SolveTspAndExtractTour(
      remap.edges,
      graph,
      boundary,
      ub_rel == -1 ? std::nullopt : std::optional(ub_rel),
      // &std::cout,
      nullptr,
      nullptr
  );
  if (!result.has_value()) {
    return std::nullopt;
  }

  // Map `result` states back to original states.
  for (TarelEdge& edge : result->tour_edges) {
    edge.origin = remap.mapped_to_original.at(edge.origin);
    edge.destination = remap.mapped_to_original.at(edge.destination);
  }

  return result;
}

struct TourAnalysis {
  int first_btaat_k = -1;
  TimeSinceServiceStart first_btaat;
  TarelEdge first_btaat_edge;
  TimeSinceServiceStart first_btaat_good_a;
  TimeSinceServiceStart first_btaat_good_b;

  TimeSinceServiceStart t_relaxed;
  TimeSinceServiceStart t_actual;
};

TourAnalysis AnalyzeTour(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    const BnbState& state,
    const TspTourResult& result,
    TimeSinceServiceStart t0
) {
  const auto& step_states = state.step_states;
  TourAnalysis analysis;
  analysis.t_relaxed = t0;
  analysis.t_actual = t0;

  // for (const TarelEdge& edge : result.tour_edges) {
  //   std::cout << problem.StopName(edge.origin.stop) << " @ " <<
  //   edge.origin.partition.v << " -> "
  //     << problem.StopName(edge.destination.stop) << " @ " <<
  //     edge.destination.partition.v << ": "
  //     << TimeSinceServiceStart{edge.weight} << "\n";
  // }

  std::cout << "  " << result.tour_edges[0].origin.partition.v << ". "
            << problem.StopName(result.tour_edges[0].origin.stop) << " @ "
            << analysis.t_relaxed << " / " << analysis.t_actual << "\n";
  for (int i = 0; i < result.tour_edges.size(); ++i) {
    const TarelEdge& edge = result.tour_edges[i];
    analysis.t_relaxed.seconds += edge.weight;

    // TODO: This lookup is pretty subtle and delicate.
    // Explain it. Make it more robust. (e.g. when not-found, exit immediately
    // instead of setting a huge dur). Think about whether it's actually
    // correct.
    int dur_actual;
    bool found = false;
    auto from_o_it = step_states[i].find(edge.origin.stop);
    if (from_o_it != step_states[i].end()) {
      for (const ArrivalTimeState& ats : from_o_it->second.states) {
        if (ats.arrival_time < analysis.t_actual) {
          continue;
        }
        for (const OnwardsStep& onwards : ats.onwards) {
          if (onwards.destination == edge.destination.stop) {
            dur_actual = onwards.duration +
                         (ats.arrival_time.seconds - analysis.t_actual.seconds);
            found = true;
            break;
          }
        }
        if (found == true) {
          break;
        }
      }
    }
    if (!found) {
      dur_actual = std::numeric_limits<int>::max() - analysis.t_actual.seconds;
    }

    {
      std::map<int, std::vector<TimeSinceServiceStart>> dur_hist;

      int better_than_actual_dur = dur_actual;
      TimeSinceServiceStart better_than_actual_arrival_time;

      // The biggest a < t_actual such that dur(a) < dur_actual.
      TimeSinceServiceStart good_a{std::numeric_limits<int>::min()};

      // The smallest b > t_actual such that dur(b) < dur_actual.
      TimeSinceServiceStart good_b{std::numeric_limits<int>::max()};

      if (found) {
        for (const ArrivalTimeState& ats : from_o_it->second.states) {
          for (const OnwardsStep& onwards : ats.onwards) {
            if (onwards.destination == edge.destination.stop) {
              dur_hist[onwards.duration].push_back(ats.arrival_time);

              if (onwards.duration < better_than_actual_dur) {
                better_than_actual_dur = onwards.duration;
                better_than_actual_arrival_time = ats.arrival_time;
              }

              if (ats.arrival_time < analysis.t_actual &&
                  ats.arrival_time > good_a && onwards.duration < dur_actual) {
                good_a = ats.arrival_time;
              }
              if (ats.arrival_time > analysis.t_actual &&
                  ats.arrival_time < good_b && onwards.duration < dur_actual) {
                good_b = ats.arrival_time;
              }

              break;
            }
          }
        }
      }

      // std::cout << "    good region: (" << good_a << ", " << good_b << ")\n";

      if (better_than_actual_dur < dur_actual) {
        // std::cout << "    BTAAT: " << better_than_actual_arrival_time <<
        // "\n";
        if (analysis.first_btaat_k == -1) {
          analysis.first_btaat_k = i - 1;
          analysis.first_btaat = better_than_actual_arrival_time;
          analysis.first_btaat_edge = edge;
          assert(
              good_a.seconds > std::numeric_limits<int>::min() ||
              good_b.seconds < std::numeric_limits<int>::max()
          );
          analysis.first_btaat_good_a = good_a;
          analysis.first_btaat_good_b = good_b;
        }
      }

      //   for (const auto& [dur, arrivals] : dur_hist) {
      //     std::cout << "    " << TimeSinceServiceStart{dur} << ": ";
      //     if (arrivals.size() <= 3) {
      //       for (size_t j = 0; j < arrivals.size(); ++j) {
      //         if (j > 0) std::cout << ", ";
      //         std::cout << arrivals[j];
      //       }
      //     } else {
      //       std::cout << arrivals.front() << ", ..., " << arrivals.back() <<
      //       " ("
      //                 << arrivals.size() << ")";
      //     }
      //     if (dur == dur_actual) {
      //       std::cout << " ****";
      //     }
      //     std::cout << "\n";
      //   }
    }

    analysis.t_actual.seconds += dur_actual;
    std::cout << "  " << edge.destination.partition.v << ". "
              << problem.StopName(edge.destination.stop) << " @ "
              << analysis.t_relaxed << " / " << analysis.t_actual << "\n";
  }

  std::cout << "  ubs: "
            << TimeSinceServiceStart{analysis.t_relaxed.seconds - t0.seconds}
            << " / "
            << TimeSinceServiceStart{analysis.t_actual.seconds - t0.seconds}
            << "\n";

  // if (analysis.first_btaat_k != -1) {
  //   std::cout << "  First BTAAT:\n"
  //             << "    step " << analysis.first_btaat_k << "\n"
  //             << "    edge "
  //             << problem.StopName(analysis.first_btaat_edge.origin.stop)
  //             << " -> "
  //             << problem.StopName(analysis.first_btaat_edge.destination.stop)
  //             << "\n"
  //             << "    arrival time " << analysis.first_btaat << "\n"
  //             << "    good region (" << analysis.first_btaat_good_a << ", "
  //             << analysis.first_btaat_good_b << ")\n";
  // }

  return analysis;
}

BnbState RequireStopStep(const BnbState& state, int k, StopId s) {
  BnbState result = state;
  std::erase_if(result.step_states[k], [&](const auto& pair) {
    return pair.first != s;
  });
  // TODO: This could instead be another pass that realizes that a certain step
  // has a single stop and then delete that stop from all the other steps.
  for (int i = 0; i < result.step_states.size(); ++i) {
    if (i == k) {
      continue;
    }
    std::erase_if(result.step_states[i], [&](const auto& pair) {
      return pair.first == s;
    });
  }
  FilterStepStatesSweep(result, k);
  return result;
}

BnbState ForbidStopStep(const BnbState& state, int k, StopId s) {
  BnbState result = state;
  std::erase_if(result.step_states[k], [&](const auto& pair) {
    return pair.first == s;
  });
  FilterStepStatesSweep(result, k);
  return result;
}

struct BnbNode {
  int lb;
  std::vector<std::string> constraints;
  BnbState state;
  bool operator<(const BnbNode& other) const { return lb > other.lb; }
};

int Bnb(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    StopId s0,
    TimeSinceServiceStart t0,
    int lb_rel,
    int ub_rel
) {
  std::vector<BnbNode> q;
  auto PushQ =
      [&](int lb, std::vector<std::string> constraints, BnbState state) {
        q.emplace_back(lb, std::move(constraints), std::move(state));
        std::push_heap(q.begin(), q.end());
      };

  PushQ(
      0,
      {},
      ComputeStepStates(
          completed, problem.required, problem.boundary, s0, t0, lb_rel, ub_rel
      )
  );

  // for (int k = 0; k < q[0].state.step_states.size(); ++k) {
  //   std::cout << k << " origins ";
  //   for (const auto& [s, _] : q[0].state.step_states[k]) {
  //     std::cout << problem.StopName(s) << " ";
  //   }
  //   std::cout << "\n";

  //   std::unordered_set<StopId> dests;
  //   for (const auto& [s, states] : q[0].state.step_states[k]) {
  //     for (const auto& ats : states.states) {
  //       for (const auto& onwards : ats.onwards) {
  //         dests.insert(onwards.destination);
  //       }
  //     }
  //   }

  //   std::cout << k << " dests ";
  //   for (StopId dest : dests) {
  //     std::cout << problem.StopName(dest) << " ";
  //   }
  //   std::cout << "\n";
  // }

  int iter_count = 0;
  int best_ub = ub_rel;
  while (q.size() > 0) {
    iter_count += 1;
    if (iter_count >= 2) {
      return best_ub;
    }

    std::pop_heap(q.begin(), q.end());
    BnbNode cur = std::move(q.back());
    q.pop_back();

    CombineForcedSteps(cur.state);

    int first_unknown_stop_k = 0;
    while (first_unknown_stop_k < cur.state.step_states.size() &&
           cur.state.step_states[first_unknown_stop_k].size() == 1) {
      first_unknown_stop_k += 1;
    }
    assert(first_unknown_stop_k < cur.state.step_states.size());
    int first_unknown_stop_cardinality =
        cur.state.step_states[first_unknown_stop_k].size();

    int last_unknown_stop_k = cur.state.step_states.size() - 1;
    while (last_unknown_stop_k >= 0 &&
           cur.state.step_states[last_unknown_stop_k].size() == 1) {
      last_unknown_stop_k -= 1;
    }
    assert(last_unknown_stop_k >= 0);

    std::cout << "iter " << iter_count << ": take "
              << TimeSinceServiceStart{cur.lb} << " (" << (q.size() + 1)
              << " active)\n";
    for (const std::string& constraint : cur.constraints) {
      std::cout << "  - " << constraint << "\n";
    }
    for (int k = std::max(0, first_unknown_stop_k - 1);
         k <= std::min(
                  last_unknown_stop_k + 1,
                  static_cast<int>(cur.state.step_states.size()) - 1
              );
         ++k) {
      if (k <= first_unknown_stop_k + kMaxStep ||
          k >= last_unknown_stop_k - kMaxStep) {
        TimeSinceServiceStart min_at{std::numeric_limits<int>::max()},
            max_at{std::numeric_limits<int>::min()};
        for (const auto& [s, states] : cur.state.step_states[k]) {
          for (const auto& ats : states.states) {
            min_at = std::min(ats.arrival_time, min_at);
            max_at = std::max(ats.arrival_time, max_at);
          }
        }

        std::cout << "  + " << k << ": ";
        if (cur.state.step_states[k].size() == 1) {
          std::cout << problem.StopName(
              cur.state.step_states[k].begin()->first
          );
        } else {
          std::cout << cur.state.step_states[k].size();
        }

        std::cout << " [";
        if (min_at == max_at) {
          std::cout << min_at;
        } else {
          std::cout << min_at << ", " << max_at;
        }
        std::cout << "]\n";
      } else if (k == first_unknown_stop_k + kMaxStep + 1) {
        std::cout << "  + ...\n";
      }
    }

    if (cur.lb > best_ub) {
      std::cout << "  terminated: smallest lb >= best_ub\n";
      return best_ub;
    }

    std::optional<TspTourResult> result =
        DoTSP(completed, problem.required, cur.state, ub_rel);
    if (!result.has_value()) {
      std::cout << "  pruned: no TSP result\n";
      continue;
    }
    std::cout << "  new lb: " << TimeSinceServiceStart{result->optimal_value}
              << "\n";
    if (result->optimal_value >= best_ub) {
      std::cout << "  pruned: lb >= best_ub\n";
      continue;
    }

    TourAnalysis analysis =
        AnalyzeTour(problem, completed, cur.state, *result, t0);
    best_ub = std::min(best_ub, analysis.t_actual.seconds - t0.seconds);
    if (result->optimal_value >= best_ub) {
      std::cout << "  pruned: lb >= cur_ub\n";
      continue;
    }

    // There must be a BTAAT cuz otherwise the lb would equal the ub.
    // TODO: Reconsider what happens when kMaxStep.
    // assert(analysis.first_btaat_k != -1);

    // Branch on what is the last stop.
    {
      int constraint_k = last_unknown_stop_k;
      StopId constraint_s = result->tour_edges[constraint_k].origin.stop;

      // Branch: Require s@k.
      {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << "Require " << problem.StopName(constraint_s) << " @ "
                   << constraint_k;
        branch_constraints.push_back(constraint.str());
        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            RequireStopStep(cur.state, constraint_k, constraint_s)
        );
      }

      // Branch: Forbid s@k.
      {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << "Forbid " << problem.StopName(constraint_s) << " @ "
                   << constraint_k;
        branch_constraints.push_back(constraint.str());
        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            ForbidStopStep(cur.state, constraint_k, constraint_s)
        );
      }
    }
    continue;

    // Branch halfway between possible times of last step.
    {
      int branch_k = cur.state.step_states.size() - 4;

      TimeSinceServiceStart min_at{std::numeric_limits<int>::max()},
          max_at{std::numeric_limits<int>::min()};
      for (const auto& [s, states] : cur.state.step_states[branch_k]) {
        for (const auto& ats : states.states) {
          min_at = std::min(ats.arrival_time, min_at);
          max_at = std::max(ats.arrival_time, max_at);
        }
      }

      TimeSinceServiceStart branch_at{min_at.seconds / 2 + max_at.seconds / 2};

      // Branch: t < branch_at.
      {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << "* @ " << branch_k << " < " << branch_at;
        branch_constraints.push_back(constraint.str());

        BnbState branch_state = cur.state;
        for (auto& [s, states] : branch_state.step_states[branch_k]) {
          std::erase_if(states.states, [&](const ArrivalTimeState& ats) {
            return ats.arrival_time >= branch_at;
          });
        }

        FilterStepStatesSweep(branch_state, branch_k);

        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }

      // Branch: t >= branch_at.
      {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << "* @ " << branch_k << " >= " << branch_at;
        branch_constraints.push_back(constraint.str());

        BnbState branch_state = cur.state;
        for (auto& [s, states] : branch_state.step_states[branch_k]) {
          std::erase_if(states.states, [&](const ArrivalTimeState& ats) {
            return ats.arrival_time < branch_at;
          });
        }

        FilterStepStatesSweep(branch_state, branch_k);

        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }
    }
    continue;

    // TODO: Think about whether this is an appropriate condition for when we
    // need to require/forbid s@k.
    if (
        first_unknown_stop_cardinality < 5 || analysis.first_btaat_k == -1 ||
        analysis.first_btaat_k >=
            first_unknown_stop_k + kMaxStep  // TODO: Off-by-1?
    ) {
      // Branches: Require/forbid s@k.
      int constraint_k = first_unknown_stop_k;
      StopId constraint_s = result->tour_edges[constraint_k].destination.stop;

      // Branch: Require s@k.
      {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << "Require " << problem.StopName(constraint_s) << " @ "
                   << constraint_k;
        branch_constraints.push_back(constraint.str());
        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            RequireStopStep(cur.state, constraint_k, constraint_s)
        );
      }

      // Branch: Forbid s@k.
      {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << "Forbid " << problem.StopName(constraint_s) << " @ "
                   << constraint_k;
        branch_constraints.push_back(constraint.str());
        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            ForbidStopStep(cur.state, constraint_k, constraint_s)
        );
      }
    } else {
      StopId constraint_s = analysis.first_btaat_edge.origin.stop;
      int constraint_k = analysis.first_btaat_k;

      // Branch: Forbid the stop.
      {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << "Forbid " << problem.StopName(constraint_s) << " @ "
                   << constraint_k;
        branch_constraints.push_back(constraint.str());
        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            ForbidStopStep(cur.state, constraint_k, constraint_s)
        );
      }

      BnbState require_state =
          RequireStopStep(cur.state, constraint_k, constraint_s);

      // Branch: First BAD region, and require the stop.
      // TODO: Think harder about whether I can omit branch for BAD region in
      // the -inf case.
      if (analysis.first_btaat_good_a.seconds !=
          std::numeric_limits<int>::min()) {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << problem.StopName(analysis.first_btaat_edge.origin.stop)
                   << " @ " << analysis.first_btaat_k
                   << " <= " << analysis.first_btaat_good_a;
        branch_constraints.push_back(constraint.str());

        BnbState branch_state = require_state;
        std::erase_if(
            branch_state
                .step_states[analysis.first_btaat_k]
                            [analysis.first_btaat_edge.origin.stop]
                .states,
            [&](const ArrivalTimeState& ats) {
              return ats.arrival_time > analysis.first_btaat_good_a;
            }
        );
        FilterStepStatesSweep(branch_state, analysis.first_btaat_k);

        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }

      // Branch: GOOD region, and require the stop.
      {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << problem.StopName(analysis.first_btaat_edge.origin.stop)
                   << " @ " << analysis.first_btaat_k << " ("
                   << analysis.first_btaat_good_a << ", "
                   << analysis.first_btaat_good_b << ")";
        branch_constraints.push_back(constraint.str());

        BnbState branch_state = require_state;
        std::erase_if(
            branch_state
                .step_states[analysis.first_btaat_k]
                            [analysis.first_btaat_edge.origin.stop]
                .states,
            [&](const ArrivalTimeState& ats) {
              return ats.arrival_time <= analysis.first_btaat_good_a ||
                     ats.arrival_time >= analysis.first_btaat_good_b;
            }
        );
        FilterStepStatesSweep(branch_state, analysis.first_btaat_k);

        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }

      // Branch: Second BAD region, and require the stop.
      // TODO: Think harder about whether I can omit branch for BAD region in
      // the inf case.
      if (analysis.first_btaat_good_b.seconds !=
          std::numeric_limits<int>::max()) {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << problem.StopName(analysis.first_btaat_edge.origin.stop)
                   << " @ " << analysis.first_btaat_k
                   << " >= " << analysis.first_btaat_good_b;
        branch_constraints.push_back(constraint.str());

        BnbState branch_state = require_state;
        std::erase_if(
            branch_state
                .step_states[analysis.first_btaat_k]
                            [analysis.first_btaat_edge.origin.stop]
                .states,
            [&](const ArrivalTimeState& ats) {
              return ats.arrival_time < analysis.first_btaat_good_b;
            }
        );
        FilterStepStatesSweep(branch_state, analysis.first_btaat_k);

        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }
    }
  }

  return best_ub;
}

int main(int argc, char* argv[]) {
  CLI::App app{"Track count tool"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  std::string gtfs_stop_id_str;
  app.add_option("gtfs_stop_id", gtfs_stop_id_str, "GTFS stop ID");

  std::string time_str;
  app.add_option("time", time_str, "Departure time in hh:mm:ss format");

  std::string lb_str;
  app.add_option(
      "--lb",
      lb_str,
      "Lower bound in hh:mm:ss format (relative to departure time)"
  );

  std::string ub_str;
  app.add_option(
      "--ub",
      ub_str,
      "Upper bound in hh:mm:ss format (relative to departure time)"
  );

  CLI11_PARSE(app, argc, argv);

  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << "\n";
    return 1;
  }

  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState problem = j.get<ProblemState>();
  in.close();

  // Reduce it to just the group representatives (eliminates non-required stops
  // and multi-stop groups which the later codes do not support yet).
  //
  // TODO: Make later codes support this stuff.
  auto fooized = ReduceToMinimalSystemPaths(
      problem.minimal, problem.required.GroupRepresentatives()
  );
  problem.minimal = MakeAdjacencyList(fooized.AllMergedSteps());
  std::cout << "required size " << problem.required.size() << "\n";

  if (gtfs_stop_id_str.empty()) {
    ListStops(problem);
    return 0;
  }

  StopId stop = FindStopByGtfsId(problem, gtfs_stop_id_str);

  int lb_rel = -1;
  if (!lb_str.empty()) {
    lb_rel = TimeSinceServiceStart::Parse(lb_str).seconds;
  }

  int ub_rel = -1;
  if (!ub_str.empty()) {
    ub_rel = TimeSinceServiceStart::Parse(ub_str).seconds;
  }

  // Collect departure times to iterate over.
  std::vector<TimeSinceServiceStart> departure_times;
  if (time_str.empty()) {
    // Gather all distinct departure times from this stop.
    std::set<TimeSinceServiceStart> times_set;
    for (const auto& group : problem.minimal.GetGroups(stop)) {
      for (const auto& step : problem.minimal.GetSteps(group)) {
        times_set.insert(step.origin_time);
      }
    }
    departure_times.assign(times_set.begin(), times_set.end());
  } else {
    departure_times.push_back(TimeSinceServiceStart::Parse(time_str));
  }

  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());

  // StopId cand_next = FindStopByGtfsId(problem, "PS_GRAM");
  // auto step_states_base = ComputeStepStates(
  //     completed, problem.required, problem.boundary, stop,
  //     departure_times[0], lb_rel, ub_rel
  // );

  auto PrintTsp = [&](const BnbState& state) {
    std::optional<TspTourResult> result =
        DoTSP(completed, problem.required, state, ub_rel);
    if (!result.has_value()) {
      std::cout << "  pruned: no TSP result\n";
      return;
    }
    std::cout << "  new lb: " << TimeSinceServiceStart{result->optimal_value}
              << "\n";
    TourAnalysis analysis =
        AnalyzeTour(problem, completed, state, *result, departure_times[0]);
  };

  // std::cout << "BASE\n";
  // PrintTsp(step_states_base);

  // std::cout << "Forbid Candidate\n";
  // PrintTsp(ForbidStopStep(step_states_base, 2, cand_next));

  // return 0;

  for (const TimeSinceServiceStart& t0 : departure_times) {
    std::cout << t0.ToString() << "\n";
    int best_ub = Bnb(problem, completed, stop, t0, lb_rel, ub_rel);
    std::cout << "  FINAL UB: " << TimeSinceServiceStart{best_ub} << "\n";

    // TODO: Can decrease ub if we found a better ub.

    // TODO: Because of periodicity, paths departing at other times that
    // follow the same sequence might also be pretty good or even better, so
    // we should consider them!!
  }

  return 0;
}
