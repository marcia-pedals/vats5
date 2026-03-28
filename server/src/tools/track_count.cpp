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

// Propagates `step_states[k]` into `step_states[k + 1]`, replacing it.
void PropagateStepStatesForwards(
    const StepsAdjacencyList& completed,
    const ProblemBoundary& boundary,
    const TimeSinceServiceStart t_ub,
    std::vector<std::unordered_map<StopId, StepState>>& step_states,
    StopId s0,
    int k
) {
  assert(k >= 0);
  assert(k + 1 < step_states.size());
  step_states[k + 1].clear();

  for (const auto& [s_cur, step_state_cur] : step_states[k]) {
    for (const StepGroup& g_next : completed.GetGroups(s_cur)) {
      StopId s_next = g_next.destination_stop;
      if (s_next == boundary.start || s_next == boundary.end || s_next == s0) {
        continue;
      }

      for (const ArrivalTimeState& ats : step_state_cur.states) {
        // TODO: Deal with flex steps.
        std::span<const AdjacencyListStep> group_steps =
            completed.GetSteps(g_next);
        size_t t_next_i =
            FindDepartureAtOrAfter(completed, g_next, ats.arrival_time);
        if (t_next_i >= group_steps.size()) {
          continue;
        }
        TimeSinceServiceStart t_next = group_steps[t_next_i].destination_time;
        if (t_next > t_ub) {
          continue;
        }
        step_states[k + 1][s_next].states.push_back({.arrival_time = t_next});
      }
    }
  }

  for (auto& [_, step_state] : step_states[k + 1]) {
    step_state.SortAndDedupe();
  }
}

// Deletes states from `step_states[k + 1]` that do not come from anything in
// `step_states[k]`.
void FilterStepStatesForwards(
    std::vector<std::unordered_map<StopId, StepState>>& step_states, int k
) {
  assert(k >= 0);
  assert(k + 1 < step_states.size());

  // arrival_times[s] is all the arrival times to s that come from something in
  // step_states[k].
  std::unordered_map<StopId, std::unordered_set<TimeSinceServiceStart>>
      arrival_times;
  for (const auto& [s_cur, step_state_cur] : step_states[k]) {
    for (const ArrivalTimeState& ats : step_state_cur.states) {
      for (const OnwardsStep& onwards : ats.onwards) {
        arrival_times[onwards.destination].insert(
            TimeSinceServiceStart{ats.arrival_time.seconds + onwards.duration}
        );
      }
    }
  }

  for (auto& [s_next, step_state_next] : step_states[k + 1]) {
    const std::unordered_set<TimeSinceServiceStart>& arrival_times_next =
        arrival_times[s_next];
    std::erase_if(step_state_next.states, [&](const ArrivalTimeState& ats) {
      return !arrival_times_next.contains(ats.arrival_time);
    });
  }

  std::erase_if(step_states[k + 1], [](const auto& pair) {
    return pair.second.states.size() == 0;
  });
}

// Deletes states from `step_states[k]` that do not lead to anything in
// `step_states[k + 1]`.
void FilterStepStatesBackwards(
    std::vector<std::unordered_map<StopId, StepState>>& step_states, int k
) {
  assert(k >= 0);
  assert(k + 1 < step_states.size());

  // arrival_times[s] is all the arrival times to s in step_states[k + 1].
  std::unordered_map<StopId, std::unordered_set<TimeSinceServiceStart>>
      arrival_times;
  for (const auto& [s_next, step_state_next] : step_states[k + 1]) {
    for (const ArrivalTimeState& ats : step_state_next.states) {
      arrival_times[s_next].insert(ats.arrival_time);
    }
  }

  for (auto& [s_cur, step_state_cur] : step_states[k]) {
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

  std::erase_if(step_states[k], [](const auto& pair) {
    return pair.second.states.size() == 0;
  });
}

// Sweeps a forwards filter from k0 to the end and then a backwards filter to
// the start.
void FilterStepStatesSweep(
    std::vector<std::unordered_map<StopId, StepState>>& step_states, int k0
) {
  for (int k = k0; k + 1 < step_states.size(); ++k) {
    FilterStepStatesForwards(step_states, k);
  }
  for (int k = step_states.size() - 2; k >= 0; --k) {
    FilterStepStatesBackwards(step_states, k);
  }

  std::unordered_set<StopId> known_step_stops;
  for (int i = 0; i < step_states.size(); ++i) {
    if (step_states[i].size() == 1) {
      known_step_stops.insert(step_states[i].begin()->first);
    }
  }
  bool changed_anything = false;
  for (int i = 0; i < step_states.size(); ++i) {
    if (step_states[i].size() == 1) {
      continue;
    }
    auto erased_n = std::erase_if(step_states[i], [&](const auto& pair) {
      return known_step_stops.contains(pair.first);
    });
    changed_anything |= erased_n > 0;
  }

  // TODO: Loop until everything stops changing?
  if (changed_anything) {
    for (int k = 0; k + 1 < step_states.size(); ++k) {
      FilterStepStatesForwards(step_states, k);
    }
    for (int k = step_states.size() - 2; k >= 0; --k) {
      FilterStepStatesBackwards(step_states, k);
    }
  }

  // TODO: We can also find known step stops by noticing if there are any stops
  // that appear in only one step.
}

// Recomputes all the `.onwards` fields in `step_states_k`.
// TODO: Probably put this inside the "propagate forwards helper".
void RecomputeOnwardsSteps(
    const StepsAdjacencyList& completed,
    const ProblemBoundary& boundary,
    std::unordered_map<StopId, StepState>& step_states_k
) {
  for (auto& [s_cur, step_state_cur] : step_states_k) {
    for (auto& ats : step_state_cur.states) {
      ats.onwards.clear();
      for (const StepGroup& g_next : completed.GetGroups(s_cur)) {
        StopId s_next = g_next.destination_stop;
        if (s_next == boundary.start || s_next == boundary.end) {
          continue;
        }

        size_t t_next_i =
            FindDepartureAtOrAfter(completed, g_next, ats.arrival_time);
        std::span<const AdjacencyListStep> group_steps =
            completed.GetSteps(g_next);
        if (t_next_i >= group_steps.size()) {
          continue;
        }
        TimeSinceServiceStart t_next = group_steps[t_next_i].destination_time;
        ats.onwards.push_back(
            {.destination = s_next,
             .duration = t_next.seconds - ats.arrival_time.seconds}
        );
      }
    }
  }
}

std::vector<std::unordered_map<StopId, StepState>> ComputeStepStates(
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

  // step_states[0]: Base case, s0@t0
  // step_states[k]: All possible states after making k steps.
  //
  // Note that at step_states[k] we have visited k+1 stops, so if there are n
  // stops then we need k+1=n-2 <=> k=n-3. (-2 because we ignore boundary
  // stops).
  // TODO: Handle boundary stops more elegantly.
  std::vector<std::unordered_map<StopId, StepState>> step_states(
      static_cast<int>(required.size()) - 2
  );

  step_states[0][s0].states.push_back({.arrival_time = t0});
  for (int k = 0; k < static_cast<int>(required.size()) - 3; ++k) {
    PropagateStepStatesForwards(completed, boundary, t_ub, step_states, s0, k);
  }

  // Anything arriving at the end before t_lb is too good to be true.
  for (auto& [s, step_state] : step_states.back()) {
    std::erase_if(step_state.states, [&](const ArrivalTimeState& ats) {
      return ats.arrival_time < t_lb;
    });
  }

  for (int k = 0; k < step_states.size(); ++k) {
    RecomputeOnwardsSteps(completed, boundary, step_states[k]);
  }

  // Now go backwardsly and filter any arrival times that do not lead to actual
  // arrival times that we have.
  for (int k = static_cast<int>(step_states.size()) - 2; k >= 0; --k) {
    FilterStepStatesBackwards(step_states, k);
  }

  // for (int k = 0; k < step_states.size(); ++k) {
  //   const std::unordered_map<StopId, StepState>& step_state_k =
  //   step_states[k]; int max_smear = 0; TimeSinceServiceStart
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

  return step_states;
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

struct ForcedAffix {
  std::vector<StopId> sequence;
  std::vector<TimeSinceServiceStart> arrival_times;

  // In case of empty sequence, there is a START->END edge of weight 0.
  // In case of nonempty sequence, the edges are:
  // START->a1: weight 0
  // a1->a2
  // ...
  // a(n-1)->an
  // an->END: weight 0
  //
  // Note that when concatenating with other paths, you have to "merge" the
  // boundary edges and compute what the weight is for the merged boundary edge.
  std::vector<TarelEdge> edges;
};

enum class AffixDirection { kPrefix, kSuffix };

ForcedAffix ComputeForcedAffix(
    AffixDirection direction,
    const ProblemBoundary& boundary,
    const std::vector<std::unordered_map<StopId, StepState>>& step_states
) {
  TarelState start_state{boundary.start, StepPartitionId::NONE};
  TarelState end_state{boundary.end, StepPartitionId::NONE};

  int affix_start, affix_end;
  if (direction == AffixDirection::kPrefix) {
    affix_start = 0;
    affix_end = 0;
    while (affix_end < step_states.size() &&
           step_states[affix_end].size() == 1) {
      ++affix_end;
    }
  } else {
    affix_end = step_states.size();
    affix_start = affix_end;
    while (affix_start > 0 && step_states[affix_start - 1].size() == 1) {
      --affix_start;
    }

    if (affix_start == 0) {
      // Break symmetry: If the whole sequence is forced, then treat it as a
      // prefix with no affix:
      affix_start = affix_end;
    }
  }

  ForcedAffix result;

  for (int k = affix_start; k < affix_end; ++k) {
    assert(step_states[k].size() == 1);
    const auto& [stop, step_state] = *step_states[k].begin();
    result.sequence.push_back(stop);

    // Oh no with a suffix there is not a single known arrival time!!
    // However we could compute the min-weight a bit more precisely than taking
    // all the mins?? Like we can DP it if we know the ending sequence!!! Yay
    // for precision.
    // TODO(Saturday): Maybe start here and then finish up the suffix-append in
    // DoTSP. (That maybe should also be included in the DP??).
    assert(step_state.states.size() > 0);
    result.arrival_times.push_back(step_state.states[0].arrival_time);
  }

  for (int i = 0; i < result.sequence.size() + 1; ++i) {
    int k = affix_start + i;
    TarelState origin =
        i == 0 ? start_state
               : TarelState{result.sequence[i - 1], StepPartitionId{k - 1}};
    TarelState destination =
        i == result.sequence.size()
            ? end_state
            : TarelState{result.sequence[i], StepPartitionId{k}};
    int weight;
    if (i == 0 || i == result.sequence.size()) {
      weight = 0;
    } else {
      weight =
          result.arrival_times[i].seconds - result.arrival_times[i - 1].seconds;
    }
    result.edges.emplace_back(origin, destination, weight);
  }

  return result;
}

constexpr int kMaxStep = 5;

std::optional<TspTourResult> DoTSP(
    const StepsAdjacencyList& completed,
    const RequiredStops& required,
    const ProblemBoundary& boundary,
    const std::vector<std::unordered_map<StopId, StepState>>& step_states,
    int ub_rel
) {
  TarelState start_state{boundary.start, StepPartitionId::NONE};
  TarelState end_state{boundary.end, StepPartitionId::NONE};

  for (const std::unordered_map<StopId, StepState>& step_state : step_states) {
    // TODO: Think about whether this is an expected condition and whether we
    // can/should detect it earlier.
    if (step_state.size() == 0) {
      return std::nullopt;
    }
  }

  ForcedAffix forced_prefix =
      ComputeForcedAffix(AffixDirection::kPrefix, boundary, step_states);

  // When the whole sequence is forced, we put it all in the prefix, so there is
  // no need to compute or check the suffix.
  if (forced_prefix.sequence.size() == step_states.size()) {
    int duration = 0;
    for (const TarelEdge& edge : forced_prefix.edges) {
      duration += edge.weight;
    }
    return TspTourResult{
        .optimal_value = duration,
        .tour_edges = std::move(forced_prefix.edges),
    };
  }

  int first_unforced_k = forced_prefix.sequence.size();

  ForcedAffix forced_suffix =
      ComputeForcedAffix(AffixDirection::kSuffix, boundary, step_states);

  int last_unforced_k = step_states.size() - 1 - forced_suffix.sequence.size();

  auto ClampedPartition = [&](int step) -> StepPartitionId {
    if (step <= first_unforced_k + kMaxStep ||
        step >= last_unforced_k - kMaxStep) {
      return StepPartitionId{step};
    } else {
      return StepPartitionId{first_unforced_k + kMaxStep};
    }
  };

  // Convention:
  // StepPartitionId::NONE is the partition for START and END.
  // StepPartitionId{k} is the partition for where you arrive after the k-th
  // step:
  // - StepPartitionId{0} is the first partition you get to after START.
  // - StepPartitionId{required.size() - 3} is the last partition after stepping
  // to END.
  std::vector<TarelEdge> edges;

  // END->START edge.
  edges.push_back({
      .origin = end_state,
      .destination = start_state,
      .weight = 0,
  });

  // Insert all the edges!!!!
  for (int k = first_unforced_k; k < last_unforced_k; ++k) {
    const std::unordered_map<StopId, StepState>& step_state_kminus1 =
        step_states[k - 1];

    for (const auto& [s_cur, step_state_cur] : step_state_kminus1) {
      TarelState origin;
      if (k == first_unforced_k) {
        origin = TarelState{boundary.start, StepPartitionId::NONE};
      } else {
        origin = TarelState{s_cur, ClampedPartition(k - 1)};
      }

      if (k == required.size() - 2) {
        edges.push_back({
            .origin = origin,
            .destination = TarelState{boundary.end, StepPartitionId::NONE},
            .weight = 0,
        });
        continue;
      }

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
            .origin = origin,
            .destination = TarelState{s_next, ClampedPartition(k)},
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
  for (StopId s : forced_prefix.sequence) {
    representatives_in_graph.insert(s);
  }
  for (StopId s : forced_suffix.sequence) {
    representatives_in_graph.insert(s);
  }
  for (const TarelState& tarel_state : graph.state_by_id) {
    representatives_in_graph.insert(required.Representative(tarel_state.stop));
  }
  for (StopId rep : required.GroupRepresentatives()) {
    if (!representatives_in_graph.contains(rep)) {
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

  // Insert forced affixes into things.
  if (forced_prefix.sequence.size() > 0) {
    // The first step from a forced prefix must have only one possible arrival
    // time because we know when it started!!
    const std::vector<ArrivalTimeState> atss =
        step_states[first_unforced_k]
            .at(result->tour_edges.front().destination.stop)
            .states;
    assert(atss.size() == 1);
    result->tour_edges.front().weight =
        atss[0].arrival_time.seconds -
        forced_prefix.arrival_times.back().seconds;
    result->tour_edges.front().origin = forced_prefix.edges.back().origin;
    result->tour_edges.insert(
        result->tour_edges.begin(),
        forced_prefix.edges.begin(),
        forced_prefix.edges.end() - 1
    );
  }
  if (forced_suffix.sequence.size() > 0) {
    const std::vector<ArrivalTimeState> atss =
        step_states[last_unforced_k]
            .at(result->tour_edges.back().origin.stop)
            .states;
    int min_weight = std::numeric_limits<int>::max();
    for (const ArrivalTimeState& ats : atss) {
      for (const OnwardsStep& onwards : ats.onwards) {
        if (onwards.destination == forced_suffix.sequence.front()) {
          min_weight = std::min(min_weight, onwards.duration);
        }
      }
    }
    result->tour_edges.back().weight = min_weight;
  }

  // Recompute optimal value cuz we might have added some edges and modified
  // some edge weights.
  result->optimal_value = 0;
  for (TarelEdge& edge : result->tour_edges) {
    result->optimal_value += edge.weight;
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
    const std::vector<std::unordered_map<StopId, StepState>>& step_states,
    const TspTourResult& result,
    TimeSinceServiceStart t0
) {
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

  // std::cout << "  " << result.tour_edges[1].origin.partition.v << ". "
  //           << problem.StopName(result.tour_edges[1].origin.stop) << " @ "
  //           << analysis.t_relaxed << " / " << analysis.t_actual << "\n";
  for (int i = 1; i + 1 < result.tour_edges.size(); ++i) {
    const TarelEdge& edge = result.tour_edges[i];
    analysis.t_relaxed.seconds += edge.weight;

    // TODO: This lookup is pretty subtle and delicate.
    // Explain it. Make it more robust. (e.g. when not-found, exit immediately
    // instead of setting a huge dur). Think about whether it's actually
    // correct.
    int dur_actual;
    bool found = false;
    auto from_o_it = step_states[i - 1].find(edge.origin.stop);
    if (from_o_it != step_states[i - 1].end()) {
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

    // NOTE: This commented-out code is wrong because it doesn't take into
    // account that we might have forbidden certain arrival times in this
    // branch.
    // int dur_actual;
    // bool found = false;
    // for (const StepGroup& g : completed.GetGroups(edge.origin.stop)) {
    //   if (g.destination_stop != edge.destination.stop) {
    //     continue;
    //   }
    //   std::span<const AdjacencyListStep> group_steps = completed.GetSteps(g);
    //   size_t t_next_i = FindDepartureAtOrAfter(completed, g,
    //   analysis.t_actual); assert(t_next_i < group_steps.size()); dur_actual =
    //       group_steps[t_next_i].destination_time.seconds -
    //       analysis.t_actual.seconds;
    //   found = true;
    //   break;
    // }
    // assert(found);

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
    // std::cout << "  " << edge.destination.partition.v << ". "
    //           << problem.StopName(edge.destination.stop) << " @ "
    //           << analysis.t_relaxed << " / " << analysis.t_actual << "\n";
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

std::vector<std::unordered_map<StopId, StepState>> RequireStopStep(
    const std::vector<std::unordered_map<StopId, StepState>>& step_states,
    int k,
    StopId s
) {
  std::vector<std::unordered_map<StopId, StepState>> result = step_states;
  std::erase_if(result[k], [&](const auto& pair) { return pair.first != s; });
  // TODO: This could instead be another pass that realizes that a certain step
  // has a single stop and then delete that stop from all the other steps.
  for (int i = 0; i < result.size(); ++i) {
    if (i == k) {
      continue;
    }
    std::erase_if(result[i], [&](const auto& pair) { return pair.first == s; });
  }
  FilterStepStatesSweep(result, k);
  return result;
}

std::vector<std::unordered_map<StopId, StepState>> ForbidStopStep(
    const std::vector<std::unordered_map<StopId, StepState>>& step_states,
    int k,
    StopId s
) {
  std::vector<std::unordered_map<StopId, StepState>> result = step_states;
  std::erase_if(result[k], [&](const auto& pair) { return pair.first == s; });
  FilterStepStatesSweep(result, k);
  return result;
}

struct BnbNode {
  int lb;
  int forced_prefix_size;
  std::vector<std::string> constraints;
  std::vector<std::unordered_map<StopId, StepState>> step_states;
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
      [&](int lb,
          int forced_prefix_size,
          std::vector<std::string> constraints,
          std::vector<std::unordered_map<StopId, StepState>> step_states) {
        q.emplace_back(
            lb,
            forced_prefix_size,
            std::move(constraints),
            std::move(step_states)
        );
        std::push_heap(q.begin(), q.end());
      };

  PushQ(
      0,
      1,
      {},
      std::move(ComputeStepStates(
          completed, problem.required, problem.boundary, s0, t0, lb_rel, ub_rel
      ))
  );

  int iter_count = 0;
  int best_ub = ub_rel;
  while (q.size() > 0) {
    iter_count += 1;

    std::pop_heap(q.begin(), q.end());
    BnbNode cur = std::move(q.back());
    q.pop_back();

    int first_unknown_stop_k = 0;
    while (first_unknown_stop_k < cur.step_states.size() &&
           cur.step_states[first_unknown_stop_k].size() == 1) {
      first_unknown_stop_k += 1;
    }
    assert(first_unknown_stop_k < cur.step_states.size());
    int first_unknown_stop_cardinality =
        cur.step_states[first_unknown_stop_k].size();

    int last_unknown_stop_k = cur.step_states.size() - 1;
    while (last_unknown_stop_k >= 0 &&
           cur.step_states[last_unknown_stop_k].size() == 1) {
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
                  static_cast<int>(cur.step_states.size()) - 1
              );
         ++k) {
      if (k <= first_unknown_stop_k + kMaxStep ||
          k >= last_unknown_stop_k - kMaxStep) {
        TimeSinceServiceStart min_at{std::numeric_limits<int>::max()},
            max_at{std::numeric_limits<int>::min()};
        for (const auto& [s, states] : cur.step_states[k]) {
          for (const auto& ats : states.states) {
            min_at = std::min(ats.arrival_time, min_at);
            max_at = std::max(ats.arrival_time, max_at);
          }
        }

        std::cout << "  + " << k << ": ";
        if (cur.step_states[k].size() == 1) {
          std::cout << problem.StopName(cur.step_states[k].begin()->first);
        } else {
          std::cout << cur.step_states[k].size();
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

    std::optional<TspTourResult> result = DoTSP(
        completed, problem.required, problem.boundary, cur.step_states, ub_rel
    );
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
        AnalyzeTour(problem, completed, cur.step_states, *result, t0);
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
            cur.forced_prefix_size + 1,
            std::move(branch_constraints),
            std::move(
                RequireStopStep(cur.step_states, constraint_k, constraint_s)
            )
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
            cur.forced_prefix_size + 1,
            std::move(branch_constraints),
            std::move(
                ForbidStopStep(cur.step_states, constraint_k, constraint_s)
            )
        );
      }
    }
    continue;

    // Branch halfway between possible times of last step.
    {
      int branch_k = cur.step_states.size() - 4;

      TimeSinceServiceStart min_at{std::numeric_limits<int>::max()},
          max_at{std::numeric_limits<int>::min()};
      for (const auto& [s, states] : cur.step_states[branch_k]) {
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

        std::vector<std::unordered_map<StopId, StepState>> branch_step_states =
            cur.step_states;
        for (auto& [s, states] : branch_step_states[branch_k]) {
          std::erase_if(states.states, [&](const ArrivalTimeState& ats) {
            return ats.arrival_time >= branch_at;
          });
        }

        FilterStepStatesSweep(branch_step_states, branch_k);

        PushQ(
            result->optimal_value,
            cur.forced_prefix_size,
            std::move(branch_constraints),
            std::move(branch_step_states)
        );
      }

      // Branch: t >= branch_at.
      {
        std::vector<std::string> branch_constraints = cur.constraints;
        std::stringstream constraint;
        constraint << "* @ " << branch_k << " >= " << branch_at;
        branch_constraints.push_back(constraint.str());

        std::vector<std::unordered_map<StopId, StepState>> branch_step_states =
            cur.step_states;
        for (auto& [s, states] : branch_step_states[branch_k]) {
          std::erase_if(states.states, [&](const ArrivalTimeState& ats) {
            return ats.arrival_time < branch_at;
          });
        }

        FilterStepStatesSweep(branch_step_states, branch_k);

        PushQ(
            result->optimal_value,
            cur.forced_prefix_size,
            std::move(branch_constraints),
            std::move(branch_step_states)
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
            cur.forced_prefix_size + 1,
            std::move(branch_constraints),
            std::move(
                RequireStopStep(cur.step_states, constraint_k, constraint_s)
            )
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
            cur.forced_prefix_size + 1,
            std::move(branch_constraints),
            std::move(
                ForbidStopStep(cur.step_states, constraint_k, constraint_s)
            )
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
            cur.forced_prefix_size,
            std::move(branch_constraints),
            std::move(
                ForbidStopStep(cur.step_states, constraint_k, constraint_s)
            )
        );
      }

      std::vector<std::unordered_map<StopId, StepState>> require_step_states =
          RequireStopStep(cur.step_states, constraint_k, constraint_s);

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

        std::vector<std::unordered_map<StopId, StepState>> branch_step_states =
            require_step_states;
        std::erase_if(
            branch_step_states[analysis.first_btaat_k]
                              [analysis.first_btaat_edge.origin.stop]
                                  .states,
            [&](const ArrivalTimeState& ats) {
              return ats.arrival_time > analysis.first_btaat_good_a;
            }
        );
        FilterStepStatesSweep(branch_step_states, analysis.first_btaat_k);

        PushQ(
            result->optimal_value,
            cur.forced_prefix_size,
            std::move(branch_constraints),
            std::move(branch_step_states)
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

        std::vector<std::unordered_map<StopId, StepState>> branch_step_states =
            require_step_states;
        std::erase_if(
            branch_step_states[analysis.first_btaat_k]
                              [analysis.first_btaat_edge.origin.stop]
                                  .states,
            [&](const ArrivalTimeState& ats) {
              return ats.arrival_time <= analysis.first_btaat_good_a ||
                     ats.arrival_time >= analysis.first_btaat_good_b;
            }
        );
        FilterStepStatesSweep(branch_step_states, analysis.first_btaat_k);

        PushQ(
            result->optimal_value,
            cur.forced_prefix_size,
            std::move(branch_constraints),
            std::move(branch_step_states)
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

        std::vector<std::unordered_map<StopId, StepState>> branch_step_states =
            require_step_states;
        std::erase_if(
            branch_step_states[analysis.first_btaat_k]
                              [analysis.first_btaat_edge.origin.stop]
                                  .states,
            [&](const ArrivalTimeState& ats) {
              return ats.arrival_time < analysis.first_btaat_good_b;
            }
        );
        FilterStepStatesSweep(branch_step_states, analysis.first_btaat_k);

        PushQ(
            result->optimal_value,
            cur.forced_prefix_size,
            std::move(branch_constraints),
            std::move(branch_step_states)
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

  auto PrintTsp =
      [&](
          const std::vector<std::unordered_map<StopId, StepState>>& step_states
      ) {
        std::optional<TspTourResult> result = DoTSP(
            completed, problem.required, problem.boundary, step_states, ub_rel
        );
        if (!result.has_value()) {
          std::cout << "  pruned: no TSP result\n";
          return;
        }
        std::cout << "  new lb: "
                  << TimeSinceServiceStart{result->optimal_value} << "\n";
        TourAnalysis analysis = AnalyzeTour(
            problem, completed, step_states, *result, departure_times[0]
        );
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
