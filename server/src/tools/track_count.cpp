#include <CLI/CLI.hpp>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <numbers>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"
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
  StepPartitionId partition;
};

struct OnwardsDuration {
  int duration;  // -1 = no connection to this destination
  StepPartitionId partition;
};

struct ArrivalTimeState {
  TimeSinceServiceStart arrival_time;

  // The destination partitions of all the onwards that come into here.
  std::vector<StepPartitionId> partitions;

  std::vector<OnwardsDuration> onwards;  // indexed by StepState::destinations
};

struct StepState;

class OnwardsStepsView {
 public:
  OnwardsStepsView(
      const std::vector<StopId>& destinations,
      const std::vector<OnwardsDuration>& onwards
  )
      : destinations_(destinations), onwards_(onwards) {}

  class Iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = OnwardsStep;
    using difference_type = std::ptrdiff_t;
    using pointer = void;
    using reference = OnwardsStep;

    Iterator(
        const std::vector<StopId>& destinations,
        const std::vector<OnwardsDuration>& onwards,
        size_t index
    )
        : destinations_(destinations), onwards_(onwards), index_(index) {
      SkipInvalid();
    }

    OnwardsStep operator*() const {
      return OnwardsStep{
          .destination = destinations_[index_],
          .duration = onwards_[index_].duration,
          .partition = onwards_[index_].partition,
      };
    }

    Iterator& operator++() {
      ++index_;
      SkipInvalid();
      return *this;
    }

    Iterator operator++(int) {
      Iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    bool operator==(const Iterator& other) const {
      return index_ == other.index_;
    }
    bool operator!=(const Iterator& other) const { return !(*this == other); }

   private:
    void SkipInvalid() {
      while (index_ < destinations_.size() &&
             (index_ >= onwards_.size() || onwards_[index_].duration == -1)) {
        ++index_;
      }
    }

    const std::vector<StopId>& destinations_;
    const std::vector<OnwardsDuration>& onwards_;
    size_t index_;
  };

  Iterator begin() const { return Iterator(destinations_, onwards_, 0); }
  Iterator end() const {
    return Iterator(destinations_, onwards_, destinations_.size());
  }

 private:
  const std::vector<StopId>& destinations_;
  const std::vector<OnwardsDuration>& onwards_;
};

struct StepState {
  std::vector<StopId> destinations;
  std::vector<ArrivalTimeState> states;

  OnwardsStepsView OnwardsSteps(const ArrivalTimeState& ats) const {
    return OnwardsStepsView(destinations, ats.onwards);
  }

  void AddOnwards(
      ArrivalTimeState& ats,
      StopId dest,
      int duration,
      StepPartitionId partition
  ) {
    auto it = std::find(destinations.begin(), destinations.end(), dest);
    size_t idx;
    if (it == destinations.end()) {
      idx = destinations.size();
      destinations.push_back(dest);
    } else {
      idx = static_cast<size_t>(it - destinations.begin());
    }
    if (idx >= ats.onwards.size()) {
      ats.onwards.resize(idx + 1, OnwardsDuration{.duration = -1});
    }
    ats.onwards[idx].duration = duration;
    ats.onwards[idx].partition = partition;
  }

  void ClearOnwards(ArrivalTimeState& ats) const {
    for (auto& od : ats.onwards) {
      od.duration = -1;
      od.partition = StepPartitionId::NONE;
    }
  }

  bool HasAnyOnwards(const ArrivalTimeState& ats) const {
    for (size_t i = 0; i < ats.onwards.size() && i < destinations.size(); ++i) {
      if (ats.onwards[i].duration != -1) {
        return true;
      }
    }
    return false;
  }

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

  // forced_after[i] is a sequence of stops that happen along the path from
  // step_states[i] to step_states[i+1].
  std::vector<std::vector<StopId>> forced_after;

  ProblemBoundary boundary;
  RequiredStops required;

  TimeSinceServiceStart T0() const {
    assert(step_states.size() > 0);
    assert(step_states[0].size() == 1);
    auto atss = step_states[0].begin()->second.states;
    assert(atss.size() == 1);
    return atss[0].arrival_time;
  }

  std::pair<TimeSinceServiceStart, TimeSinceServiceStart> StepBounds(
      int k
  ) const {
    TimeSinceServiceStart lb{std::numeric_limits<int>::max()};
    TimeSinceServiceStart ub{std::numeric_limits<int>::min()};
    for (const auto& [s, states] : step_states[k]) {
      assert(states.states.size() > 0);
      lb = std::min(lb, states.states.front().arrival_time);
      ub = std::max(lb, states.states.back().arrival_time);
    }
    return {lb, ub};
  }
};

std::pair<TimeSinceServiceStart, StepPartitionId> GetTNext(
    const StepsAdjacencyList& completed,
    const StepGroup& g_next,
    TimeSinceServiceStart t_cur
) {
  StepPartitionId part_next_flex = StepPartitionId::NONE;
  TimeSinceServiceStart t_next_flex{std::numeric_limits<int>::max()};
  // TODO: Handle duration>0 flex steps.
  // (It would be correct to simply delete the FlexDurationSections() == 0 here,
  // but that slows things down so much. I hope there is some optimization we
  // can do to handle them better.)
  if (g_next.flex_step.has_value() &&
      g_next.flex_step->FlexDurationSeconds() == 0) {
    part_next_flex = g_next.flex_step->destination_partition;
    t_next_flex.seconds =
        t_cur.seconds + g_next.flex_step->FlexDurationSeconds();
  }

  StepPartitionId part_next_sched = StepPartitionId::NONE;
  TimeSinceServiceStart t_next_sched{std::numeric_limits<int>::max()};
  std::span<const AdjacencyListStep> group_steps = completed.GetSteps(g_next);
  size_t t_next_i = FindDepartureAtOrAfter(completed, g_next, t_cur);
  if (t_next_i < group_steps.size()) {
    part_next_sched = group_steps[t_next_i].destination_partition;
    t_next_sched = group_steps[t_next_i].destination_time;
  }

  if (t_next_flex < t_next_sched) {
    return {t_next_flex, part_next_flex};
  } else {
    return {t_next_sched, part_next_sched};
  }
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
        auto [t_next, part_next] =
            GetTNext(completed, g_next, ats.arrival_time);
        if (t_next > t_ub) {
          continue;
        }
        step_state_cur.AddOnwards(
            ats, s_next, t_next.seconds - ats.arrival_time.seconds, part_next
        );
        if (k + 1 < state.step_states.size()) {
          auto& vec = state.step_states[k + 1][s_next].states;
          ArrivalTimeState entry{.arrival_time = t_next};
          auto it = std::lower_bound(
              vec.begin(), vec.end(), entry, [](const auto& a, const auto& b) {
                return a.arrival_time < b.arrival_time;
              }
          );
          if (it == vec.end() || it->arrival_time != t_next) {
            it = vec.insert(it, std::move(entry));
          }
          auto existing_partition_it = std::find_if(
              it->partitions.begin(),
              it->partitions.end(),
              [&](const StepPartitionId partition) {
                return partition == part_next;
              }
          );
          if (existing_partition_it == it->partitions.end()) {
            it->partitions.push_back(part_next);
          }
        }
      }
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
      for (OnwardsStep onwards : step_state_cur.OnwardsSteps(ats)) {
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
      for (size_t i = 0;
           i < ats.onwards.size() && i < step_state_cur.destinations.size();
           ++i) {
        if (ats.onwards[i].duration == -1) {
          continue;
        }
        if (!arrival_times[step_state_cur.destinations[i]].contains(
                TimeSinceServiceStart{
                    ats.arrival_time.seconds + ats.onwards[i].duration
                }
            )) {
          ats.onwards[i].duration = -1;
          ats.onwards[i].partition = StepPartitionId::NONE;
        }
      }
    }
    std::erase_if(
        step_state_cur.states, [&step_state_cur](const ArrivalTimeState& ats) {
          return !step_state_cur.HasAnyOnwards(ats);
        }
    );
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

  std::vector<std::vector<StopId>> result_forced_after;
  result_forced_after.reserve(state.forced_after.size());

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
      result_forced_after.push_back(std::move(state.forced_after[i]));
      continue;
    }

    // Ok we have a size==1 state and it's not at the beginning or the end.
    const auto& [forced_s, forced_state] = *state.step_states[i].begin();
    state.required.EraseGroup(forced_s);

    // So we want to modify the prev state to represent arriving at forced_s.
    auto& prev_forced_after = result_forced_after.back();
    prev_forced_after.reserve(1 + state.forced_after[i].size());
    prev_forced_after.push_back(forced_s);
    prev_forced_after.insert(
        prev_forced_after.end(),
        state.forced_after[i].begin(),
        state.forced_after[i].end()
    );
    for (auto& [prev_s, prev_state] : result_states.back()) {
      int forced_ats_i = 0;
      for (ArrivalTimeState& prev_ats : prev_state.states) {
        auto prev_onwards_view = prev_state.OnwardsSteps(prev_ats);
        auto prev_onwards_it = prev_onwards_view.begin();
        assert(prev_onwards_it != prev_onwards_view.end());
        OnwardsStep prev_onwards = *prev_onwards_it;
        ++prev_onwards_it;
        assert(prev_onwards_it == prev_onwards_view.end());
        assert(prev_onwards.destination == forced_s);
        prev_state.ClearOnwards(prev_ats);

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

        for (OnwardsStep forced_onwards :
             forced_state.OnwardsSteps(forced_state.states[forced_ats_i])) {
          prev_state.AddOnwards(
              prev_ats,
              forced_onwards.destination,
              prev_onwards.duration + forced_onwards.duration,
              forced_onwards.partition
          );
        }
      }
    }
  }

  state.step_states = std::move(result_states);
  state.forced_after = std::move(result_forced_after);
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
  state.required = required;

  // state.step_states[0]: Base case, START@t0.
  //
  // state.step_states[1]: s0@t0.
  //
  // state.step_states[k]: All possible states after making k steps (visited k+1
  // stops counting boundary stops).
  //
  // state.step_states[n-1]: END@{tf1, tf2, ..., tfx}.
  state.step_states.resize(required.size());
  state.forced_after.resize(required.size());

  state.step_states[0][boundary.start].states.push_back({.arrival_time = t0});
  for (int k = 0; k < state.step_states.size(); ++k) {
    PropagateStepStatesForwards(completed, t_ub, state, s0, k);
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

constexpr int kMaxStep = 10;

std::optional<TspTourResult> DoTSP(
    const StepsAdjacencyList& completed, const BnbState& state, int ub_rel
) {
  const auto& boundary = state.boundary;
  const auto& required = state.required;
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
    return StepPartitionId{0};
    // if (step <= kMaxStep || step >= n - kMaxStep) {
    // if (step <= kMaxStep) {
    //   return StepPartitionId{step};
    // } else {
    //   return StepPartitionId{kMaxStep};
    // }
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
        for (OnwardsStep onwards : step_state_cur.OnwardsSteps(ats)) {
          auto [it, inserted] = best_dur_by_dest.try_emplace(
              onwards.destination, onwards.duration
          );
          if (!inserted) {
            it->second = std::min(it->second, onwards.duration);
          }
        }
      }

      // Collect median duration per destination!!
      std::unordered_map<StopId, std::vector<int>> durs_by_dest;
      for (const ArrivalTimeState& ats : step_state_cur.states) {
        for (OnwardsStep onwards : step_state_cur.OnwardsSteps(ats)) {
          durs_by_dest[onwards.destination].push_back(onwards.duration);
        }
      }
      std::unordered_map<StopId, int> median_durs_by_dest;
      std::unordered_map<StopId, int> average_durs_by_dest;
      for (auto& [s, durs] : durs_by_dest) {
        std::ranges::sort(durs);
        median_durs_by_dest[s] = durs[4 * durs.size() / 9];
        // median_durs_by_dest[s] = durs[rand() % durs.size()];

        int total = 0;
        for (int d : durs) {
          total += d;
        }
        average_durs_by_dest[s] = total / durs.size();
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

  std::cout << "calling to tsp\n";
  std::optional<TspTourResult> result = SolveTspAndExtractTour(
      remap.edges,
      graph,
      boundary,
      std::nullopt,
      // ub_rel == -1 ? std::nullopt : std::optional(ub_rel),
      // &std::cout,
      nullptr,
      nullptr
  );
  if (!result.has_value()) {
    return std::nullopt;
  }
  std::cout << "done tsp\n";

  // Map `result` states back to original states.
  for (TarelEdge& edge : result->tour_edges) {
    edge.origin = remap.mapped_to_original.at(edge.origin);
    edge.destination = remap.mapped_to_original.at(edge.destination);
  }

  return result;
}

struct ArrivalTimeAndOnwards {
  TimeSinceServiceStart arrival_time;

  // The partitions that arrive at this arrival time.
  std::vector<StepPartitionId> partitions;

  int duration;

  StepPartitionId dest_partition;
};

struct TourAnalysisPart {
  StopId stop;
  std::string stop_name;
  TimeSinceServiceStart t_relaxed;
  TimeSinceServiceStart t_actual;
  std::vector<TimeSinceServiceStart> ts_used_for_next_relaxed_step;
  std::vector<ArrivalTimeAndOnwards> onwards;
  int weight_onwards;
  int actual_onwards;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ArrivalTimeAndOnwards, arrival_time, partitions, duration, dest_partition
)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    TourAnalysisPart,
    stop,
    stop_name,
    t_relaxed,
    t_actual,
    ts_used_for_next_relaxed_step,
    onwards,
    weight_onwards,
    actual_onwards
)

struct TourAnalysis {
  int first_btaat_k = -1;
  TimeSinceServiceStart first_btaat;
  TarelEdge first_btaat_edge;
  TimeSinceServiceStart first_btaat_good_a;
  TimeSinceServiceStart first_btaat_good_b;

  TimeSinceServiceStart t_relaxed;
  TimeSinceServiceStart t_actual;

  std::vector<TourAnalysisPart> parts;
};

// TODO NEXT: This whole stepwise data structure is not working at all for
// DeepExclusion. What I really want is for the stepwise structure itself to be
// all merged up together so that the analysis and the tour match better. So
// Friday I do that!

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

  // std::cout << "  " << result.tour_edges[0].origin.partition.v << ". "
  //           << problem.StopName(result.tour_edges[0].origin.stop) << " @ "
  //           << analysis.t_relaxed << " / " << analysis.t_actual << "\n";
  analysis.parts.emplace_back(
      result.tour_edges[0].origin.stop,
      problem.StopName(result.tour_edges[0].origin.stop),
      analysis.t_relaxed,
      analysis.t_actual,
      std::vector<TimeSinceServiceStart>(),
      std::vector<ArrivalTimeAndOnwards>(),
      result.tour_edges[0].weight,
      -1
  );
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
        for (OnwardsStep onwards : from_o_it->second.OnwardsSteps(ats)) {
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

    if (from_o_it != step_states[i].end()) {
      int min_dur = std::numeric_limits<int>::max();
      int max_dur = std::numeric_limits<int>::min();

      for (const ArrivalTimeState& ats : from_o_it->second.states) {
        for (OnwardsStep onwards : from_o_it->second.OnwardsSteps(ats)) {
          if (onwards.destination == edge.destination.stop) {
            min_dur = std::min(min_dur, onwards.duration);
            max_dur = std::max(max_dur, onwards.duration);
          }
        }
      }

      // Might not exist past kMaxStep!
      // TODO: Reallly?
      if (min_dur < std::numeric_limits<int>::max() &&
          max_dur > std::numeric_limits<int>::min()) {
        int mid_dur = min_dur / 2 + max_dur / 2;
        for (const ArrivalTimeState& ats : from_o_it->second.states) {
          for (OnwardsStep onwards : from_o_it->second.OnwardsSteps(ats)) {
            if (onwards.destination == edge.destination.stop &&
                onwards.duration <= min_dur + 5 * 60) {
              analysis.parts.back().ts_used_for_next_relaxed_step.push_back(
                  ats.arrival_time
              );
            }
          }
        }
      }

      for (const ArrivalTimeState& ats : from_o_it->second.states) {
        for (OnwardsStep onwards : from_o_it->second.OnwardsSteps(ats)) {
          if (onwards.destination == edge.destination.stop) {
            analysis.parts.back().onwards.push_back(
                {ats.arrival_time,
                 ats.partitions,
                 onwards.duration,
                 onwards.partition}
            );
            if (ats.arrival_time == analysis.parts.back().t_actual) {
              analysis.parts.back().actual_onwards = onwards.duration;
            }
          }
        }
      }
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
          for (OnwardsStep onwards : from_o_it->second.OnwardsSteps(ats)) {
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
    analysis.parts.emplace_back(
        edge.destination.stop,
        problem.StopName(edge.destination.stop),
        analysis.t_relaxed,
        analysis.t_actual,
        std::vector<TimeSinceServiceStart>(),
        std::vector<ArrivalTimeAndOnwards>(),
        i + 1 < result.tour_edges.size() ? result.tour_edges[i + 1].weight : 0,
        -1
    );
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

std::string ApproxBnbStateSizeMB(const BnbState& state) {
  size_t bytes = sizeof(BnbState);

  for (const auto& step_map : state.step_states) {
    bytes += sizeof(step_map);
    for (const auto& [stop, step_state] : step_map) {
      bytes += sizeof(stop) + sizeof(step_state);
      bytes += step_state.destinations.capacity() * sizeof(StopId);
      for (const ArrivalTimeState& ats : step_state.states) {
        bytes += sizeof(ats);
        bytes += ats.onwards.capacity() * sizeof(OnwardsDuration);
      }
      bytes += step_state.states.capacity() * sizeof(ArrivalTimeState);
    }
    // Rough estimate for unordered_map bucket overhead.
    bytes += step_map.bucket_count() * sizeof(void*);
  }

  for (const auto& forced : state.forced_after) {
    bytes += sizeof(forced) + forced.capacity() * sizeof(StopId);
  }

  bytes += state.required.representative.bucket_count() * sizeof(void*);
  bytes +=
      state.required.representative.size() * (sizeof(StopId) + sizeof(StopId));

  double mb = static_cast<double>(bytes) / (1024.0 * 1024.0);
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << mb << " MB";
  return oss.str();
}

struct RequireExactStopStepTime {
  StopId stop;
  int k;
  std::vector<TimeSinceServiceStart> times;

  std::string ToString(const ProblemState& problem) const {
    std::stringstream ss;
    ss << "Require " << problem.StopName(stop) << " @ " << k << ", [";
    if (times.size() <= 3) {
      for (int i = 0; i < times.size(); ++i) {
        if (i > 0) {
          ss << ", ";
        }
        ss << times[i];
      }
    } else {
      ss << times.front() << ", ..., " << times.back();
    }
    ss << "] (" << times.size() << ")";
    return ss.str();
  }

  void Apply(BnbState& state) const {
    std::erase_if(state.step_states[k], [&](const auto& pair) {
      return pair.first != stop;
    });
    assert(state.step_states[k].size() == 1);
    auto ss_it = state.step_states[k].find(stop);
    assert(ss_it != state.step_states[k].end());
    std::unordered_set<TimeSinceServiceStart> times_set(
        times.begin(), times.end()
    );
    std::erase_if(ss_it->second.states, [&](const ArrivalTimeState& ats) {
      return !times_set.contains(ats.arrival_time);
    });
    // TODO: This could instead be another pass that realizes that a certain
    // step has a single stop and then delete that stop from all the other
    // steps.
    for (int i = 0; i < state.step_states.size(); ++i) {
      if (i == k) {
        continue;
      }
      std::erase_if(state.step_states[i], [&](const auto& pair) {
        return pair.first == stop;
      });
    }
    FilterStepStatesSweep(state, k);
    // TODO: This could instead be another pass that realizes that a certain
    // step has a single stop and then delete that stop from all the other
    // steps.
    for (int i = 0; i < state.step_states.size(); ++i) {
      if (i == k) {
        continue;
      }
      std::erase_if(state.step_states[i], [&](const auto& pair) {
        return pair.first == stop;
      });
    }
    FilterStepStatesSweep(state, k);
  }
};

struct ForbidExactStopStepTime {
  StopId stop;
  int k;
  std::vector<TimeSinceServiceStart> times;

  std::string ToString(const ProblemState& problem) const {
    std::stringstream ss;
    ss << "Forbid " << problem.StopName(stop) << " @ " << k << ", [";
    if (times.size() <= 3) {
      for (int i = 0; i < times.size(); ++i) {
        if (i > 0) {
          ss << ", ";
        }
        ss << times[i];
      }
    } else {
      ss << times.front() << ", ..., " << times.back();
    }
    ss << "] (" << times.size() << ")";
    return ss.str();
  }

  void Apply(BnbState& state) const {
    std::unordered_set<TimeSinceServiceStart> times_set(
        times.begin(), times.end()
    );
    for (auto& [s, states] : state.step_states[k]) {
      if (s == stop) {
        std::erase_if(states.states, [&](const ArrivalTimeState& ats) {
          return times_set.contains(ats.arrival_time);
        });
      }
    }
    std::erase_if(state.step_states[k], [&](const auto& pair) {
      return pair.second.states.size() == 0;
    });
    FilterStepStatesSweep(state, k);
  }
};

struct ForbidStopTime {
  StopId stop;
  std::vector<TimeSinceServiceStart> times;

  std::string ToString(const ProblemState& problem) const {
    std::stringstream ss;
    ss << "Forbid " << problem.StopName(stop) << " @ [";
    if (times.size() <= 3) {
      for (int i = 0; i < times.size(); ++i) {
        if (i > 0) {
          ss << ", ";
        }
        ss << times[i];
      }
    } else {
      ss << times.front() << ", ..., " << times.back();
    }
    ss << "] (" << times.size() << ")";
    return ss.str();
  }

  void Apply(BnbState& state) const {
    std::unordered_set<TimeSinceServiceStart> times_set(
        times.begin(), times.end()
    );
    for (int k = 0; k < state.step_states.size(); ++k) {
      for (auto& [s, states] : state.step_states[k]) {
        if (s == stop) {
          std::erase_if(states.states, [&](const ArrivalTimeState& ats) {
            return times_set.contains(ats.arrival_time);
          });
        }
      }
      std::erase_if(state.step_states[k], [&](const auto& pair) {
        return pair.second.states.size() == 0;
      });
    }
    FilterStepStatesSweep(state, 0);
  }
};

struct RequireStopAtStep {
  StopId stop;
  int k;

  void Apply(BnbState& state) const {
    std::erase_if(state.step_states[k], [&](const auto& pair) {
      return pair.first != stop;
    });
    // TODO: This could instead be another pass that realizes that a certain
    // step has a single stop and then delete that stop from all the other
    // steps.
    for (int i = 0; i < state.step_states.size(); ++i) {
      if (i == k) {
        continue;
      }
      std::erase_if(state.step_states[i], [&](const auto& pair) {
        return pair.first == stop;
      });
    }
    FilterStepStatesSweep(state, k);
  }

  std::string ToString(const ProblemState& problem) const {
    std::stringstream ss;
    ss << "Require " << problem.StopName(stop) << " @ " << k;
    return ss.str();
  }
};

struct ForbidStopAtStep {
  StopId stop;
  int k;

  void Apply(BnbState& state) const {
    std::erase_if(state.step_states[k], [&](const auto& pair) {
      return pair.first == stop;
    });
    FilterStepStatesSweep(state, k);
  }

  std::string ToString(const ProblemState& problem) const {
    std::stringstream ss;
    ss << "Forbid " << problem.StopName(stop) << " @ " << k;
    return ss.str();
  }
};

struct TimeUpperBound {
  int k;
  TimeSinceServiceStart time;

  void Apply(BnbState& state) const {
    for (auto& [s, states] : state.step_states[k]) {
      std::erase_if(states.states, [&](const ArrivalTimeState& ats) {
        return ats.arrival_time >= time;
      });
    }
    FilterStepStatesSweep(state, k);
  }

  std::string ToString(const ProblemState& problem) const {
    std::stringstream ss;
    ss << "* @ " << k << " < " << time;
    return ss.str();
  }
};

struct TimeLowerBound {
  int k;
  TimeSinceServiceStart time;

  void Apply(BnbState& state) const {
    for (auto& [s, states] : state.step_states[k]) {
      std::erase_if(states.states, [&](const ArrivalTimeState& ats) {
        return ats.arrival_time < time;
      });
    }
    FilterStepStatesSweep(state, k);
  }

  std::string ToString(const ProblemState& problem) const {
    std::stringstream ss;
    ss << "* @ " << k << " >= " << time;
    return ss.str();
  }
};

struct StopTimeUpperBound {
  StopId stop;
  int k;
  TimeSinceServiceStart time;

  void Apply(BnbState& state) const {
    std::erase_if(
        state.step_states[k][stop].states,
        [&](const ArrivalTimeState& ats) { return ats.arrival_time > time; }
    );
    FilterStepStatesSweep(state, k);
  }

  std::string ToString(const ProblemState& problem) const {
    std::stringstream ss;
    ss << problem.StopName(stop) << " @ " << k << " <= " << time;
    return ss.str();
  }
};

struct StopTimeRange {
  StopId stop;
  int k;
  TimeSinceServiceStart a;
  TimeSinceServiceStart b;

  void Apply(BnbState& state) const {
    std::erase_if(
        state.step_states[k][stop].states, [&](const ArrivalTimeState& ats) {
          return ats.arrival_time <= a || ats.arrival_time >= b;
        }
    );
    FilterStepStatesSweep(state, k);
  }

  std::string ToString(const ProblemState& problem) const {
    std::stringstream ss;
    ss << problem.StopName(stop) << " @ " << k << " (" << a << ", " << b << ")";
    return ss.str();
  }
};

struct StopTimeLowerBound {
  StopId stop;
  int k;
  TimeSinceServiceStart time;

  void Apply(BnbState& state) const {
    std::erase_if(
        state.step_states[k][stop].states,
        [&](const ArrivalTimeState& ats) { return ats.arrival_time < time; }
    );
    FilterStepStatesSweep(state, k);
  }

  std::string ToString(const ProblemState& problem) const {
    std::stringstream ss;
    ss << problem.StopName(stop) << " @ " << k << " >= " << time;
    return ss.str();
  }
};

using BnbConstraint = std::variant<
    RequireExactStopStepTime,
    ForbidExactStopStepTime,
    RequireStopAtStep,
    ForbidStopAtStep,
    TimeUpperBound,
    TimeLowerBound,
    StopTimeUpperBound,
    StopTimeRange,
    StopTimeLowerBound>;

struct BnbNode {
  int lb;
  std::vector<BnbConstraint> constraints;
  BnbState state;
  bool operator<(const BnbNode& other) const { return lb > other.lb; }
};

void FilterStepBounds(BnbState& state, int k, int lb_rel, int ub_rel) {
  TimeSinceServiceStart t0 = state.T0();

  for (auto& [s, step_state] : state.step_states[k]) {
    std::erase_if(step_state.states, [&](const ArrivalTimeState& ats) {
      return (
          ats.arrival_time < TimeSinceServiceStart{t0.seconds + lb_rel} ||
          ats.arrival_time > TimeSinceServiceStart{t0.seconds + ub_rel}
      );
    });
  }
  FilterStepStatesSweep(state, k);
}

void FilterLastStepBounds(BnbState& state, int lb_rel, int ub_rel) {
  FilterStepBounds(state, state.step_states.size() - 1, lb_rel, ub_rel);
}

void PrintStateSummary(
    const ProblemState& problem, const BnbState& state, int show_count
) {
  for (int k = 0; k + 1 <= state.step_states.size(); ++k) {
    if (k <= show_count || k + show_count >= state.step_states.size()) {
      TimeSinceServiceStart min_at{std::numeric_limits<int>::max()},
          max_at{std::numeric_limits<int>::min()};
      for (const auto& [s, states] : state.step_states[k]) {
        for (const auto& ats : states.states) {
          min_at = std::min(ats.arrival_time, min_at);
          max_at = std::max(ats.arrival_time, max_at);
        }
      }

      std::cout << "  + " << k << ": ";
      if (state.step_states[k].size() <= 3) {
        int print_count = 0;
        for (const auto& [s, _] : state.step_states[k]) {
          if (print_count > 0) {
            std::cout << " | ";
          }
          print_count += 1;
          std::cout << problem.StopName(s);
        }
      } else {
        std::cout << state.step_states[k].size();
      }
      std::cout << " [";
      if (min_at == max_at) {
        std::cout << min_at;
      } else {
        std::cout << min_at << ", " << max_at;
      }
      std::cout << "]";
      for (StopId fa : state.forced_after[k]) {
        std::cout << " -> " << problem.StopName(fa);
      }
      std::cout << "\n";
    } else if (k == show_count + 1) {
      std::cout << "  + ...\n";
    }
  }
}

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
      [&](int lb, std::vector<BnbConstraint> constraints, BnbState state) {
        std::cout << "  PushQ: " << ApproxBnbStateSizeMB(state) << "\n";
        q.emplace_back(lb, std::move(constraints), std::move(state));
        std::push_heap(q.begin(), q.end());
      };

  PushQ(
      lb_rel,
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

    std::pop_heap(q.begin(), q.end());
    BnbNode cur = std::move(q.back());
    q.pop_back();

    FilterLastStepBounds(cur.state, cur.lb, best_ub);
    CombineForcedSteps(cur.state);

    std::cout << "iter " << iter_count << ": take "
              << TimeSinceServiceStart{cur.lb} << " (" << (q.size() + 1)
              << " active)\n";
    for (const BnbConstraint& constraint : cur.constraints) {
      std::cout << "  - "
                << std::visit(
                       [&](const auto& c) { return c.ToString(problem); },
                       constraint
                   )
                << "\n";
    }
    PrintStateSummary(problem, cur.state, kMaxStep);

    if (cur.lb > best_ub) {
      std::cout << "  terminated: smallest lb >= best_ub\n";
      return best_ub;
    }

    std::optional<TspTourResult> result = DoTSP(completed, cur.state, ub_rel);
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
    // assert(analysis.first_btaat_k > 0);

    int first_mismatch_k = 0;
    while (first_mismatch_k < analysis.parts.size() &&
           analysis.parts[first_mismatch_k].t_actual ==
               analysis.parts[first_mismatch_k].t_relaxed) {
      first_mismatch_k += 1;
    }
    assert(first_mismatch_k > 1);
    assert(first_mismatch_k < analysis.parts.size());

    // int biggest_mismatch = 0, biggest_mismatch_k;
    // for (int k = 0; k < kMaxStep && k < analysis.parts.size(); ++k) { //
    // TODO: off by 1?
    //   if (analysis.parts[k].mismatch_increase > biggest_mismatch) {
    //     biggest_mismatch = analysis.parts[k].mismatch_increase;
    //     biggest_mismatch_k = k;
    //   }
    // }
    // assert(biggest_mismatch_k > 1);

    // Either the solution gets to the pre-mismatch step at a time when it can
    // actually acomplish the relaxed dur, or it does not.
    int branch_k = analysis.parts.size() - 3;
    StopId branch_stop = analysis.parts[branch_k].stop;

    // TODO: Off by 1?
    if ((branch_k < kMaxStep || branch_k > analysis.parts.size() - kMaxStep) &&
        analysis.parts[branch_k].ts_used_for_next_relaxed_step.size() > 0) {
      {
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        BnbState branch_state = cur.state;
        RequireExactStopStepTime constraint{
            branch_stop,
            branch_k,
            analysis.parts[branch_k].ts_used_for_next_relaxed_step
        };
        branch_constraints.push_back(constraint);
        constraint.Apply(branch_state);
        PushQ(
            std::max(
                result->optimal_value,
                branch_state.step_states.back()
                        .begin()
                        ->second.states.front()
                        .arrival_time.seconds -
                    t0.seconds
            ),
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }
      {
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        BnbState branch_state = cur.state;
        ForbidExactStopStepTime constraint{
            branch_stop,
            branch_k,
            analysis.parts[branch_k].ts_used_for_next_relaxed_step
        };
        branch_constraints.push_back(constraint);
        constraint.Apply(branch_state);
        PushQ(
            std::max(
                result->optimal_value,
                branch_state.step_states.back()
                        .begin()
                        ->second.states.front()
                        .arrival_time.seconds -
                    t0.seconds
            ),
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }
      continue;
    }

    // Everything is good up to kMaxStep -- start restricting the initial stop
    // so that we can push kMaxStep farther.
    assert(analysis.parts.size() > 1);

    // int branch_k = (cur.state.forced_after[0].size() +
    // cur.state.forced_after[cur.state.forced_after.size() - 2].size()) % 2 ==
    // 0 ? 1 : analysis.parts.size() - 2;

    int branch_stop_k = 1;

    // Branch: Require s@1.
    {
      std::vector<BnbConstraint> branch_constraints = cur.constraints;
      BnbState branch_state = cur.state;
      RequireStopAtStep constraint{
          analysis.parts[branch_stop_k].stop, branch_stop_k
      };
      branch_constraints.push_back(constraint);
      constraint.Apply(branch_state);
      PushQ(
          result->optimal_value,
          std::move(branch_constraints),
          std::move(branch_state)
      );
    }

    // Branch: Forbid s@1.
    {
      std::vector<BnbConstraint> branch_constraints = cur.constraints;
      BnbState branch_state = cur.state;
      ForbidStopAtStep constraint{
          analysis.parts[branch_stop_k].stop, branch_stop_k
      };
      branch_constraints.push_back(constraint);
      constraint.Apply(branch_state);
      PushQ(
          result->optimal_value,
          std::move(branch_constraints),
          std::move(branch_state)
      );
    }

    continue;

    int small_cardinality_k = 1;
    while (small_cardinality_k + 1 < cur.state.step_states.size() &&
           cur.state.step_states[small_cardinality_k].size() > 5) {
      small_cardinality_k += 1;
    }

    // Branch on the small cardinality step if there is one.
    if (small_cardinality_k + 1 < cur.state.step_states.size()) {
      StopId constraint_s = result->tour_edges[small_cardinality_k].origin.stop;

      // Branch: Require s@k.
      {
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        branch_constraints.push_back(
            RequireStopAtStep{constraint_s, small_cardinality_k}
        );
        BnbState branch_state = cur.state;
        RequireStopAtStep{constraint_s, small_cardinality_k}.Apply(
            branch_state
        );
        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }

      // Branch: Forbid s@k.
      {
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        branch_constraints.push_back(
            ForbidStopAtStep{constraint_s, small_cardinality_k}
        );
        BnbState branch_state = cur.state;
        ForbidStopAtStep{constraint_s, small_cardinality_k}.Apply(branch_state);
        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }
      continue;
    }

    // Branch halfway between possible times of a step.
    {
      // int branch_k = cur.state.step_states.size() - 1;
      // while (branch_k > 0) {
      int branch_k = 0;
      while (branch_k < cur.state.step_states.size()) {
        bool has_onwards_disagreement = false;
        for (const auto& [s, states] : cur.state.step_states[branch_k]) {
          std::unordered_map<StopId, int> onwards_dur;
          for (const ArrivalTimeState& ats : states.states) {
            for (OnwardsStep onwards : states.OnwardsSteps(ats)) {
              auto [it, inserted] = onwards_dur.try_emplace(
                  onwards.destination, onwards.duration
              );
              if (it->second != onwards.duration) {
                has_onwards_disagreement = true;
                break;
              }
            }
            if (has_onwards_disagreement) {
              break;
            }
          }
          if (has_onwards_disagreement) {
            break;
          }
        }
        if (has_onwards_disagreement) {
          break;
        }
        branch_k += 1;
        // branch_k -= 1;
      }
      // assert(branch_k > 0);
      assert(branch_k < cur.state.step_states.size());

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
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        branch_constraints.push_back(TimeUpperBound{branch_k, branch_at});

        BnbState branch_state = cur.state;
        TimeUpperBound{branch_k, branch_at}.Apply(branch_state);

        TimeSinceServiceStart min_last_at = branch_state.step_states.back()
                                                .begin()
                                                ->second.states.front()
                                                .arrival_time;
        int min_last_at_lb = min_last_at.seconds - t0.seconds;

        PushQ(
            std::max(cur.lb, std::max(result->optimal_value, min_last_at_lb)),
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }

      // Branch: t >= branch_at.
      {
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        branch_constraints.push_back(TimeLowerBound{branch_k, branch_at});

        BnbState branch_state = cur.state;
        TimeLowerBound{branch_k, branch_at}.Apply(branch_state);

        TimeSinceServiceStart min_last_at = branch_state.step_states.back()
                                                .begin()
                                                ->second.states.front()
                                                .arrival_time;
        int min_last_at_lb = min_last_at.seconds - t0.seconds;

        PushQ(
            std::max(cur.lb, std::max(result->optimal_value, min_last_at_lb)),
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }
    }
    continue;

    // TODO: Think about whether this is an appropriate condition for when we
    // need to require/forbid s@k.
    if (
        false
        // first_unknown_stop_cardinality < 5 || analysis.first_btaat_k == -1 ||
        // analysis.first_btaat_k >=
        //     first_unknown_stop_k + kMaxStep  // TODO: Off-by-1?
    ) {
      // Branches: Require/forbid s@k.
      int constraint_k = 1;
      StopId constraint_s = result->tour_edges[constraint_k].destination.stop;

      // Branch: Require s@k.
      {
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        branch_constraints.push_back(
            RequireStopAtStep{constraint_s, constraint_k}
        );
        BnbState branch_state = cur.state;
        RequireStopAtStep{constraint_s, constraint_k}.Apply(branch_state);
        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }

      // Branch: Forbid s@k.
      {
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        branch_constraints.push_back(
            ForbidStopAtStep{constraint_s, constraint_k}
        );
        BnbState branch_state = cur.state;
        ForbidStopAtStep{constraint_s, constraint_k}.Apply(branch_state);
        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }
    } else {
      StopId constraint_s = analysis.first_btaat_edge.origin.stop;
      int constraint_k = analysis.first_btaat_k;

      // Branch: Forbid the stop.
      {
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        branch_constraints.push_back(
            ForbidStopAtStep{constraint_s, constraint_k}
        );
        BnbState branch_state = cur.state;
        ForbidStopAtStep{constraint_s, constraint_k}.Apply(branch_state);
        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }

      BnbState require_state = cur.state;
      RequireStopAtStep{constraint_s, constraint_k}.Apply(require_state);

      // Branch: First BAD region, and require the stop.
      // TODO: Think harder about whether I can omit branch for BAD region in
      // the -inf case.
      if (analysis.first_btaat_good_a.seconds !=
          std::numeric_limits<int>::min()) {
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        StopTimeUpperBound stop_constraint{
            analysis.first_btaat_edge.origin.stop,
            analysis.first_btaat_k,
            analysis.first_btaat_good_a
        };
        branch_constraints.push_back(stop_constraint);

        BnbState branch_state = require_state;
        stop_constraint.Apply(branch_state);

        PushQ(
            result->optimal_value,
            std::move(branch_constraints),
            std::move(branch_state)
        );
      }

      // Branch: GOOD region, and require the stop.
      {
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        StopTimeRange range_constraint{
            analysis.first_btaat_edge.origin.stop,
            analysis.first_btaat_k,
            analysis.first_btaat_good_a,
            analysis.first_btaat_good_b
        };
        branch_constraints.push_back(range_constraint);

        BnbState branch_state = require_state;
        range_constraint.Apply(branch_state);

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
        std::vector<BnbConstraint> branch_constraints = cur.constraints;
        StopTimeLowerBound stop_constraint{
            analysis.first_btaat_edge.origin.stop,
            analysis.first_btaat_k,
            analysis.first_btaat_good_b
        };
        branch_constraints.push_back(stop_constraint);

        BnbState branch_state = require_state;
        stop_constraint.Apply(branch_state);

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

void DoTimeSplit(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    StopId s0,
    TimeSinceServiceStart t0,
    int lb_rel,
    int ub_rel
) {
  BnbState base_state = ComputeStepStates(
      completed, problem.required, problem.boundary, s0, t0, lb_rel, ub_rel
  );
  FilterLastStepBounds(base_state, lb_rel, ub_rel);
  CombineForcedSteps(base_state);
  PrintStateSummary(problem, base_state, 10);

  std::cout << "Initial: " << TimeSinceServiceStart{lb_rel} << " / "
            << TimeSinceServiceStart{ub_rel} << "\n";

  // std::optional<TspTourResult> base_result = DoTSP(completed, base_state,
  // ub_rel); assert(base_result.has_value()); TourAnalysis base_analysis =
  // AnalyzeTour(problem, completed, base_state, *base_result, t0); std::cout
  //   << "Base: " << TimeSinceServiceStart{base_result->optimal_value} << " / "
  //   << TimeSinceServiceStart{base_analysis.t_actual.seconds - t0.seconds} <<
  //   "\n";

  // lb_rel = base_result->optimal_value;
  // ub_rel = base_analysis.t_actual.seconds - t0.seconds;
  // FilterLastStepBounds(base_state, lb_rel, ub_rel);
  // CombineForcedSteps(base_state);
  // PrintStateSummary(problem, base_state, 10);

  // base_result = DoTSP(completed, base_state, ub_rel);
  // assert(base_result.has_value());
  // base_analysis = AnalyzeTour(problem, completed, base_state, *base_result,
  // t0); std::cout
  //   << "Base 2: " << TimeSinceServiceStart{base_result->optimal_value} << " /
  //   "
  //   << TimeSinceServiceStart{base_analysis.t_actual.seconds - t0.seconds} <<
  //   "\n";

  for (int k_chop = base_state.step_states.size() - 1; k_chop >= 0; --k_chop) {
    // Binary search to find the greatest t such that if we chop base_state@k<t
    // the problem is infeasible.
    auto [k_chop_lo, k_chop_hi] = base_state.StepBounds(k_chop);
    TimeSinceServiceStart lo = k_chop_lo;
    TimeSinceServiceStart hi = k_chop_hi;

    while (lo.seconds < hi.seconds - 60) {
      std::cout << "Searching at " << k_chop << ": " << lo << " to " << hi
                << "\n";
      int mid = (lo.seconds + hi.seconds) / 2;

      // Create a "chopped state" where the k_chop step arrives < mid.
      BnbState chopped_state = base_state;
      FilterStepBounds(
          chopped_state,
          k_chop,
          k_chop_lo.seconds - t0.seconds,
          mid - 1 - t0.seconds
      );
      CombineForcedSteps(chopped_state);
      auto [chopped_last_lo, chopped_last_hi] =
          chopped_state.StepBounds(chopped_state.step_states.size() - 1);

      std::optional<TspTourResult> chopped_result =
          DoTSP(completed, chopped_state, ub_rel);
      if (!chopped_result.has_value() ||
          t0.seconds + chopped_result->optimal_value >
              chopped_last_hi.seconds) {
        // Problem infeasible!
        lo = TimeSinceServiceStart{mid};
      } else {
        // Problem feasible!
        hi = TimeSinceServiceStart{mid};
      }
    }
    std::cout << "Result for " << k_chop << ": " << lo << "\n";
    FilterStepBounds(
        base_state,
        k_chop,
        lo.seconds - t0.seconds,
        k_chop_hi.seconds - t0.seconds
    );
    PrintStateSummary(problem, base_state, 10);

    std::optional<TspTourResult> base_result =
        DoTSP(completed, base_state, ub_rel);
    assert(base_result.has_value());
    TourAnalysis base_analysis =
        AnalyzeTour(problem, completed, base_state, *base_result, t0);
    std::cout
        << "Base: " << TimeSinceServiceStart{base_result->optimal_value}
        << " / "
        << TimeSinceServiceStart{base_analysis.t_actual.seconds - t0.seconds}
        << "\n";
  }

  // std::cout << "k_chop_ " << k_chop_lb_t << " " << k_chop_ub_t << "\n";
  // int k_chop_lb = k_chop_lb_t.seconds - t0.seconds;
  // int k_chop_ub = k_chop_ub_t.seconds - t0.seconds;
  // PrintStateSummary(problem, chopped_state, 10);

  // std::optional<TspTourResult> chopped_result = DoTSP(completed,
  // chopped_state, ub_rel); assert(chopped_result.has_value()); TourAnalysis
  // chopped_analysis = AnalyzeTour(problem, completed, chopped_state,
  // *chopped_result, t0); std::cout
  //   << "chopped: " << TimeSinceServiceStart{chopped_result->optimal_value} <<
  //   " / "
  //   << TimeSinceServiceStart{chopped_analysis.t_actual.seconds - t0.seconds}
  //   << "\n";
}

void DeepExclusion(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    StopId s0,
    TimeSinceServiceStart t0,
    int lb_rel,
    int ub_rel
) {
  BnbState state = ComputeStepStates(
      completed, problem.required, problem.boundary, s0, t0, lb_rel, ub_rel
  );
  FilterLastStepBounds(state, lb_rel, ub_rel);
  CombineForcedSteps(state);
  PrintStateSummary(problem, state, 10);

  std::cout << "Initial: " << TimeSinceServiceStart{lb_rel} << " / "
            << TimeSinceServiceStart{ub_rel} << "\n";

  for (int iter = 0; iter < 1000; ++iter) {
    std::optional<TspTourResult> result = DoTSP(completed, state, ub_rel);
    if (!result.has_value()) {
      std::cout << "no result";
      return;
    }
    TourAnalysis analysis = AnalyzeTour(problem, completed, state, *result, t0);

    int total_discrepancy = 0;
    int worst_discrepancy = 0;
    int worst_discrepancy_i = -1;
    for (int i = 0; i < analysis.parts.size(); ++i) {
      const auto& part = analysis.parts[i];
      std::cout << i << ". " << problem.StopName(part.stop) << ": "
                << part.t_relaxed << " / " << part.t_actual << "\n";
      if (part.actual_onwards > 0) {
        int discrepancy = part.actual_onwards - part.weight_onwards;
        std::cout << "  discrepancy " << TimeSinceServiceStart{discrepancy}
                  << "\n";
        assert(discrepancy >= 0);

        // TODO: Figure out when this branch can happen even if
        // total_discrepancy == inf.
        if (total_discrepancy < std::numeric_limits<int>::max()) {
          total_discrepancy += discrepancy;
        }

        if (discrepancy > worst_discrepancy) {
          worst_discrepancy = discrepancy;
          worst_discrepancy_i = i;
        }
      } else {
        std::cout << "  no more actual path\n";
        total_discrepancy = std::numeric_limits<int>::max();

        if (worst_discrepancy < std::numeric_limits<int>::max()) {
          worst_discrepancy = std::numeric_limits<int>::max();
          worst_discrepancy_i = i;
        }
      }
    }
    if (total_discrepancy < std::numeric_limits<int>::max()) {
      assert(
          total_discrepancy ==
          analysis.t_actual.seconds - analysis.t_relaxed.seconds
      );
    } else {
      assert(analysis.t_actual.seconds == std::numeric_limits<int>::max());
    }

    if (worst_discrepancy_i == -1) {
      std::cout << "all done\n";
      break;
    }

    const TourAnalysisPart& disc_part = analysis.parts[worst_discrepancy_i];
    ForbidStopTime constraint{disc_part.stop, {disc_part.t_actual}};
    std::cout << "applying " << constraint.ToString(problem) << "\n";
    constraint.Apply(state);
  }
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

  // auto PrintTsp = [&](const BnbState& state) {
  //   std::optional<TspTourResult> result = DoTSP(completed, state, ub_rel);
  //   if (!result.has_value()) {
  //     std::cout << "no result";
  //     return;
  //   }
  //   TourAnalysis analysis =
  //       AnalyzeTour(problem, completed, state, *result, departure_times[0]);
  //   std::cout
  //     << "result: " << TimeSinceServiceStart{result->optimal_value}
  //     << " / " << TimeSinceServiceStart{analysis.t_actual.seconds -
  //     departure_times[0].seconds}
  //     << "\n";

  //   nlohmann::json j;
  //   j["parts"] = analysis.parts;
  //   nlohmann::json pnames = nlohmann::json::object();
  //   for (const auto& [id, name] : problem.step_partition_names) {
  //     pnames[std::to_string(id.v)] = name;
  //   }
  //   j["partition_names"] = pnames;
  //   std::ofstream out("/tmp/tour_analysis_parts.json");
  //   out << j.dump();
  //   out.close();
  //   std::cout << "Saved tour analysis parts to
  //   /tmp/tour_analysis_parts.json\n";
  // };

  // auto state = ComputeStepStates(
  //     completed, problem.required, problem.boundary, stop,
  //     departure_times[0], lb_rel, ub_rel
  // );
  // FilterLastStepBounds(state, lb_rel, ub_rel);
  // CombineForcedSteps(state);
  // PrintStateSummary(problem, state, 10);
  // PrintTsp(state);
  // return 0;

  // std::cout << "BASE\n";
  // PrintTsp(step_states_base);

  // std::cout << "Forbid Candidate\n";
  // PrintTsp(ForbidStopStep(step_states_base, 2, cand_next));

  // return 0;

  // DoTimeSplit(problem, completed, stop, departure_times[0], lb_rel, ub_rel);
  DeepExclusion(problem, completed, stop, departure_times[0], lb_rel, ub_rel);
  return 0;

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
