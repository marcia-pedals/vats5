#include <crow/http_parser_merged.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <asio/any_completion_handler.hpp>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

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

struct PairStep {
  TimeSinceServiceStart origin_time;
  int duration;
  TimeSinceServiceStart DestinationTime() const {
    return TimeSinceServiceStart{origin_time.seconds + duration};
  }
};

struct PairSteps {
  std::vector<PairStep> steps;
  int local_embiggening = 0;

  int RelaxedWeight() const {
    auto min_it = std::ranges::min_element(steps, {}, [](const PairStep& ps) {
      return ps.duration;
    });
    assert(min_it != steps.end());
    return min_it->duration + local_embiggening;
  }

  std::vector<int> PercentileDurs(const std::vector<int>& percentiles) const {
    assert(steps.size() > 0);

    std::vector<int> durations;
    durations.reserve(steps.size());
    for (const PairStep& ps : steps) {
      durations.push_back(ps.duration);
    }
    std::ranges::sort(durations);

    std::vector<int> result;
    result.reserve(percentiles.size());
    for (int p : percentiles) {
      result.push_back(durations[p * (durations.size() - 1) / 100]);
    }
    return result;
  }
};

struct RefinerState {
  std::vector<StopId> stops;
  std::unordered_map<std::pair<StopId, StopId>, PairSteps> pairs;
};

RefinerState MakeRefinerState(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    StopId s0,
    TimeSinceServiceStart t0,
    int ub_rel
) {
  TimeSinceServiceStart t_ub{t0.seconds + ub_rel};

  auto required_stop_reps = problem.required.GroupRepresentatives();

  RefinerState result;
  result.stops =
      std::vector<StopId>(required_stop_reps.begin(), required_stop_reps.end());
  result.pairs.reserve(problem.required.size() * problem.required.size());

  std::unordered_set<std::pair<StopId, TimeSinceServiceStart>> visited;

  std::vector<std::pair<StopId, TimeSinceServiceStart>> stack;
  stack.push_back({problem.boundary.start, t0});

  while (stack.size() > 0) {
    auto [s, t] = stack.back();
    stack.pop_back();

    if (visited.contains({s, t})) {
      continue;
    }
    visited.insert({s, t});

    for (const StepGroup& g_next : completed.GetGroups(s)) {
      StopId s_next = g_next.destination_stop;

      if (
        // START must go to s0.
        (s == problem.boundary.start && s_next != s0) ||
        // Never go back to s0.
        (s != problem.boundary.start && s_next == s0) ||
        // END may not go anywhere.
        (s == problem.boundary.end) ||
        // Don't go to current stop.
        (s == s_next)
      ) {
        continue;
      }
      auto [t_next, _] = GetTNext(completed, g_next, t);
      if (t_next > t_ub) {
        continue;
      }
      result.pairs[{s, s_next}].steps.push_back(
          {t, t_next.seconds - t.seconds}
      );
      stack.push_back({s_next, t_next});
    }
  }

  for (auto& [_, pair] : result.pairs) {
    std::ranges::sort(pair.steps, {}, [](const PairStep& ps) {
      return ps.origin_time;
    });
    for (size_t i = 0; i + 1 < pair.steps.size(); ++i) {
      assert(pair.steps[i].origin_time != pair.steps[i + 1].origin_time);
    }
  }

  return result;
}

std::optional<TspTourResult> DoTSP(
    const ProblemState& problem, const RefinerState& state, int ub_rel
) {
  TarelState start_state{problem.boundary.start, StepPartitionId{0}};
  TarelState end_state{problem.boundary.end, StepPartitionId{0}};
  std::vector<TarelEdge> edges;

  // END->START edge.
  edges.push_back({
      .origin = end_state,
      .destination = start_state,
      .weight = 0,
  });

  // All the other edges.
  for (const auto& [pair, pair_steps] : state.pairs) {
    // Percentile!
    // std::vector<int> durations;
    // durations.reserve(pair_steps.steps.size());
    // for (const PairStep& ps : pair_steps.steps) {
    //   durations.push_back(ps.duration);
    // }
    // std::ranges::sort(durations);
    // edges.push_back({
    //   .origin = TarelState{pair.first, StepPartitionId{0}},
    //   .destination = TarelState{pair.second, StepPartitionId{0}},
    //   .weight = durations[durations.size() / 10],
    // });

    // Actual lower bound!

    if (pair_steps.steps.size() == 0) {
      // std::cout << "No steps: " << problem.StopName(pair.first) << " -> " <<
      // problem.StopName(pair.second) << "\n";
      continue;
    }

    edges.push_back({
        .origin = TarelState{pair.first, StepPartitionId{0}},
        .destination = TarelState{pair.second, StepPartitionId{0}},
        .weight = pair_steps.RelaxedWeight(),
    });
  }

  // SOLVE!!!
  TarelStateRemapResult remap = RemapTarelStates(edges, problem.required);
  TspGraphData graph = MakeTspGraphEdges(remap.edges, problem.boundary);

  // Check that at least one representative from each group of required stops
  // appears in `graph`.
  //
  // This is necessary for correctness because the above construction can omit
  // stops from `graph`, and if it does, then the TSP on `graph` will give a
  // solution that does not reach all the stops. (Specifically, stops that don't
  // appear as both origins and destinations are omitted).
  std::unordered_set<StopId> representatives_in_graph;
  for (const TarelState& tarel_state : graph.state_by_id) {
    representatives_in_graph.insert(
        problem.required.Representative(tarel_state.stop)
    );
  }
  for (StopId rep : problem.required.GroupRepresentatives()) {
    if (!representatives_in_graph.contains(rep)) {
      std::cout << "  missing required, no solution\n";
      return std::nullopt;
    }
  }

  std::optional<TspTourResult> result = SolveTspAndExtractTour(
      remap.edges,
      graph,
      problem.boundary,
      ub_rel,
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
  std::optional<TarelEdge> worst_edge;
  int val_relaxed;
  int val_actual;
};

TourAnalysis AnalyzeTour(
    const ProblemState& problem,
    const RefinerState& state,
    const TspTourResult& result,
    TimeSinceServiceStart t0,
    bool print
) {
  TimeSinceServiceStart t_actual = t0;
  TimeSinceServiceStart t_relaxed = t0;
  StopId s = problem.boundary.start;

  auto EmitCur = [&]() {
    if (print) {
      std::cout << problem.StopName(s) << " @ " << t_relaxed << " / "
                << t_actual << "\n";
    }
  };

  int worst_pct_diff = 0;
  std::optional<TarelEdge> worst_edge;

  EmitCur();
  for (const TarelEdge& edge : result.tour_edges) {
    assert(edge.origin.stop == s);
    const PairSteps& pair_steps =
        state.pairs.at({edge.origin.stop, edge.destination.stop});

    // std::vector<int> pcts{0, 10, 50, 100};
    // std::vector<int> pcts_durs = pair_steps.PercentileDurs(pcts);
    // for (int i = 0; i < pcts.size(); ++i) {
    //   std::cout << "  p" << pcts[i] << ": " <<
    //   TimeSinceServiceStart{pcts_durs[i]} << "\n";
    // }

    std::vector<int> pcts_durs = pair_steps.PercentileDurs({0, 50});
    int pct_diff = pcts_durs[1] - pcts_durs[0];
    if (print) {
      std::cout << "  p50-p0: " << TimeSinceServiceStart{pct_diff} << "\n";
    }

    auto step_it = std::find_if(
        pair_steps.steps.begin(),
        pair_steps.steps.end(),
        [&](const PairStep& ps) { return ps.origin_time == t_actual; }
    );
    int actual_diff = std::numeric_limits<int>::max();
    if (step_it == pair_steps.steps.end()) {
      t_actual.seconds = std::numeric_limits<int>::max();
    } else {
      t_actual = step_it->DestinationTime();
      actual_diff = step_it->duration - pcts_durs[0];
    }
    if (print) {
      std::cout << "  actual-p0: " << TimeSinceServiceStart{actual_diff}
                << "\n";
    }

    if (actual_diff > 0 && pct_diff > worst_pct_diff) {
      worst_pct_diff = pct_diff;
      worst_edge = edge;
    }

    s = edge.destination.stop;
    t_relaxed.seconds += edge.weight;
    EmitCur();
  }

  return TourAnalysis{
      .worst_edge = worst_edge,
      .val_relaxed = t_relaxed.seconds - t0.seconds,
      .val_actual = t_actual.seconds - t0.seconds,
  };
}

void ForbidDurationBetween(
    RefinerState& state, StopId a, StopId b, int duration
) {
  std::unordered_set<TimeSinceServiceStart> a_times_to_forbid;
  for (const PairStep& step : state.pairs.at({a, b}).steps) {
    if (step.duration == duration) {
      a_times_to_forbid.insert(step.origin_time);
    }
  }

  for (auto& [pair, pair_states] : state.pairs) {
    if (pair.first == a) {
      std::erase_if(pair_states.steps, [&](const PairStep& ps) {
        return a_times_to_forbid.contains(ps.origin_time);
      });
    }
    if (pair.second == a) {
      std::erase_if(pair_states.steps, [&](const PairStep& ps) {
        return a_times_to_forbid.contains(ps.DestinationTime());
      });
    }
  }
}

struct EmbiggenState {
  int allowed_embiggen = 0;
  int path_relaxed_weight = 0;
  std::vector<StopId> path;
  std::vector<PairStep> steps;

  // a_origin_times[i] is the time that steps[i] gets to (and departs -- these
  // are the same because of how we do things) "a".
  //
  // Note that duplicates are quite expected because multiple actual times
  // starting at path[0] may be forced to wait for the same departure from a
  // when they do finally get to a.
  std::vector<TimeSinceServiceStart> a_origin_times;

  bool operator<(const EmbiggenState& other) const {
    return allowed_embiggen > other.allowed_embiggen;
  }

  int MinDurI() const {
    int min_dur_i = -1;
    for (int i = 0; i < steps.size(); ++i) {
      if (min_dur_i == -1 || steps[i].duration < steps[min_dur_i].duration) {
        min_dur_i = i;
      }
    }
    return min_dur_i;
  }

  int MinDurIWithoutAOT(TimeSinceServiceStart aot) const {
    int min_dur_i = -1;
    for (int i = 0; i < steps.size(); ++i) {
      if (a_origin_times[i] == aot) {
        continue;
      }
      if (min_dur_i == -1 || steps[i].duration < steps[min_dur_i].duration) {
        min_dur_i = i;
      }
    }
    return min_dur_i;
  }
};

struct EmbiggenResult {
  int embiggening;
  TimeSinceServiceStart min_dur_aot;
};

EmbiggenResult LocalEmbiggenIterative(
    const ProblemState& problem, const RefinerState& state, StopId a, StopId b
) {
  auto ab_it = state.pairs.find({a, b});
  assert(ab_it != state.pairs.end() && ab_it->second.steps.size() > 0);
  const PairSteps& ab = ab_it->second;

  std::vector<EmbiggenState> q;

  auto CombineAndPush =
      [&](const std::vector<PairStep>& xy,
          const std::vector<PairStep>& yz,
          int path_relaxed_weight,
          const std::vector<StopId> path,
          const std::vector<TimeSinceServiceStart> a_origin_times,
          bool a_in_xy) {
        if (xy.size() == 0 || yz.size() == 0) {
          // If there is no path, there is no need to push anything.
          return;
        }

        if (a_in_xy) {
          assert(a_origin_times.size() == xy.size());
        } else {
          assert(a_origin_times.size() == yz.size());
        }

        std::vector<PairStep> new_steps;
        std::vector<TimeSinceServiceStart> new_a_origin_times;
        {
          int new_steps_size_estimate = std::max(xy.size(), yz.size());
          new_steps.reserve(new_steps_size_estimate);
          new_a_origin_times.reserve(new_steps_size_estimate);
        }

        int min_duration = std::numeric_limits<int>::max();
        int yz_i = 0;
        for (int xy_i = 0; xy_i < xy.size(); ++xy_i) {
          while (yz_i < yz.size() &&
                 yz[yz_i].origin_time < xy[xy_i].DestinationTime()) {
            yz_i += 1;
          }
          if (yz_i == yz.size()) {
            // TODO: Can I assert that this only happens when the yz step has
            // been ub truncated? Cuz that's the only time I expect this to
            // happen.
            break;
          }
          assert(xy[xy_i].DestinationTime() == yz[yz_i].origin_time);
          int combined_dur = xy[xy_i].duration + yz[yz_i].duration;
          new_steps.push_back({
              xy[xy_i].origin_time,
              combined_dur,
          });
          if (a_in_xy) {
            new_a_origin_times.push_back(a_origin_times[xy_i]);
          } else {
            new_a_origin_times.push_back(a_origin_times[yz_i]);
          }
          min_duration = std::min(min_duration, combined_dur);
        }

        if (new_steps.size() == 0) {
          // If there is no path, there is no need to push anything.
          // TODO: Can this happen? Might want to assert that it doesn't, or
          // that it only happens in certain situations.
          return;
        }
        assert(min_duration < std::numeric_limits<int>::max());

        q.push_back(
            EmbiggenState{
                .allowed_embiggen = min_duration - path_relaxed_weight,
                .path_relaxed_weight = path_relaxed_weight,
                .path = path,
                .steps = std::move(new_steps),
                .a_origin_times = std::move(new_a_origin_times),
            }
        );
        std::push_heap(q.begin(), q.end());
      };

  {
    std::vector<TimeSinceServiceStart> initial_a_origin_times;
    initial_a_origin_times.reserve(ab.steps.size());
    for (const PairStep& step : ab.steps) {
      initial_a_origin_times.push_back(step.origin_time);
    }
    q.push_back(
        EmbiggenState{
            .allowed_embiggen = 0,
            .path_relaxed_weight = ab.RelaxedWeight(),
            .path = {a, b},
            .steps = ab.steps,
            .a_origin_times = std::move(initial_a_origin_times),
        }
    );
  }

  // TODO: We could terminate early if we see that there is no chance of
  // improving the bound in the remaining iterations.
  // We could even (maybe) prioritize over all edges and only spend effort on
  // the ones that have the most improveable bound.
  for (int iter = 0; iter < 1000; ++iter) {
    // The queue should never become empty because we can keep extending until a
    // full tour, and we early-return if we find a full tour on top of the
    // queue.
    assert(q.size() > 0);

    std::pop_heap(q.begin(), q.end());
    EmbiggenState cur = std::move(q.back());
    q.pop_back();

    auto IsInCurPath = [&](StopId s) {
      for (StopId path_s : cur.path) {
        if (path_s == s) {
          return true;
        }
      }
      return false;
    };

    if (cur.path.front() == problem.boundary.start &&
        cur.path.back() == problem.boundary.end) {
      // This shouldn't really ever happen, but handle it just in case. We've
      // found an entire tour and we're not allowed to embiggen past it.
      std::cout << "wat " << cur.path.size() << " / " << state.stops.size()
                << "\n";
      assert(false);
      // return EmbiggenResult{
      //   .embiggening = cur.allowed_embiggen,
      //   .min_dur_aot = TODO TODO
      // }
    }

    if (cur.path.front() == problem.boundary.start) {
      // We need to go forwards because the front of the path is START. It's
      // possible to go forwards because we've handled the case above where the
      // back of the path is END.
      for (StopId s : state.stops) {
        if (IsInCurPath(s)) {
          continue;
        }

        // If the front of the path is START, we're only allowed to go to END if
        // we've hit all the other stops.
        if (cur.path.front() == problem.boundary.start &&
            cur.path.size() < state.stops.size() - 1 &&
            s == problem.boundary.end) {
          continue;
        }

        std::vector<StopId> new_path = cur.path;
        new_path.push_back(s);
        auto tos_it = state.pairs.find({cur.path.back(), s});
        if (tos_it == state.pairs.end()) {
          continue;
        }
        CombineAndPush(
            cur.steps,
            tos_it->second.steps,
            cur.path_relaxed_weight + tos_it->second.RelaxedWeight(),
            new_path,
            cur.a_origin_times,
            true
        );
      }
    } else {
      // We can go backwards because the front of the path is not START.
      for (StopId s : state.stops) {
        if (IsInCurPath(s)) {
          continue;
        }

        // If the back of the path is END, we're only allowed to go to START if
        // we've hit all the other stops.
        if (cur.path.back() == problem.boundary.end &&
            cur.path.size() < state.stops.size() - 1 &&
            s == problem.boundary.start) {
          continue;
        }

        std::vector<StopId> new_path = {s};
        new_path.insert(new_path.end(), cur.path.begin(), cur.path.end());
        auto froms_it = state.pairs.find({s, cur.path.front()});
        if (froms_it == state.pairs.end()) {
          continue;
        }
        CombineAndPush(
            froms_it->second.steps,
            cur.steps,
            cur.path_relaxed_weight + froms_it->second.RelaxedWeight(),
            new_path,
            cur.a_origin_times,
            false
        );
      }
    }
  }

  const EmbiggenState& front = q.front();
  return EmbiggenResult{
      .embiggening = front.allowed_embiggen,
      .min_dur_aot = front.a_origin_times[front.MinDurI()],
  };

  // int allowed_embiggen = q.front().allowed_embiggen;
  // std::cout
  //   << problem.StopName(a) << " -> " << problem.StopName(b)
  //   << ": " << TimeSinceServiceStart{allowed_embiggen} << "\n";

  // {
  //   int front_min_dur_i = q.front().MinDurI();
  //   assert(front_min_dur_i != -1);
  //   int front_min_dur = q.front().steps[front_min_dur_i].duration;
  //   TimeSinceServiceStart front_min_dur_aot =
  //   q.front().a_origin_times[front_min_dur_i]; int
  //   front_min_dur_without_aot_i =
  //   q.front().MinDurIWithoutAOT(front_min_dur_aot);
  //   assert(front_min_dur_without_aot_i != -1);
  //   int front_min_dur_without_aot =
  //   q.front().steps[front_min_dur_without_aot_i].duration; int
  //   front_extra_embiggen = front_min_dur_without_aot - front_min_dur;

  //   int increased_embiggen = q.front().allowed_embiggen +
  //   front_extra_embiggen;

  //   while (q.size() > 0) {
  //     std::pop_heap(q.begin(), q.end());
  //     EmbiggenState cur = std::move(q.back());
  //     q.pop_back();

  //     int min_dur_i = cur.MinDurI();
  //     assert(min_dur_i != -1);
  //     int min_dur = cur.steps[min_dur_i].duration;
  //     TimeSinceServiceStart min_dur_aot = cur.a_origin_times[min_dur_i];
  //     int min_dur_without_aot_i = cur.MinDurIWithoutAOT(min_dur_aot);
  //     if (min_dur_without_aot_i == -1) {
  //       // std::cout
  //       //   << "  "
  //       //   << TimeSinceServiceStart{cur.allowed_embiggen} << ": all thru
  //       aot!\n"; continue;
  //     }
  //     int min_dur_without_aot = cur.steps[min_dur_without_aot_i].duration;
  //     int extra_embiggen = min_dur_without_aot - min_dur;

  //     // std::cout
  //     //   << "  "
  //     //   << TimeSinceServiceStart{cur.allowed_embiggen} << " + "
  //     //   << TimeSinceServiceStart{extra_embiggen} << " = "
  //     //   << TimeSinceServiceStart{cur.allowed_embiggen + extra_embiggen} <<
  //     "\n";

  //     increased_embiggen = std::min(increased_embiggen, cur.allowed_embiggen
  //     + extra_embiggen); if (cur.allowed_embiggen >= increased_embiggen) {
  //       break;
  //     }
  //   }

  //   std::cout << "  increased embiggen from forbid one aot: "
  //     << TimeSinceServiceStart{allowed_embiggen} << " -> "
  //     << TimeSinceServiceStart{increased_embiggen} << "\n";
  // }

  // // for (int i = 0; i < 10 && q.size() > 0; ++i) {
  // //   std::pop_heap(q.begin(), q.end());
  // //   EmbiggenState cur = std::move(q.back());
  // //   q.pop_back();

  // //   std::vector<TimeSinceServiceStart> intersected;
  // //   std::set_intersection(
  // //       a_origin_times.begin(), a_origin_times.end(),
  // //       cur.a_origin_times.begin(), cur.a_origin_times.end(),
  // //       std::back_inserter(intersected)
  // //   );
  // //   a_origin_times = std::move(intersected);

  // //   std::cout << "  " << TimeSinceServiceStart{cur.allowed_embiggen} << "
  // / " << a_origin_times.size() << "\n";
  // // }

  // return allowed_embiggen;
}

int LocalEmbiggen(RefinerState& state, StopId a, StopId b) {
  PairSteps ab = state.pairs[{a, b}];
  if (ab.steps.size() == 0) {
    return 0;
  }

  // Consider all x->a->b.
  int allowed_embiggen = std::numeric_limits<int>::max();
  for (StopId x : state.stops) {
    if (x == a || x == b) {
      continue;
    }

    const PairSteps& xa = state.pairs[{x, a}];
    if (xa.steps.size() == 0) {
      continue;
    }

    // The current relaxed weight of x->a->b;
    int relaxed_weight = xa.RelaxedWeight() + ab.RelaxedWeight();

    // The best possible x->a->b duration.
    int actual_best_dur = std::numeric_limits<int>::max();
    int ab_i = 0;
    for (int xa_i = 0; xa_i < xa.steps.size(); ++xa_i) {
      while (ab_i < ab.steps.size() &&
             ab.steps[ab_i].origin_time < xa.steps[xa_i].DestinationTime()) {
        ab_i += 1;
      }
      if (ab_i == ab.steps.size()) {
        // TODO: Can I assert that this only happens when the ab step has been
        // ub truncated? Cuz that's the only time I expect this to happen.
        continue;
      }
      assert(ab.steps[ab_i].origin_time == xa.steps[xa_i].DestinationTime());
      actual_best_dur = std::min(
          actual_best_dur, ab.steps[ab_i].duration + xa.steps[xa_i].duration
      );
    }

    assert(actual_best_dur >= relaxed_weight);
    allowed_embiggen =
        std::min(allowed_embiggen, actual_best_dur - relaxed_weight);
  }

  return allowed_embiggen;
}

int main(int argc, char* argv[]) {
  CLI::App app{"Refiner tool"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  std::string gtfs_stop_id_str;
  app.add_option("gtfs_stop_id", gtfs_stop_id_str, "GTFS stop ID")->required();

  std::string time_str;
  app.add_option("time", time_str, "Departure time in hh:mm:ss format")
      ->required();

  std::string lb_str;
  app.add_option(
         "--lb",
         lb_str,
         "Lower bound in hh:mm:ss format (relative to departure time)"
  )
      ->required();

  std::string ub_str;
  app.add_option(
         "--ub",
         ub_str,
         "Upper bound in hh:mm:ss format (relative to departure time)"
  )
      ->required();

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
  std::cout << "required size: " << problem.required.size() << "\n";

  StopId s0 = FindStopByGtfsId(problem, gtfs_stop_id_str);
  std::cout << "s0: " << problem.StopName(s0) << "\n";

  TimeSinceServiceStart t0 = TimeSinceServiceStart::Parse(time_str);
  std::cout << "t0: " << t0 << "\n";

  int lb_rel = TimeSinceServiceStart::Parse(lb_str).seconds;
  int ub_rel = TimeSinceServiceStart::Parse(ub_str).seconds;

  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());

  RefinerState rs = MakeRefinerState(problem, completed, s0, t0, ub_rel);

  for (StopId s : problem.required.GroupRepresentatives()) {
    std::set<TimeSinceServiceStart> arrival_times_set;

    for (const auto& [pair, pair_steps] : rs.pairs) {
      if (pair.first == s) {
        for (const PairStep ps : pair_steps.steps) {
          arrival_times_set.insert(ps.origin_time);
        }
      }
      if (pair.second == s) {
        for (const PairStep ps : pair_steps.steps) {
          arrival_times_set.insert(ps.DestinationTime());
        }
      }
    }

    std::vector<TimeSinceServiceStart> arrival_times(
        arrival_times_set.begin(), arrival_times_set.end()
    );

    std::cout << problem.StopName(s) << ": ";
    if (arrival_times.size() <= 3) {
      for (int i = 0; i < arrival_times.size(); ++i) {
        if (i > 0) {
          std::cout << ", ";
        }
        std::cout << arrival_times[i];
      }
    } else {
      std::cout << arrival_times.front() << ", ..., " << arrival_times.back()
                << " (" << arrival_times.size() << ")";
    }
    std::cout << "\n";
  }

  // for (StopId a : rs.stops) {
  //   for (StopId b : rs.stops) {
  //     if (a == b) {
  //       continue;
  //     }

  //     // int once_local_embiggen = LocalEmbiggen(rs, a, b);
  //     int iter_local_embiggen = LocalEmbiggenIterative(problem, rs, a, b);

  //     std::cout
  //       << problem.StopName(a) << " -> " << problem.StopName(b) << ": "
  //       << TimeSinceServiceStart{iter_local_embiggen} << "\n";

  //     if (iter_local_embiggen > 0) {
  //       rs.pairs[{a, b}].local_embiggening += iter_local_embiggen;
  //     }

  //     // assert(iter_local_embiggen == once_local_embiggen);
  //   }
  // }

  int last_lb = 0;

  while (true) {
    std::optional<TspTourResult> result = DoTSP(problem, rs, ub_rel);
    if (!result.has_value()) {
      std::cout << "no result\n";
      break;
    }
    if (result->optimal_value == last_lb) {
      break;
    }
    last_lb = result->optimal_value;

    TourAnalysis analysis = AnalyzeTour(problem, rs, *result, t0, false);
    std::cout << "result: " << TimeSinceServiceStart{analysis.val_relaxed}
              << " / " << TimeSinceServiceStart{analysis.val_actual} << "\n";

    for (const TarelEdge& edge : result->tour_edges) {
      EmbiggenResult embiggen = LocalEmbiggenIterative(
          problem, rs, edge.origin.stop, edge.destination.stop
      );
      if (embiggen.embiggening > 0) {
        rs.pairs[{edge.origin.stop, edge.destination.stop}].local_embiggening +=
            embiggen.embiggening;
      }

      {
        RefinerState rs_forbid = rs;
        PairSteps& rs_forbid_pair_steps =
            rs_forbid.pairs.at({edge.origin.stop, edge.destination.stop});
        // ohno i need to repropagate everything...
        std::erase_if(rs_forbid_pair_steps.steps, [&](const PairStep& ps) {
          return ps.origin_time == embiggen.min_dur_aot;
        });
        if (rs_forbid_pair_steps.steps.size() > 0) {
          EmbiggenResult embiggen_forbid = LocalEmbiggenIterative(
              problem, rs_forbid, edge.origin.stop, edge.destination.stop
          );
          std::cout << problem.StopName(edge.origin.stop) << " -> "
                    << problem.StopName(edge.destination.stop) << ": "
                    << TimeSinceServiceStart{embiggen.embiggening} << " => "
                    << TimeSinceServiceStart{embiggen_forbid.embiggening}
                    << "\n";
        }
      }
    }

    // if (!analysis.worst_edge.has_value()) {
    //   std::cout << "no worst edge\n";
    //   break;
    // }
    // std::cout << "Forbid "
    //   << problem.StopName(analysis.worst_edge->origin.stop) << " -> "
    //   << problem.StopName(analysis.worst_edge->destination.stop) << ": "
    //   << TimeSinceServiceStart{analysis.worst_edge->weight} << "\n";
    // ForbidDurationBetween(
    //   rs,
    //   analysis.worst_edge->origin.stop,
    //   analysis.worst_edge->destination.stop,
    //   analysis.worst_edge->weight
    // );
  }

  return 0;
}
