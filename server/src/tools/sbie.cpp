#include <crow/http_parser_merged.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/branch_and_bound.h"
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

// a -> b@tb
struct StepId {
  StopId a;
  StopId b;
  TimeSinceServiceStart tb;
  bool operator==(const StepId& o) const {
    return a == o.a && b == o.b && tb.seconds == o.tb.seconds;
  }
  std::string DebugString(const ProblemState& problem) const {
    return problem.StopName(a) + " -> " + problem.StopName(b) + "@" +
           tb.ToString();
  }
};

template <>
struct std::hash<StepId> {
  size_t operator()(const StepId& s) const {
    size_t h = std::hash<int>{}(s.a.v);
    h ^= std::hash<int>{}(s.b.v) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(s.tb.seconds) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

std::pair<TimeSinceServiceStart, StepPartitionId> GetTNext(
    const StepsAdjacencyList& completed,
    const StepGroup& g_next,
    StopId from_s,
    TimeSinceServiceStart t_cur,
    const std::unordered_set<StepId>& forbidden_steps = {}
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
    if (forbidden_steps.contains(
            StepId{.a = from_s, .b = g_next.destination_stop, .tb = t_next_flex}
        )) {
      part_next_flex = StepPartitionId::NONE;
      t_next_flex.seconds = std::numeric_limits<int>::max();
    }
  }

  StepPartitionId part_next_sched = StepPartitionId::NONE;
  TimeSinceServiceStart t_next_sched{std::numeric_limits<int>::max()};
  std::span<const AdjacencyListStep> group_steps = completed.GetSteps(g_next);
  size_t t_next_i = FindDepartureAtOrAfter(completed, g_next, t_cur);
  while (t_next_i < group_steps.size() &&
         forbidden_steps.contains(
             StepId{
                 .a = from_s,
                 .b = g_next.destination_stop,
                 .tb = group_steps[t_next_i].destination_time
             }
         )) {
    t_next_i += 1;
  }
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

  int MinDur() const {
    auto min_it = std::ranges::min_element(steps, {}, [](const PairStep& ps) {
      return ps.duration;
    });
    assert(min_it != steps.end());
    return min_it->duration;
  }

  int RelaxedWeight() const { return MinDur() + local_embiggening; }

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

// TODO(Friday): Generalize this to take all constraints into account.
// REQUIRE constraints operate similarly to the s0@t0 thing in that they seed
// the thing there. But also they constrain you to not be able to go to places
// where you won't manage to reach the required stop in time. FORBID constraints
// simply eliminate it as an allowed step.

struct KnownPoint {
  StopId s;
  TimeSinceServiceStart t;
  bool operator==(const KnownPoint&) const = default;
};

template <>
struct std::hash<KnownPoint> {
  size_t operator()(const KnownPoint& p) const {
    size_t h = std::hash<int>{}(p.s.v);
    h ^= std::hash<int>{}(p.t.seconds) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

// The path starts at known_steps[0], which must be START->s@t.
// Then it does whatever it needs to do to get to and accomplish known_steps[1],
// known_steps[2], etc. Finally it goes to END. The path duration does not
// exceed ub_rel, and it does not get to END with duration under lb_rel.
RefinerState GeneralizedMakeRefinerState(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    std::vector<StepId> known_steps,
    std::unordered_set<StepId> forbidden_steps,
    int lb_rel,
    int ub_rel
) {
  assert(known_steps.size() > 0);
  assert(known_steps[0].a == problem.boundary.start);
  for (int i = 0; i + 1 < known_steps.size(); ++i) {
    assert(known_steps[i].tb <= known_steps[i + 1].tb);
  }

  RefinerState result;
  {
    auto required_stop_reps = problem.required.GroupRepresentatives();
    result.stops = std::vector<StopId>(
        required_stop_reps.begin(), required_stop_reps.end()
    );
    result.pairs.reserve(problem.required.size() * problem.required.size());
  }

  std::unordered_set<StopId> known_step_stops;
  for (const StepId step : known_steps) {
    known_step_stops.insert(step.a);
    known_step_stops.insert(step.b);
  }
  known_step_stops.insert(problem.boundary.end);

  auto ExtendSteps = [&](KnownPoint a,
                         StopId b,
                         TimeSinceServiceStart t_b_lo,
                         TimeSinceServiceStart t_b_hi) {
    std::vector<KnownPoint> stack;
    std::unordered_set<KnownPoint> visited;

    std::vector<KnownPoint> b_times;

    stack.push_back(a);

    while (stack.size() > 0) {
      KnownPoint cur = stack.back();
      stack.pop_back();
      if (visited.contains(cur)) {
        continue;
      }
      visited.insert(cur);
      if (cur.s == b) {
        b_times.push_back(cur);
        continue;
      }
      for (const StepGroup& g_next : completed.GetGroups(cur.s)) {
        StopId s_next = g_next.destination_stop;
        // We can only go to a known_point_stop if it is the next known point.
        if (known_step_stops.contains(s_next) && s_next != b) {
          continue;
        }
        auto [t_next, _] =
            GetTNext(completed, g_next, cur.s, cur.t, forbidden_steps);
        // if (s_next == b && b != problem.boundary.end) {
        //   if (t_next > t_b_hi) {
        //     std::cout << "  to b pruned, " << t_next << " > " << t_b_hi <<
        //     "\n";
        //   }
        //   if (t_next < t_b_lo) {
        //     std::cout << "  to b pruned, " << t_next << " < " << t_b_lo <<
        //     "\n";
        //   }
        // }
        if (t_next > t_b_hi || (s_next == b && t_next < t_b_lo)) {
          continue;
        }
        result.pairs[{cur.s, s_next}].steps.push_back(
            {cur.t, t_next.seconds - cur.t.seconds}
        );
        stack.push_back({s_next, t_next});
      }
    }

    // TODO: Maybe count the number of distinct paths somehow?
    // std::cout
    //   << "Extending "
    //   << problem.StopName(a.s) << " @ " << a.t << " -> "
    //   << problem.StopName(b) << " @ [" << t_b_lo << ", " << t_b_hi << "]";
    // if (b_times.size() == 0) {
    //   std::cout  << ": NOT REACHED!";
    // }
    // std::cout << "\n";

    return b_times;
  };

  std::vector<KnownPoint> step_heads;
  step_heads.push_back({problem.boundary.start, known_steps[0].tb});

  for (int i = 0; i < known_steps.size(); ++i) {
    // First extend along the known step.
    std::ranges::sort(step_heads, {}, [](const KnownPoint& kp) {
      return kp.t;
    });
    int found_exts = 0;
    for (const StepGroup& g_next : completed.GetGroups(known_steps[i].a)) {
      if (g_next.destination_stop != known_steps[i].b) {
        continue;
      }
      for (const KnownPoint& head : step_heads) {
        assert(head.s == known_steps[i].a);
        auto [t_next, _] =
            GetTNext(completed, g_next, head.s, head.t, forbidden_steps);
        if (t_next == known_steps[i].tb) {
          found_exts += 1;
          result.pairs[{head.s, g_next.destination_stop}].steps.push_back(
              {head.t, t_next.seconds - head.t.seconds}
          );
        }
      }
    }
    std::cout << "Extend step: " << problem.StopName(known_steps[i].a) << " -> "
              << problem.StopName(known_steps[i].b) << " @ "
              << known_steps[i].tb << ": " << found_exts << "\n";
    // TODO: Can we avoid this ever happening?
    // Like I think it happens when we select an edge that cannot reach a future
    // step. So I could filter such edges preemptively. And then assert this.
    // assert(found_exts > 0);

    // Then extend to the next known step.
    KnownPoint a{known_steps[i].b, known_steps[i].tb};
    if (i + 1 < known_steps.size()) {
      step_heads = ExtendSteps(
          a,
          known_steps[i + 1].a,
          TimeSinceServiceStart{0},
          known_steps[i + 1].tb
      );
    } else {
      TimeSinceServiceStart t0 = known_steps[0].tb;
      TimeSinceServiceStart t_lb{t0.seconds + lb_rel};
      TimeSinceServiceStart t_ub{t0.seconds + ub_rel};
      step_heads = ExtendSteps(a, problem.boundary.end, t_lb, t_ub);
    }
  }

  // for (int i = 0; i < known_points.size(); ++i) {
  //   KnownPoint a = known_points[i];
  //   if (i + 1 < known_points.size()) {
  //     KnownPoint b = known_points[i + 1];
  //     ExtendSteps(a, b.s, b.t, b.t);
  //   } else {
  //     TimeSinceServiceStart t0 = known_points[0].t;
  //     TimeSinceServiceStart t_lb{t0.seconds + lb_rel};
  //     TimeSinceServiceStart t_ub{t0.seconds + ub_rel};
  //     ExtendSteps(a, problem.boundary.end, t_lb, t_ub);
  //   }
  // }

  for (auto& [_, pair] : result.pairs) {
    std::ranges::sort(pair.steps, {}, [](const PairStep& ps) {
      return ps.origin_time;
    });
    for (size_t i = 0; i + 1 < pair.steps.size(); ++i) {
      assert(pair.steps[i].origin_time != pair.steps[i + 1].origin_time);
    }
  }

  // Now delete steps that don't go anywhere.
  // This happens because if we have a constraint that you get to s@t, then
  // before that constraint we consider all the steps going anywhere <t, and
  // some of those may go somewhere where they can then never get to s in time.
  while (true) {
    int iter_deleted = 0;
    std::unordered_map<StopId, std::unordered_set<TimeSinceServiceStart>>
        dep_times;
    for (const auto& [p, ps] : result.pairs) {
      for (const PairStep& step : ps.steps) {
        dep_times[p.first].insert(step.origin_time);
        if (p.second == problem.boundary.end) {
          dep_times[p.second].insert(step.DestinationTime());
        }
      }
    }
    for (auto& [p, ps] : result.pairs) {
      std::erase_if(ps.steps, [&](const PairStep& step) {
        if (!dep_times[p.second].contains(step.DestinationTime())) {
          iter_deleted += 1;
          return true;
        }
        return false;
      });
    }
    std::erase_if(result.pairs, [](const auto& p) {
      return p.second.steps.size() == 0;
    });
    if (iter_deleted == 0) {
      break;
    }
  }

  return result;
}

struct EmbiggenState {
  // TODO: We can do a constant-size bitvector for much more speed.
  std::vector<StopId> path;

  TimeSinceServiceStart t_front;
  TimeSinceServiceStart t_back;
  int delta;
  bool operator<(const EmbiggenState& other) const {
    return delta > other.delta;
  }
};

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
    // Actual lower bound!
    if (pair_steps.steps.size() == 0) {
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

    auto step_it = std::find_if(
        pair_steps.steps.begin(),
        pair_steps.steps.end(),
        [&](const PairStep& ps) { return ps.origin_time == t_actual; }
    );
    if (step_it == pair_steps.steps.end()) {
      t_actual.seconds = std::numeric_limits<int>::max();
    } else {
      t_actual = step_it->DestinationTime();
    }
    s = edge.destination.stop;
    t_relaxed.seconds += edge.weight;
    EmitCur();
  }
  return TourAnalysis{
      .val_relaxed = t_relaxed.seconds - t0.seconds,
      .val_actual = t_actual.seconds - t0.seconds,
  };
}

void VisitUnforcedSteps(
    const RefinerState& rs,
    const EmbiggenState& steps,
    std::function<void(StepId)> f
) {
  assert(steps.path.size() >= 2);

  TimeSinceServiceStart t_cur = steps.t_front;
  for (int i = 0; i + 1 < steps.path.size(); ++i) {
    const PairSteps& ps = rs.pairs.at({steps.path[i], steps.path[i + 1]});
    auto step_it =
        std::ranges::lower_bound(ps.steps, t_cur, {}, [](const PairStep& step) {
          return step.origin_time;
        });
    assert(step_it != ps.steps.end());
    assert(step_it->origin_time == t_cur);
    t_cur = step_it->DestinationTime();

    bool is_forced = true;
    for (const PairStep& step : ps.steps) {
      if (step.DestinationTime() != ps.steps[0].DestinationTime()) {
        is_forced = false;
        break;
      }
    }
    if (!is_forced) {
      f(StepId{steps.path[i], steps.path[i + 1], t_cur});
    }
  }

  assert(t_cur == steps.t_back);
}

int LocalEmbiggenIterative(
    const ProblemState& problem,
    const RefinerState& state,
    int ub_rel,
    StopId a,
    StopId b,
    std::unordered_map<StepId, int>& accumulated_extra_delta
) {
  std::vector<EmbiggenState> q;

  // Initialize the heap with all the a->b steps.
  {
    auto ab_it = state.pairs.find({a, b});
    assert(ab_it != state.pairs.end() && ab_it->second.steps.size() > 0);
    q.reserve(ab_it->second.steps.size());
    int ab_relaxed_weight = ab_it->second.RelaxedWeight();
    for (const PairStep& step : ab_it->second.steps) {
      // assert(step.duration >= ab_relaxed_weight);
      q.push_back(
          EmbiggenState{
              .path = {a, b},
              .t_front = step.origin_time,
              .t_back = step.DestinationTime(),
              .delta = step.duration - ab_relaxed_weight,
          }
      );
    }
    std::make_heap(q.begin(), q.end());
  }

  for (int embiggen_round = 0; embiggen_round < 100; ++embiggen_round) {
    std::pop_heap(q.begin(), q.end());
    EmbiggenState cur = std::move(q.back());
    q.pop_back();

    if (cur.path.front() == problem.boundary.start &&
        cur.path.back() == problem.boundary.end) {
      // We have found an entire valid tour through a->b, so we can't embiggen
      // past it, so this is as good as we can do.

      // The tour must hit every stop.
      assert(cur.path.size() == state.stops.size());

      // TODO: Actually return a result in this case. For now we just assert so
      // that we don't accidentally forget to handle the case when it starts
      // happening.
      assert(false);
    }

    auto IsInCurPath = [&](StopId s) {
      for (StopId path_s : cur.path) {
        if (path_s == s) {
          return true;
        }
      }
      return false;
    };

    auto EmbiggenForwards = [&]() {
      for (StopId s : state.stops) {
        if (
          // Can't go to a stop we've already visited.
          IsInCurPath(s) ||
          // Can't go forwards to START
          s == problem.boundary.start ||
          // If the path starts at START and hasn't visited everything else, can't go to END.
          (cur.path.front() == problem.boundary.start && s == problem.boundary.end && cur.path.size() + 1 < state.stops.size())
        ) {
          continue;
        }
        auto ps_it = state.pairs.find({cur.path.back(), s});
        if (ps_it == state.pairs.end()) {
          continue;
        }
        const PairSteps& ps = ps_it->second;
        auto step_it = std::ranges::lower_bound(
            ps.steps, cur.t_back, {}, [](const PairStep& step) {
              return step.origin_time;
            }
        );
        if (step_it == ps.steps.end() || step_it->origin_time != cur.t_back) {
          // TODO: Under what conditions can this happen? Assert them?
          continue;
        }
        std::vector<StopId> new_path;
        new_path.reserve(cur.path.size() + 1);
        new_path.insert(new_path.end(), cur.path.begin(), cur.path.end());
        new_path.push_back(s);
        q.push_back(
            EmbiggenState{
                .path = new_path,
                .t_front = cur.t_front,
                .t_back = step_it->DestinationTime(),
                .delta = cur.delta + step_it->duration - ps.RelaxedWeight(),
            }
        );
        std::push_heap(q.begin(), q.end());
      }
    };

    auto EmbiggenBackwards = [&]() {
      for (StopId s : state.stops) {
        if (
          // Can't go to a stop we've already visited.
          IsInCurPath(s) ||
          // Can't go backwards to END.
          s == problem.boundary.end ||
          // If the path ends at END and we haven't visited everything else, then we can't go to START.
          (cur.path.back() == problem.boundary.end && s == problem.boundary.start && cur.path.size() + 1 < state.stops.size())
        ) {
          continue;
        }
        auto ps_it = state.pairs.find({s, cur.path.front()});
        if (ps_it == state.pairs.end()) {
          continue;
        }
        const PairSteps& ps = ps_it->second;
        auto step_it = std::ranges::lower_bound(
            ps.steps, cur.t_front, {}, [](const PairStep& step) {
              return step.DestinationTime();
            }
        );
        if (step_it == ps.steps.end() ||
            step_it->DestinationTime() != cur.t_front) {
          // TODO: Under what conditions can this happen? Assert them?
          continue;
        }
        std::vector<StopId> new_path;
        new_path.reserve(cur.path.size() + 1);
        new_path.push_back(s);
        new_path.insert(new_path.end(), cur.path.begin(), cur.path.end());
        q.push_back(
            EmbiggenState{
                .path = new_path,
                .t_front = step_it->origin_time,
                .t_back = cur.t_back,
                .delta = cur.delta + step_it->duration - ps.RelaxedWeight(),
            }
        );
        std::push_heap(q.begin(), q.end());
      }
    };

    if (cur.path.front() == problem.boundary.start) {
      // We have to extend forwards.
      EmbiggenForwards();
    } else if (cur.path.back() == problem.boundary.end) {
      // We have to extend backwards.
      EmbiggenBackwards();
    } else {
      // We are free to extend in either direction.
      EmbiggenBackwards();
    }

    // If the queue is empty, it means that we have pruned all possible paths
    // through a->b, so we can delete a->b from the problem.
    // TODO: Do deletion in a cleaner way than setting the weight to be massive.
    if (q.size() == 0) {
      std::cout << "Deleting: " << problem.StopName(a) << " -> "
                << problem.StopName(b) << "\n";
      return 24 * 3600;
    }
  }

  std::pop_heap(q.begin(), q.end());
  EmbiggenState top_state = std::move(q.back());
  q.pop_back();

  std::unordered_map<StepId, int> extra_delta;
  std::unordered_set<StepId> blocking_steps;
  VisitUnforcedSteps(state, top_state, [&](const StepId step) {
    blocking_steps.insert(step);
  });
  while (q.size() > 0 && blocking_steps.size() > 0) {
    std::pop_heap(q.begin(), q.end());
    EmbiggenState cur = std::move(q.back());
    q.pop_back();
    for (const StepId& step : blocking_steps) {
      extra_delta[step] = cur.delta;
    }

    std::unordered_set<StepId> new_blocking_steps;
    VisitUnforcedSteps(state, cur, [&](const StepId step) {
      if (blocking_steps.contains(step)) {
        new_blocking_steps.insert(step);
      }
    });
    blocking_steps = std::move(new_blocking_steps);
  }

  for (const auto& [step, delta] : extra_delta) {
    accumulated_extra_delta[step] += delta;
  }

  return top_state.delta;
}

// void RequireEdge(RefinerState& state, StopId a, StopId b) {
//   std::erase_if(state.pairs, [&](const std::pair<std::pair<StopId, StopId>,
//   PairSteps> kv) {
//     std::pair<StopId, StopId> k = kv.first;
//     return (
//       (k.first == a && k.second != b) ||
//       (k.first != a && k.second == b)
//     );
//   });
// }

// void ForbidEdge(RefinerState& state, StopId a, StopId b) {
//   std::erase_if(state.pairs, [&](const std::pair<std::pair<StopId, StopId>,
//   PairSteps> kv) {
//     std::pair<StopId, StopId> k = kv.first;
//     return k.first == a && k.second == b;
//   });
// }

// void RequireStep(RefinerState& state, StepId id) {
//   std::erase_if(state.pairs.at({id.a, id.b}).steps, [&](const PairStep& step)
//   {
//     return step.origin_time != id.t;
//   });
//   RequireEdge(state, id.a, id.b);
// }

// void ForbidStep(RefinerState& state, StepId id) {
//   std::vector<PairStep>& steps = state.pairs.at({id.a, id.b}).steps;
//   if (steps.size() == 1) {
//     assert(steps[0].origin_time == id.t);
//     state.pairs.erase({id.a, id.b});
//     return;
//   }
//   std::erase_if(steps, [&](const PairStep& step) {
//     return step.origin_time == id.t;
//   });
// }

// void CleanupSteps(StopId start, RefinerState& state) {
//   bool deleted = true;
//   while (deleted) {
//     deleted = false;
//     std::unordered_map<StopId, std::unordered_set<TimeSinceServiceStart>>
//     arrival_times; for (const auto& [p, ps] : state.pairs) {
//       for (const PairStep& step : ps.steps) {
//         arrival_times[p.second].insert(step.DestinationTime());
//       }
//     }
//     for (auto& [p, ps] : state.pairs) {
//       if (p.first == start) {
//         continue;
//       }
//       std::erase_if(ps.steps, [&](const PairStep& step) {
//         if (!arrival_times[p.first].contains(step.origin_time)) {
//           deleted = true;
//           return true;
//         }
//         return false;
//       });
//     }
//   }
// }

struct RefineResult {
  TspTourResult tour;
  std::unordered_map<StepId, int> accumulated_extra_delta;
  std::vector<std::pair<StopId, StopId>> refined_edges;
  RefinerState refined_state;

  TspTourResult best_tour;
  int best_tour_ub;

  std::vector<std::pair<StepId, int>> SortedExtraDelta() const {
    std::vector<std::pair<StepId, int>> sorted_extra(
        accumulated_extra_delta.begin(), accumulated_extra_delta.end()
    );
    std::sort(
        sorted_extra.begin(),
        sorted_extra.end(),
        [](const auto& a, const auto& b) { return a.second > b.second; }
    );
    return sorted_extra;
  }
};

std::optional<RefineResult> DoRefine(
    const ProblemState& problem,
    const RefinerState& state,
    TimeSinceServiceStart t0,
    int ub_rel,
    const std::vector<std::pair<StopId, StopId>>& initial_refine_edges,
    bool print
) {
  int best_tour_ub = std::numeric_limits<int>::max();
  TspTourResult best_tour;
  RefinerState rs = state;

  using Clock = std::chrono::steady_clock;
  auto total_tsp_time = Clock::duration::zero();
  auto total_embiggen_time = Clock::duration::zero();

  std::vector<std::pair<StopId, StopId>> refined_edges;

  // Refine the initial_refine_edges.
  // {
  //   int acc_delta = 0;
  //   std::unordered_map<StepId, int> accumulated_extra_delta;
  //   auto embiggen_start = Clock::now();
  //   for (const std::pair<StopId, StopId>& edge_pair : initial_refine_edges) {
  //     auto ab_it = rs.pairs.find({edge_pair.first, edge_pair.second});
  //     if (ab_it == rs.pairs.end() || ab_it->second.steps.size() == 0 ||
  //         ab_it->second.RelaxedWeight() >= 24 * 3600) {
  //       continue;
  //     }

  //     // TODO: Consider whether it's correct and/or good to skip edges that
  //     have
  //     // already been refined. One possible problem is that it might
  //     interfere
  //     // with the accumulated_extra_delta. if
  //     (std::find(refined_edges.begin(),
  //     // refined_edges.end(), edge_pair) != refined_edges.end()) {
  //     //   continue;
  //     // }
  //     refined_edges.push_back(edge_pair);

  //     // const TarelEdge& edge = result->tour_edges[edge_i];

  //     // TODO: Explain why it's disadvantageous to embiggen these edges.
  //     // if (rs.pairs.at({edge.origin.stop,
  //     edge.destination.stop}).steps.size()
  //     // == 1) {
  //     //   continue;
  //     // }

  //     int delta = LocalEmbiggenIterative(
  //         problem,
  //         rs,
  //         ub_rel,
  //         edge_pair.first,
  //         edge_pair.second,
  //         accumulated_extra_delta
  //     );
  //     // std::cout << edge_i << ". " << problem.StopName(edge.origin.stop) <<
  //     "
  //     // -> " << problem.StopName(edge.destination.stop) << ": " <<
  //     // TimeSinceServiceStart{delta} << "\n";
  //     if (delta > 0) {
  //       rs.pairs.at(edge_pair).local_embiggening += delta;
  //       acc_delta += delta;
  //     }
  //   }
  //   total_embiggen_time += Clock::now() - embiggen_start;
  //   if (print)
  //     std::cout << "Initial Refinement Total Delta: "
  //               << TimeSinceServiceStart{acc_delta} << "\n";
  // }

  int refine_round = 0;
  while (true) {
    if (print) std::cout << "===== REFINE ROUND " << refine_round << " =====\n";
    refine_round += 1;

    auto tsp_start = Clock::now();
    std::optional<TspTourResult> result = DoTSP(problem, rs, ub_rel);
    total_tsp_time += Clock::now() - tsp_start;

    if (!result.has_value()) {
      return std::nullopt;
    }
    TourAnalysis analysis = AnalyzeTour(problem, rs, *result, t0, false);
    if (analysis.val_actual < best_tour_ub) {
      best_tour_ub = analysis.val_actual;
      best_tour = *result;
    }
    if (print)
      std::cout << "result: " << TimeSinceServiceStart{result->optimal_value}
                << " / " << TimeSinceServiceStart{analysis.val_actual} << "\n";
    int acc_delta = 0;

    std::unordered_map<StepId, int> accumulated_extra_delta;
    auto embiggen_start = Clock::now();
    for (int edge_i = 0; edge_i < result->tour_edges.size(); ++edge_i) {
      const TarelEdge& edge =
          result->tour_edges[result->tour_edges.size() - 1 - edge_i];
      std::pair<StopId, StopId> edge_pair = {
          edge.origin.stop, edge.destination.stop
      };
      refined_edges.push_back(edge_pair);
      int delta = LocalEmbiggenIterative(
          problem,
          rs,
          ub_rel,
          edge.origin.stop,
          edge.destination.stop,
          accumulated_extra_delta
      );
      if (delta > 0) {
        rs.pairs.at(edge_pair).local_embiggening += delta;
        acc_delta += delta;
      }
    }
    total_embiggen_time += Clock::now() - embiggen_start;

    if (print)
      std::cout << "Total Delta: " << TimeSinceServiceStart{acc_delta} << "\n";
    if (acc_delta == 0) {
      if (print) {
        auto tsp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          total_tsp_time
        )
                          .count();
        auto embiggen_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                total_embiggen_time
            )
                .count();
        std::cout << "Time in DoTSP: " << tsp_ms << " ms\n";
        std::cout << "Time in embiggen loop: " << embiggen_ms << " ms\n";
      }

      std::unordered_map<StepId, int> reacc_extra_delta;
      for (int edge_i = 0; edge_i < result->tour_edges.size(); ++edge_i) {
        const TarelEdge& edge =
            result->tour_edges[result->tour_edges.size() - 1 - edge_i];
        int delta = LocalEmbiggenIterative(
            problem,
            rs,
            ub_rel,
            edge.origin.stop,
            edge.destination.stop,
            reacc_extra_delta
        );
        assert(delta <= 0);
      }

      return RefineResult{
          .tour = std::move(*result),
          .accumulated_extra_delta = std::move(reacc_extra_delta),
          .refined_edges = refined_edges,
          .refined_state = rs,
          .best_tour = best_tour,
          .best_tour_ub = best_tour_ub,
      };
    }
  }
}

int main(int argc, char* argv[]) {
  CLI::App app{"SBIE tool"};

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
  std::vector<Step> completed_steps = completed.AllSteps();

  // Just one step, no branching.
  if (false) {
    // RefinerState rs = MakeRefinerState(problem, completed, s0, t0, lb_rel,
    // ub_rel);
    std::vector<StepId> known_steps;
    known_steps.push_back({problem.boundary.start, s0, t0});
    RefinerState rs = GeneralizedMakeRefinerState(
        problem, completed, known_steps, {}, lb_rel, ub_rel
    );
    std::optional<RefineResult> result =
        DoRefine(problem, rs, t0, ub_rel, {}, true);
    if (!result.has_value()) {
      std::cout << "no result\n";
      return 0;
    }
    TourAnalysis analysis = AnalyzeTour(problem, rs, result->tour, t0, false);
    std::cout << "result: " << TimeSinceServiceStart{analysis.val_relaxed}
              << " / " << TimeSinceServiceStart{analysis.val_actual} << "\n";
    {
      std::vector<std::pair<StepId, int>> sorted_extra(
          result->accumulated_extra_delta.begin(),
          result->accumulated_extra_delta.end()
      );
      std::sort(
          sorted_extra.begin(),
          sorted_extra.end(),
          [](const auto& a, const auto& b) { return a.second > b.second; }
      );
      int n = std::min<int>(sorted_extra.size(), 10);
      for (int i = 0; i < n; ++i) {
        std::cout << "  extra_delta[" << i
                  << "]: " << sorted_extra[i].first.DebugString(problem)
                  << " = " << TimeSinceServiceStart{sorted_extra[i].second}
                  << "\n";
      }
    }
    return 0;
  }

  struct BranchConstraint {
    // TODO: ta is inelegant and confusing. Make it better or at least explain
    // it.
    TimeSinceServiceStart ta;
    StepId step;
    bool reqiure;
  };

  struct SearchNode {
    int lb;
    std::vector<BranchConstraint> constraints;
    int require_count;
    std::vector<std::pair<StopId, StopId>> refined_edges;
    std::unordered_map<std::pair<StopId, StopId>, int> parent_relaxed_weights;
    bool operator<(const SearchNode& other) const {
      return lb > other.lb;
      // return require_count < other.require_count;
    }
  };

  int best_ub = ub_rel;
  std::vector<SearchNode> q;
  q.push_back(SearchNode({
      .lb = 0,
      .constraints = {},
      .require_count = 0,
      .refined_edges = {},
      .parent_relaxed_weights = {},
  }));

  while (q.size() > 0) {
    std::pop_heap(q.begin(), q.end());
    SearchNode cur = std::move(q.back());
    q.pop_back();

    std::cout << "take " << TimeSinceServiceStart{cur.lb} << " ("
              << 1 + q.size() << " active)\n";
    if (cur.lb >= best_ub) {
      std::cout << "  pruned: exceed ub\n";
      continue;
    }

    std::vector<BranchConstraint> req_constraints;
    std::unordered_set<StepId> forbidden_steps;

    for (const BranchConstraint& constraint : cur.constraints) {
      if (constraint.reqiure) {
        req_constraints.push_back(constraint);
        std::cout << "  require ";
      } else {
        forbidden_steps.insert(constraint.step);
        std::cout << "  forbid ";
      }
      std::cout << constraint.step.DebugString(problem) << "\n";
    }

    std::ranges::sort(
        req_constraints, {}, [](const BranchConstraint& constraint) {
          return constraint.step.tb;
        }
    );

    std::vector<StepId> known_steps;
    known_steps.push_back({problem.boundary.start, s0, t0});
    for (const BranchConstraint& constraint : req_constraints) {
      known_steps.push_back(constraint.step);
    }

    RefinerState rs = GeneralizedMakeRefinerState(
        problem, completed, known_steps, forbidden_steps, cur.lb, best_ub
    );
    for (auto& [p, ps] : rs.pairs) {
      int embiggening_from_parent = cur.parent_relaxed_weights[p] - ps.MinDur();
      if (embiggening_from_parent > 0) {
        ps.local_embiggening = embiggening_from_parent;
      }
    }

    // std::vector<Step> completed_steps_constrained = completed_steps;
    // std::erase_if(completed_steps_constrained, [&](const Step& step) {
    //   for (const BranchConstraint& constraint : cur.constraints) {
    //     bool origin_match = step.origin.stop == constraint.step.a;
    //     bool dest_match = step.destination.stop == constraint.step.b;
    //     bool edge_match = origin_match && dest_match;
    //     bool full_match = edge_match && (
    //       step.destination.time == constraint.step.tb ||
    //       step.is_flex // TODO: This only works right now because there are
    //       no edges with flex and non-flex steps.
    //     );
    //     if (constraint.reqiure) {
    //       if (
    //         (edge_match && !full_match) ||
    //         (!edge_match && (origin_match || dest_match))
    //       ) {
    //         return true;
    //       }
    //     } else {
    //       if (full_match) {
    //         return true;
    //       }
    //     }
    //   }
    //   return false;
    // });
    // StepsAdjacencyList completed_constrained =
    // MakeAdjacencyList(completed_steps_constrained);

    // // Quick prune if there are any unsatisfiable requirements.
    // {
    //   std::vector<StepId> required_steps;
    //   for (const BranchConstraint& constraint : cur.constraints) {
    //     if (constraint.reqiure) {
    //       required_steps.push_back(constraint.step);
    //     }
    //   }
    //   std::ranges::sort(required_steps, {}, [](const StepId& step) {
    //     return step.tb;
    //   });

    //   bool pruned = false;
    //   for (int i = 0; i + 1 < required_steps.size(); ++i) {
    //     StopId a = required_steps[i].b;
    //     TimeSinceServiceStart ta = required_steps[i].tb;
    //     StopId b = required_steps[i + 1].b;
    //     TimeSinceServiceStart tb = required_steps[i + 1].tb;
    //     Step step_to_b = FindShortestPathsAtTime(completed_constrained, ta,
    //     a, {b})[b.v]; if (step_to_b.destination.time > tb) {
    //       std::cout
    //         << "pruned: "
    //         << problem.StopName(a) << "@" << ta << " -> "
    //         << problem.StopName(b) << "@" << step_to_b.destination.time <<
    //         " exceeds requirement " << tb << "\n";
    //       pruned = true;
    //       break;
    //     }
    //   }
    //   if (pruned) {
    //     continue;
    //   }
    // }

    // RefinerState rs = MakeRefinerState(problem, completed_constrained, s0,
    // t0, cur.lb, best_ub);

    std::optional<RefineResult> result =
        DoRefine(problem, rs, t0, best_ub, cur.refined_edges, true);
    if (!result.has_value()) {
      std::cout << "no result, pruning\n";
      continue;
    }
    {
      TourAnalysis analysis =
          AnalyzeTour(problem, rs, result->best_tour, t0, false);
      if (analysis.val_actual < best_ub) {
        std::cout << "Found new UB:\n";
        AnalyzeTour(problem, rs, result->best_tour, t0, true);
        best_ub = analysis.val_actual;
      }
    }
    TourAnalysis analysis = AnalyzeTour(problem, rs, result->tour, t0, false);
    std::cout << "result: " << TimeSinceServiceStart{analysis.val_relaxed}
              << " / " << TimeSinceServiceStart{analysis.val_actual}
              << " (best " << TimeSinceServiceStart{best_ub} << ")\n";
    if (analysis.val_relaxed >= best_ub) {
      std::cout << "  pruned: reached or exceeded ub\n";
      continue;
    }

    // Find the last forced edge.
    // int last_forced_i = -1;
    // for (int edge_i = 0; edge_i < result->tour.tour_edges.size(); ++edge_i) {
    //   const TarelEdge& edge = result->tour.tour_edges[edge_i];
    //   if (last_forced_i + 1 == edge_i) {
    //     int out_count = 0;
    //     for (const auto& [pair, _] : rs.pairs) {
    //       if (pair.first == edge.origin.stop) {
    //         out_count += 1;
    //       }
    //     }
    //     assert(out_count > 0);
    //     if (out_count == 1) {
    //       last_forced_i = edge_i;
    //     }
    //   }
    // }
    // if (last_forced_i + 1 == result->tour.tour_edges.size()) {
    //   std::cout << "whole tour forced!\n";
    //   return 0;
    // }

    // Print out all the forced edges.
    // std::cout << "forced edges (" << last_forced_i + 1 << "):\n";
    // for (int edge_i = 0; edge_i <= last_forced_i; ++edge_i) {
    //   const TarelEdge& edge = result->tour.tour_edges[edge_i];
    //   const std::vector<PairStep>& steps = rs.pairs.at({edge.origin.stop,
    //   edge.destination.stop}).steps; assert(steps.size() == 1); std::cout
    //     << "  " << problem.StopName(edge.origin.stop) << " @ " <<
    //     steps[0].origin_time
    //     << " -> " << problem.StopName(edge.destination.stop) << " @ " <<
    //     steps[0].DestinationTime() << "\n";
    // }

    // Require/forbid the next tour edge.
    // const TarelEdge& next_edge = result->tour.tour_edges[last_forced_i + 1];
    // const std::vector<PairStep>& next_edge_steps =
    // rs.pairs.at({next_edge.origin.stop, next_edge.destination.stop}).steps;
    // assert(next_edge_steps.size() == 1);
    // std::cout
    //   << "Next edge: "
    //   << problem.StopName(next_edge.origin.stop) << " @ " <<
    //   next_edge_steps[0].origin_time
    //   << " -> " << problem.StopName(next_edge.destination.stop) << " @ " <<
    //   next_edge_steps[0].DestinationTime() << "\n";
    // StepId next_step_id = StepId{next_edge.origin.stop,
    // next_edge.destination.stop, next_edge_steps[0].DestinationTime()};

    auto sorted_extra_delta = result->SortedExtraDelta();
    assert(sorted_extra_delta.size() > 0);
    std::cout << "Next edge: "
              << sorted_extra_delta[0].first.DebugString(problem) << ": "
              << TimeSinceServiceStart{sorted_extra_delta[0].second} << "\n";
    StepId next_step_id = sorted_extra_delta[0].first;

    std::unordered_map<std::pair<StopId, StopId>, int> parent_relaxed_weights;
    for (const auto& [p, ps] : result->refined_state.pairs) {
      parent_relaxed_weights[p] = ps.RelaxedWeight();
    }

    {
      SearchNode node_require = cur;
      node_require.lb = std::max(node_require.lb, result->tour.optimal_value);
      node_require.constraints.push_back(
          BranchConstraint{
              .step = next_step_id,
              .reqiure = true,
          }
      );
      node_require.require_count += 1;
      node_require.refined_edges = result->refined_edges;
      node_require.parent_relaxed_weights = parent_relaxed_weights;
      q.push_back(std::move(node_require));
      std::push_heap(q.begin(), q.end());
    }

    {
      SearchNode node_forbid = cur;
      node_forbid.lb = std::max(node_forbid.lb, result->tour.optimal_value);
      node_forbid.constraints.push_back(
          BranchConstraint{
              .step = next_step_id,
              .reqiure = false,
          }
      );
      node_forbid.refined_edges = result->refined_edges;
      node_forbid.parent_relaxed_weights = parent_relaxed_weights;
      q.push_back(std::move(node_forbid));
      std::push_heap(q.begin(), q.end());
    }
  }

  return 0;
}
