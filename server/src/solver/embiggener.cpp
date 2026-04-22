#include "embiggener.h"

#include <unordered_map>
#include <unordered_set>

#include "solver/data.h"
#include "solver/steps_shortest_path.h"

namespace {

using namespace vats5;

std::pair<TimeSinceServiceStart, StepPartitionId> GetTNext(
    const StepsAdjacencyList& completed,
    const StepGroup& g_next,
    StopId from_s,
    TimeSinceServiceStart t_cur,
    bool include_nonzero_flex = false
) {
  StepPartitionId part_next_flex = StepPartitionId::NONE;
  TimeSinceServiceStart t_next_flex{std::numeric_limits<int>::max()};
  if (g_next.flex_step.has_value() &&
      (g_next.flex_step->FlexDurationSeconds() == 0 || include_nonzero_flex)) {
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

}  // namespace

namespace vats5 {

EmbiggenerState MakeEmbiggenerState(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    std::vector<PointBound> known_points,
    std::unordered_set<PointInstant> forbidden_points,
    EmbiggenerOptions options
) {
  assert(known_points.size() > 0);
  assert(known_points.front().s == problem.boundary.start);
  assert(known_points.back().s == problem.boundary.end);
  for (int i = 0; i + 1 < known_points.size(); ++i) {
    assert(known_points[i].t_lo <= known_points[i + 1].t_lo);
    assert(known_points[i].t_hi <= known_points[i + 1].t_hi);
  }

  std::unordered_set<StopId> known_point_stops;
  known_point_stops.reserve(known_points.size());
  for (const PointBound& known : known_points) {
    known_point_stops.insert(known.s);
  }

  using State =
      std::pair<PointInstant, int>;  // (point, last_known_point_index)

  struct Edge {
    State to;
    PointInstant from_point;
    PointInstant to_point;
  };

  // Phase 1: Forward BFS to discover all reachable states and edges.
  std::unordered_map<State, std::vector<Edge>> forward_edges;
  std::unordered_set<State> discovered;
  std::vector<State> worklist;

  assert(known_points[0].t_lo == known_points[0].t_hi);
  State initial{PointInstant{known_points[0].s, known_points[0].t_lo}, 0};
  discovered.insert(initial);
  worklist.push_back(initial);

  for (size_t wi = 0; wi < worklist.size(); ++wi) {
    auto [cur, last_known_point_index] = worklist[wi];

    if (last_known_point_index == static_cast<int>(known_points.size()) - 1) {
      continue;
    }

    const PointBound& next_known_point =
        known_points[last_known_point_index + 1];

    for (const StepGroup& g_next : completed.GetGroups(cur.s)) {
      StopId s_next = g_next.destination_stop;
      // We can only go to a known_point_stop if it is next_known_point.
      if (known_point_stops.contains(s_next) && s_next != next_known_point.s) {
        continue;
      }
      auto [t_next, _] = GetTNext(
          completed, g_next, cur.s, cur.t, options.include_nonzero_flex
      );
      PointInstant next = PointInstant{s_next, t_next};
      if (next.t > next_known_point.t_hi ||
          (next.s == next_known_point.s && next.t < next_known_point.t_lo) ||
          forbidden_points.contains(next)) {
        // Time is out of bounds.
        continue;
      }

      int next_idx =
          last_known_point_index + (next.s == next_known_point.s ? 1 : 0);
      State next_state{next, next_idx};

      forward_edges[worklist[wi]].push_back({next_state, cur, next});

      if (!discovered.contains(next_state)) {
        discovered.insert(next_state);
        worklist.push_back(next_state);
      }
    }
  }

  // Phase 2: Backward BFS from base-case states to find all states that
  // have a good path (one that hits all remaining known_points and reaches
  // END).
  std::unordered_map<State, std::vector<State>> reverse_edges;
  for (const auto& [from, edges] : forward_edges) {
    for (const auto& edge : edges) {
      reverse_edges[edge.to].push_back(from);
    }
  }

  std::unordered_set<State> good_states;
  std::vector<State> good_worklist;
  for (const State& s : discovered) {
    if (s.second == static_cast<int>(known_points.size()) - 1) {
      good_states.insert(s);
      good_worklist.push_back(s);
    }
  }
  for (size_t gi = 0; gi < good_worklist.size(); ++gi) {
    auto it = reverse_edges.find(good_worklist[gi]);
    if (it == reverse_edges.end()) continue;
    for (const State& parent : it->second) {
      if (!good_states.contains(parent)) {
        good_states.insert(parent);
        good_worklist.push_back(parent);
      }
    }
  }

  // Phase 3: Collect result_steps — edges from good states to good children.
  std::unordered_set<std::pair<PointInstant, PointInstant>> result_steps;
  for (const auto& [from, edges] : forward_edges) {
    if (!good_states.contains(from)) continue;
    for (const auto& edge : edges) {
      if (good_states.contains(edge.to)) {
        result_steps.insert({edge.from_point, edge.to_point});
      }
    }
  }

  std::unordered_map<PlainEdge, EmbiggenerEdge> result_edges;
  result_edges.reserve(problem.required.size() * problem.required.size());
  for (const std::pair<PointInstant, PointInstant>& result_step :
       result_steps) {
    auto [a, b] = result_step;
    result_edges[PlainEdge{a.s, b.s}].steps.push_back(FlatStep{a.t, b.t});
  }

  for (auto& [_, edge] : result_edges) {
    std::sort(
        edge.steps.begin(),
        edge.steps.end(),
        [](const FlatStep& a, const FlatStep& b) {
          return a.origin_time < b.origin_time;
        }
    );
    for (size_t i = 1; i < edge.steps.size(); ++i) {
      assert(
          edge.steps[i - 1].destination_time <= edge.steps[i].destination_time
      );
    }
  }

  return EmbiggenerState{
      .required = problem.required,
      .edges = std::move(result_edges),
  };
}

}  // namespace vats5
