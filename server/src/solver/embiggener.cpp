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
    const std::unordered_set<StepId>& forbidden_steps = {},
    bool include_nonzero_flex = false
) {
  StepPartitionId part_next_flex = StepPartitionId::NONE;
  TimeSinceServiceStart t_next_flex{std::numeric_limits<int>::max()};
  if (g_next.flex_step.has_value() &&
      (g_next.flex_step->FlexDurationSeconds() == 0 || include_nonzero_flex)) {
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

}  // namespace

namespace vats5 {

EmbiggenerState MakeEmbiggenerState(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    std::vector<PointBound> known_points,
    std::unordered_set<StepId> forbidden_steps,
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

  // Map from dest to all origins that reach it.
  std::unordered_map<PointInstant, std::vector<PointInstant>> steps_back;

  auto ExtendEdges = [&](int known_point_index, PointBound a, PointBound b) {
    std::vector<PointInstant> stack;
    std::unordered_set<PointInstant> visited;

    // Currently all points except the last one have t_lo == t_hi, so we only
    // handle this case. If we want intervals for non-last points, then we'll
    // have to handle that case by adding more to the stack.
    assert(a.t_lo == a.t_hi);
    stack.push_back(PointInstant{.s = a.s, .t = a.t_lo});

    // DFS all steps from a.
    while (stack.size() > 0) {
      PointInstant cur = stack.back();
      stack.pop_back();
      if (visited.contains(cur) || cur.s == b.s) {
        continue;
      }
      visited.insert(cur);
      for (const StepGroup& g_next : completed.GetGroups(cur.s)) {
        StopId s_next = g_next.destination_stop;
        // We can only go to a known_point_stop if it is b.
        if (known_point_stops.contains(s_next) && s_next != b.s) {
          continue;
        }
        auto [t_next, _] = GetTNext(
            completed,
            g_next,
            cur.s,
            cur.t,
            forbidden_steps,
            options.include_nonzero_flex
        );
        if (t_next > b.t_hi || (s_next == b.s && t_next < b.t_lo)) {
          // Time is out of bounds.
          continue;
        }
        steps_back[PointInstant{s_next, t_next}].push_back(
            PointInstant{cur.s, cur.t}
        );
        stack.push_back(PointInstant{s_next, t_next});
      }
    }
  };

  for (int i = 0; i + 1 < known_points.size(); ++i) {
    ExtendEdges(i, known_points[i], known_points[i + 1]);
  }

  // Get all the steps that eventually lead to END by doing a backwards DFS from
  // END. Steps that don't lead anywhere happen because the DFS in ExtendSteps
  // can't see whether a step will eventually reach the next known point within
  // its time bounds.
  std::unordered_map<PlainEdge, EmbiggenerEdge> result_edges;
  result_edges.reserve(problem.required.size() * problem.required.size());
  {
    std::vector<PointInstant> stack;
    std::unordered_set<PointInstant> visited;

    for (const auto& [dest, origins] : steps_back) {
      if (dest.s == problem.boundary.end) {
        stack.push_back(dest);
      }
    }

    while (stack.size() > 0) {
      PointInstant cur = stack.back();
      stack.pop_back();
      if (visited.contains(cur)) {
        continue;
      }
      visited.insert(cur);
      for (const PointInstant& origin : steps_back[cur]) {
        result_edges[PlainEdge{origin.s, cur.s}].steps.push_back(
            FlatStep{origin.t, cur.t}
        );
        stack.push_back(origin);
      }
    }
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

int Embiggen(int value) { return value + 1; }

}  // namespace vats5
