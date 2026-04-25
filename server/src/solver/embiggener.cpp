#include "embiggener.h"

#include <algorithm>
#include <asio/execution/any_executor.hpp>
#include <unordered_map>
#include <unordered_set>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

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

struct BFSState {
  vats5::PointInstant point;
  int last_known_point_index;
  bool operator==(const BFSState&) const = default;
};

}  // namespace

template <>
struct std::hash<BFSState> {
  std::size_t operator()(const BFSState& v) const {
    std::size_t seed = std::hash<vats5::PointInstant>{}(v.point);
    seed ^= std::hash<int>{}(v.last_known_point_index) + 0x9e3779b9 +
            (seed << 6) + (seed >> 2);
    return seed;
  }
};

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

  // Phase 1: Forward BFS to discover all reachable states and edges.
  std::unordered_map<BFSState, std::vector<BFSState>> forward_edges;
  std::unordered_set<BFSState> discovered;
  std::vector<BFSState> worklist;

  assert(known_points[0].t_lo == known_points[0].t_hi);
  BFSState initial{PointInstant{known_points[0].s, known_points[0].t_lo}, 0};
  discovered.insert(initial);
  worklist.push_back(initial);

  for (size_t wi = 0; wi < worklist.size(); ++wi) {
    BFSState cur_state = worklist[wi];

    if (cur_state.last_known_point_index ==
        static_cast<int>(known_points.size()) - 1) {
      continue;
    }

    const PointBound& next_known_point =
        known_points[cur_state.last_known_point_index + 1];

    for (const StepGroup& g_next : completed.GetGroups(cur_state.point.s)) {
      StopId s_next = g_next.destination_stop;
      // We can only go to a known_point_stop if it is next_known_point.
      if (known_point_stops.contains(s_next) && s_next != next_known_point.s) {
        continue;
      }
      auto [t_next, _] = GetTNext(
          completed,
          g_next,
          cur_state.point.s,
          cur_state.point.t,
          options.include_nonzero_flex
      );
      PointInstant next_point{s_next, t_next};
      if (next_point.t > next_known_point.t_hi ||
          (next_point.s == next_known_point.s &&
           next_point.t < next_known_point.t_lo) ||
          forbidden_points.contains(next_point)) {
        // Time is out of bounds.
        continue;
      }

      BFSState next_state{
          next_point,
          cur_state.last_known_point_index +
              (next_point.s == next_known_point.s ? 1 : 0)
      };

      forward_edges[cur_state].push_back(next_state);

      if (!discovered.contains(next_state)) {
        discovered.insert(next_state);
        worklist.push_back(next_state);
      }
    }
  }

  // Phase 2: Backward BFS from base-case states to find all states that
  // have a good path (one that hits all remaining known_points and reaches
  // END).
  std::unordered_map<BFSState, std::vector<BFSState>> reverse_edges;
  for (const auto& [from, children] : forward_edges) {
    for (const BFSState& to : children) {
      reverse_edges[to].push_back(from);
    }
  }

  std::unordered_set<BFSState> good_states;
  std::vector<BFSState> good_worklist;
  for (const BFSState& s : discovered) {
    if (s.last_known_point_index == static_cast<int>(known_points.size()) - 1) {
      good_states.insert(s);
      good_worklist.push_back(s);
    }
  }
  for (size_t gi = 0; gi < good_worklist.size(); ++gi) {
    auto it = reverse_edges.find(good_worklist[gi]);
    if (it == reverse_edges.end()) continue;
    for (const BFSState& parent : it->second) {
      if (!good_states.contains(parent)) {
        good_states.insert(parent);
        good_worklist.push_back(parent);
      }
    }
  }

  // Phase 3: Collect result_steps — edges from good states to good children.
  std::unordered_set<std::pair<PointInstant, PointInstant>> result_steps;
  for (const auto& [from, children] : forward_edges) {
    if (!good_states.contains(from)) continue;
    for (const BFSState& to : children) {
      if (good_states.contains(to)) {
        result_steps.insert({from.point, to.point});
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

std::optional<TspTourResult> DoTSP(
    const ProblemState& problem, const EmbiggenerState& state, int ub_rel
) {
  TarelState start_state{problem.boundary.start, StepPartitionId{0}};
  TarelState end_state{problem.boundary.end, StepPartitionId{0}};
  std::vector<TarelEdge> edges;

  // END->START edge.
  edges.push_back(
      TarelEdge{
          .origin = end_state,
          .destination = start_state,
          .weight = 0,
      }
  );

  // All the other edges.
  for (const auto& [plain_edge, edge_data] : state.edges) {
    edges.push_back(
        TarelEdge{
            .origin = TarelState{plain_edge.a, StepPartitionId{0}},
            .destination = TarelState{plain_edge.b, StepPartitionId{0}},
            .weight = edge_data.weight,
        }
    );
  }

  // SOLVE!!

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
  for (StopId representative : problem.required.GroupRepresentatives()) {
    if (!representatives_in_graph.contains(representative)) {
      std::cout << "  missing required, no solution\n";
      return std::nullopt;
    }
  }

  std::optional<TspTourResult> result = SolveTspAndExtractTour(
      edges, graph, problem.boundary, ub_rel, nullptr, nullptr
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

struct LocalEmbiggenState {
  // This state represents an actual path from path.front()@t_front to
  // path.back()@t_back.
  //
  // TODO: We can do a constant-size bitvector for much more speed.
  std::vector<StopId> path;
  TimeSinceServiceStart t_front;
  TimeSinceServiceStart t_back;

  // The excess duration of the actual path over the relaxed weight on the
  // graph.
  int delta;

  bool operator<(const LocalEmbiggenState& other) const {
    return delta > other.delta;
  }
};

int LocalEmbiggenIterative(
    const ProblemState& problem,
    const EmbiggenerState& state,
    PlainEdge target,
    int ub_rel,
    int num_rounds
) {
  std::vector<LocalEmbiggenState> q;

  // Initialize the heap with all the target edge steps.
  auto target_edge_it = state.edges.find(target);
  assert(target_edge_it != state.edges.end());
  const EmbiggenerEdge& target_edge = target_edge_it->second;
  assert(target_edge.steps.size() > 0);
  q.reserve(target_edge.steps.size());
  for (const FlatStep& step : target_edge.steps) {
    q.push_back(
        LocalEmbiggenState{
            .path = {target.a, target.b},
            .t_front = step.origin_time,
            .t_back = step.destination_time,
            .delta = step.DurationSeconds() - target_edge.weight,
        }
    );
  }
  std::make_heap(q.begin(), q.end());

  for (int round = 0; round < num_rounds; ++round) {
    std::pop_heap(q.begin(), q.end());
    LocalEmbiggenState cur = std::move(q.back());
    q.pop_back();

    if (cur.path.front() == problem.boundary.start &&
        cur.path.back() == problem.boundary.end) {
      // We have found an entire valid tour through a->b, so we can't embiggen
      // past it, so this is as good as we can do.

      // The tour must hit every stop.
      assert(cur.path.size() == state.required.size());

      return cur.delta;
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
      for (StopId s : state.required.AllFlat()) {
        if (
          // Can't go to a stop we've already visited.
          IsInCurPath(s) ||
          // Can't go forwards to START.
          s == problem.boundary.start ||
          // If the path starts at START and hasn't visited everything else, can't go to END.
          (cur.path.front() == problem.boundary.start && s == problem.boundary.end && cur.path.size() + 1 < state.required.size())
        ) {
          continue;
        }
        auto edge_data_it = state.edges.find(PlainEdge{cur.path.back(), s});
        if (edge_data_it == state.edges.end()) {
          continue;
        }
        const EmbiggenerEdge& edge_data = edge_data_it->second;
        auto step_it = std::ranges::lower_bound(
            edge_data.steps, cur.t_back, {}, [](const FlatStep& step) {
              return step.origin_time;
            }
        );
        if (step_it == edge_data.steps.end() ||
            step_it->origin_time != cur.t_back) {
          // TODO: Under what conditions can this happen? Assert them?
          continue;
        }
        std::vector<StopId> new_path;
        new_path.reserve(cur.path.size() + 1);
        new_path.insert(new_path.end(), cur.path.begin(), cur.path.end());
        new_path.push_back(s);
        q.push_back(
            LocalEmbiggenState{
                .path = std::move(new_path),
                .t_front = cur.t_front,
                .t_back = step_it->destination_time,
                .delta =
                    cur.delta + step_it->DurationSeconds() - edge_data.weight,
            }
        );
        std::push_heap(q.begin(), q.end());
      }
    };

    auto EmbiggenBackwards = [&]() {
      for (StopId s : state.required.AllFlat()) {
        if (
          // Can't go to a stop we've already visited.
          IsInCurPath(s) ||
          // Can't go backwards to END.
          s == problem.boundary.end ||
          // If the path ends at END and hasn't visited everything else, can't go to START.
          (cur.path.back() == problem.boundary.end && s == problem.boundary.start && cur.path.size() + 1 < state.required.size())
        ) {
          continue;
        }
        auto edge_data_it = state.edges.find(PlainEdge{s, cur.path.front()});
        if (edge_data_it == state.edges.end()) {
          continue;
        }
        const EmbiggenerEdge& edge_data = edge_data_it->second;
        auto step_it = std::ranges::lower_bound(
            edge_data.steps, cur.t_front, {}, [](const FlatStep& step) {
              return step.destination_time;
            }
        );
        if (step_it == edge_data.steps.end() ||
            step_it->destination_time != cur.t_front) {
          // TODO: Under what conditions can this happen? Assert them?
          continue;
        }
        std::vector<StopId> new_path;
        new_path.reserve(cur.path.size() + 1);
        new_path.push_back(s);
        new_path.insert(new_path.end(), cur.path.begin(), cur.path.end());
        q.push_back(
            LocalEmbiggenState{
                .path = std::move(new_path),
                .t_front = step_it->origin_time,
                .t_back = cur.t_back,
                .delta =
                    cur.delta + step_it->DurationSeconds() - edge_data.weight,
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

    // If the queue is empty, it means that we have pruned all paths through
    // a->b, so we can delete a->b from the problem.
    // TODO: Do deletion in a cleaner way than setting the weight to be massive.
    if (q.size() == 0) {
      return 30 * 24 * 3600;
    }
  }

  return std::max(0, q.front().delta);
}

std::optional<TspTourResult> DoRefine(
    const ProblemState& problem, EmbiggenerState& state, int ub_rel
) {
  int refine_round = 0;
  while (true) {
    std::cout << "===== REFINE ROUND " << refine_round << " =====\n";
    refine_round += 1;

    std::optional<TspTourResult> result = DoTSP(problem, state, ub_rel);
    if (!result.has_value()) {
      return std::nullopt;
    }
    std::cout << "  tsp result: "
              << TimeSinceServiceStart{result->optimal_value} << "\n";

    int total_delta = 0;
    for (int edge_i = result->tour_edges.size() - 1; edge_i >= 0; --edge_i) {
      const TarelEdge& edge = result->tour_edges[edge_i];
      PlainEdge target{edge.origin.stop, edge.destination.stop};

      int delta = LocalEmbiggenIterative(problem, state, target, ub_rel, 2000);
      total_delta += delta;
      state.edges.at(target).weight += delta;
    }
    std::cout << "  total delta: " << TimeSinceServiceStart{total_delta}
              << "\n";
    if (total_delta == 0) {
      return result;
    }
  }
}

}  // namespace vats5
