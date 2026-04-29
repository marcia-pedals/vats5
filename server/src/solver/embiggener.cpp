#include "embiggener.h"

#include <algorithm>
#include <asio/execution/any_executor.hpp>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

// Drop-in replacement for assert() that throws instead of aborting, so
// rapidcheck can catch the failure and shrink the input. To switch back to
// real assertions, find-and-replace `AssertOrRaise` with `assert`.
#define AssertOrRaise(expr) \
  ((expr) ? (void)0 : throw std::runtime_error("AssertOrRaise failed: " #expr))

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

std::vector<LocalEmbiggenState> BuildPrimitivePaths(
    const ProblemState& problem, const EmbiggenerState& state
) {
  std::vector<LocalEmbiggenState> primitive_paths;
  for (const auto& [edge, edge_data] : state.edges) {
    int smallest_delta = std::numeric_limits<int>::max();
    for (const FlatStep& step : edge_data.steps) {
      int delta = step.DurationSeconds() - edge_data.weight;
      assert(delta >= 0);
      smallest_delta = std::min(smallest_delta, delta);
      primitive_paths.push_back(
          LocalEmbiggenState{
              .path = {edge.a, edge.b},
              .t_front = step.origin_time,
              .t_back = step.destination_time,
              .delta = delta,
          }
      );
    }
    assert(smallest_delta == 0);
  }
  return primitive_paths;
}

std::optional<int> LocalEmbiggenCorrect(
    const ProblemState& problem,
    EmbiggenerState& state,
    std::vector<LocalEmbiggenState>& primitive_paths,
    PlainEdge edge_to_embiggen
) {
  auto Joinable = [&](const LocalEmbiggenState& p1,
                      const LocalEmbiggenState& p2) -> bool {
    // Paths must join at a stop and time.
    if (p1.path.back() != p2.path.front() || p1.t_back != p2.t_front) {
      return false;
    }

    // If the joined path goes from START to END, it must touch all the stops.
    if (p1.path.front() == problem.boundary.start &&
        p2.path.back() == problem.boundary.end &&
        p1.path.size() + p2.path.size() != problem.required.size() + 1) {
      return false;
    }

    // The joined path must not repeat any stops.
    // TODO: This check is embarassingly slow.
    std::unordered_set<StopId> p1_stops;
    for (StopId s : p1.path) {
      p1_stops.insert(s);
    }
    for (int i = 1; i < p2.path.size(); ++i) {
      if (p1_stops.contains(p2.path[i])) {
        return false;
      }
    }

    return true;
  };

  auto ExtendForwards =
      [&](const LocalEmbiggenState& target) -> std::vector<LocalEmbiggenState> {
    AssertOrRaise(target.path.back() != problem.boundary.end);
    std::vector<LocalEmbiggenState> result;
    for (const LocalEmbiggenState& candidate : primitive_paths) {
      if (!Joinable(target, candidate)) {
        continue;
      }
      std::vector<StopId> new_path;
      new_path.reserve(target.path.size() + candidate.path.size() - 1);
      new_path.insert(new_path.end(), target.path.begin(), target.path.end());
      new_path.insert(
          new_path.end(), candidate.path.begin() + 1, candidate.path.end()
      );
      result.push_back(
          LocalEmbiggenState{
              .path = std::move(new_path),
              .t_front = target.t_front,
              .t_back = candidate.t_back,
              .delta = target.delta + candidate.delta,
          }
      );
    }
    return result;
  };

  auto ExtendBackwards = [&](const LocalEmbiggenState& target) {
    AssertOrRaise(target.path.front() != problem.boundary.start);
    std::vector<LocalEmbiggenState> result;
    for (const LocalEmbiggenState& candidate : primitive_paths) {
      if (!Joinable(candidate, target)) {
        continue;
      }
      std::vector<StopId> new_path;
      new_path.reserve(target.path.size() + candidate.path.size() - 1);
      new_path.insert(
          new_path.end(), candidate.path.begin(), candidate.path.end()
      );
      new_path.insert(
          new_path.end(), target.path.begin() + 1, target.path.end()
      );
      result.push_back(
          LocalEmbiggenState{
              .path = std::move(new_path),
              .t_front = candidate.t_front,
              .t_back = target.t_back,
              .delta = candidate.delta + target.delta,
          }
      );
    }
    return result;
  };

  auto ContainsEdge = [](const std::vector<StopId> p,
                         const PlainEdge& edge) -> bool {
    for (int i = 0; i + 1 < p.size(); ++i) {
      if (p[i] == edge.a && p[i + 1] == edge.b) {
        return true;
      }
    }
    return false;
  };

  int smallest_preextend_delta = std::numeric_limits<int>::max();
  for (const LocalEmbiggenState& candidate : primitive_paths) {
    if (ContainsEdge(candidate.path, edge_to_embiggen)) {
      smallest_preextend_delta =
          std::min(smallest_preextend_delta, candidate.delta);
    }
  }
  if (smallest_preextend_delta == std::numeric_limits<int>::max()) {
    // There are no primitive paths through this edge! This is totally possible
    // -- embiggening a different edge might happen to disprove all primitive
    // paths through this edge.
    state.edges.erase(edge_to_embiggen);
    return std::nullopt;
  }

  int smallest_noncritical_delta = std::numeric_limits<int>::max();
  std::vector<LocalEmbiggenState> critical_paths;
  std::erase_if(primitive_paths, [&](LocalEmbiggenState& candidate) {
    if (ContainsEdge(candidate.path, edge_to_embiggen)) {
      if (candidate.delta == smallest_preextend_delta) {
        critical_paths.push_back(std::move(candidate));
        return true;
      }
      smallest_noncritical_delta =
          std::min(smallest_noncritical_delta, candidate.delta);
    }
    return false;
  });
  AssertOrRaise(critical_paths.size() > 0);

  std::vector<LocalEmbiggenState> extensions;
  for (const LocalEmbiggenState& critical_path : critical_paths) {
    std::vector<LocalEmbiggenState> new_extensions;
    if (critical_path.path.front() == problem.boundary.start &&
        critical_path.path.back() == problem.boundary.end) {
      // This is a full tour, so it can't be extended further.
      AssertOrRaise(critical_path.path.size() == state.required.size());
      extensions.push_back(critical_path);
    } else if (critical_path.path.front() == problem.boundary.start) {
      // Must extend forwards.
      new_extensions = ExtendForwards(critical_path);
    } else if (critical_path.path.back() == problem.boundary.end) {
      // Must extend backwards.
      new_extensions = ExtendBackwards(critical_path);
    } else {
      // We can arbitrarily choose which direction to extend.
      new_extensions = ExtendBackwards(critical_path);
    }
    for (LocalEmbiggenState& new_extension : new_extensions) {
      extensions.push_back(std::move(new_extension));
    }
  }

  int smallest_delta = smallest_noncritical_delta;
  for (LocalEmbiggenState& extension : extensions) {
    AssertOrRaise(ContainsEdge(extension.path, edge_to_embiggen));
    primitive_paths.push_back(std::move(extension));
    smallest_delta = std::min(smallest_delta, extension.delta);
  }

  // TODO: Consider. If smallest_delta==0, might it be better to leave
  // primitive_paths unchanged?

  if (smallest_delta == std::numeric_limits<int>::max()) {
    // This means that there are no feasible paths through `edge_to_embiggen` so
    // we can simply delete it from the state. (We've already deleted all
    // primitive paths through it, because we deleted all the critical primitive
    // paths and there are no non-critical primitive paths through it).
    state.edges.erase(edge_to_embiggen);
    return std::nullopt;
  }

  state.edges.at(edge_to_embiggen).weight += smallest_delta;
  for (LocalEmbiggenState& primitive_path : primitive_paths) {
    if (ContainsEdge(primitive_path.path, edge_to_embiggen)) {
      AssertOrRaise(primitive_path.delta >= smallest_delta);
      primitive_path.delta -= smallest_delta;
    }
  }

  return smallest_delta;
}

int LocalEmbiggenIterative(
    const ProblemState& problem,
    const EmbiggenerState& state,
    PlainEdge target,
    int ub_rel,
    int num_rounds
) {
  std::vector<LocalEmbiggenState> q;

  // The biggest delta of any state we've ever popped from the heap.
  int biggest_popped_delta = 0;

  auto Push = [&](LocalEmbiggenState s) {
    q.push_back(std::move(s));
    std::push_heap(q.begin(), q.end());
  };

  // Initialize the heap with all the target edge steps.
  auto target_edge_it = state.edges.find(target);
  assert(target_edge_it != state.edges.end());
  const EmbiggenerEdge& target_edge = target_edge_it->second;
  assert(target_edge.steps.size() > 0);
  q.reserve(target_edge.steps.size());
  for (const FlatStep& step : target_edge.steps) {
    Push(
        LocalEmbiggenState{
            .path = {target.a, target.b},
            .t_front = step.origin_time,
            .t_back = step.destination_time,
            .delta = step.DurationSeconds() - target_edge.weight,
        }
    );
  }

  for (int round = 0; round < num_rounds; ++round) {
    if (q.empty()) {
      // Exhaustive search complete with no full tour found, so target is on
      // no full tour and can be deleted.
      // TODO: Do deletion in a cleaner way than setting the weight to be
      // massive.
      return 30 * 24 * 3600;
    }
    std::pop_heap(q.begin(), q.end());
    LocalEmbiggenState cur = std::move(q.back());
    q.pop_back();
    biggest_popped_delta = std::max(biggest_popped_delta, cur.delta);

    if (cur.path.front() == problem.boundary.start &&
        cur.path.back() == problem.boundary.end) {
      // The tour must hit every stop.
      assert(cur.path.size() == state.required.size());
      return biggest_popped_delta;
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
        Push(
            LocalEmbiggenState{
                .path = std::move(new_path),
                .t_front = cur.t_front,
                .t_back = step_it->destination_time,
                .delta =
                    cur.delta + step_it->DurationSeconds() - edge_data.weight,
            }
        );
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
        // Multiple FlatSteps can share a destination_time (e.g. arrivals at
        // (b, 1201) reachable from both (c, 0) and (c, 1) via the same merged
        // step). All of them are valid backward extensions.
        auto first_it = std::ranges::lower_bound(
            edge_data.steps, cur.t_front, {}, [](const FlatStep& step) {
              return step.destination_time;
            }
        );
        auto last_it = std::ranges::upper_bound(
            edge_data.steps, cur.t_front, {}, [](const FlatStep& step) {
              return step.destination_time;
            }
        );
        for (auto step_it = first_it; step_it != last_it; ++step_it) {
          std::vector<StopId> new_path;
          new_path.reserve(cur.path.size() + 1);
          new_path.push_back(s);
          new_path.insert(new_path.end(), cur.path.begin(), cur.path.end());
          Push(
              LocalEmbiggenState{
                  .path = std::move(new_path),
                  .t_front = step_it->origin_time,
                  .t_back = cur.t_back,
                  .delta =
                      cur.delta + step_it->DurationSeconds() - edge_data.weight,
              }
          );
        }
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
      EmbiggenForwards();
    }
  }

  // Round budget exhausted without finding a full tour.
  return biggest_popped_delta;
}

int AnalyzeTour(
    const ProblemState& problem,
    const EmbiggenerState& state,
    TimeSinceServiceStart t0,
    const TspTourResult& tour
) {
  TimeSinceServiceStart t_cur = t0;
  for (const TarelEdge& edge : tour.tour_edges) {
    PlainEdge plain_edge{edge.origin.stop, edge.destination.stop};
    // std::cout << problem.StopName(plain_edge.a) << " -> " <<
    // problem.StopName(plain_edge.b) << "\n";
    const EmbiggenerEdge edge_data = state.edges.at(plain_edge);
    auto step_it = std::ranges::lower_bound(
        edge_data.steps, t_cur, {}, [](const FlatStep& step) {
          return step.origin_time;
        }
    );
    if (step_it == edge_data.steps.end() || step_it->origin_time != t_cur) {
      return std::numeric_limits<int>::max();
    }
    t_cur = step_it->destination_time;
  }
  return t_cur.seconds - t0.seconds;
}

std::optional<TspTourResult> DoRefine(
    const ProblemState& problem,
    EmbiggenerState& state,
    TimeSinceServiceStart t0,
    int ub_rel
) {
  std::vector<LocalEmbiggenState> primitive_paths =
      BuildPrimitivePaths(problem, state);

  int refine_round = 0;
  while (true) {
    std::cout << "===== REFINE ROUND " << refine_round << " =====\n";
    refine_round += 1;
    std::cout << "  primitive paths: " << primitive_paths.size() << "\n";

    std::optional<TspTourResult> result = DoTSP(problem, state, ub_rel);
    if (!result.has_value()) {
      return std::nullopt;
    }
    int t_actual = AnalyzeTour(problem, state, t0, *result);
    std::cout << "  tsp result: "
              << TimeSinceServiceStart{result->optimal_value} << " / "
              << TimeSinceServiceStart{t_actual} << "\n";

    int total_delta = 0;
    for (int round = 0; round < 1; ++round) {
      int round_delta = 0;
      for (const TarelEdge& edge : result->tour_edges) {
        PlainEdge target{edge.origin.stop, edge.destination.stop};
        std::optional<int> delta =
            LocalEmbiggenCorrect(problem, state, primitive_paths, target);
        // TODO: Handle nullopt as inf?
        if (delta.has_value()) {
          round_delta += *delta;
        }
      }
      std::cout << "  round " << round
                << " delta: " << TimeSinceServiceStart{round_delta} << "\n";
      total_delta += round_delta;
    }
    std::cout << "  total delta: " << TimeSinceServiceStart{total_delta}
              << "\n";
    if (total_delta == 0) {
      return result;
    }
  }
}

}  // namespace vats5
