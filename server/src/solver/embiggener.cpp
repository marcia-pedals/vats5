#include "embiggener.h"

#include <algorithm>
#include <asio/execution/any_executor.hpp>
#include <limits>
#include <map>
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

void RegisterPrimitivePath(EmbiggenerState& state, std::size_t idx) {
  const LocalEmbiggenState& p = state.primitive_paths[idx];
  state.by_front[p.path.front()].push_back(idx);
  state.by_back[p.path.back()].push_back(idx);
  for (std::size_t i = 0; i + 1 < p.path.size(); ++i) {
    state.by_edge[state.EdgeIndex(PlainEdge{p.path[i].s, p.path[i + 1].s})]
        .push_back(idx);
  }
  ++state.num_active_primitive_paths;
}

// Marks the slot as tombstoned. Index entries are NOT removed; iteration sites
// must skip slots whose `path` is empty.
void UnregisterPrimitivePath(EmbiggenerState& state, std::size_t /*idx*/) {
  --state.num_active_primitive_paths;
}

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

void EmbiggenerState::Compact() {
  std::vector<LocalEmbiggenState> new_paths;
  new_paths.reserve(num_active_primitive_paths);
  for (LocalEmbiggenState& p : primitive_paths) {
    if (p.path.empty()) continue;
    new_paths.push_back(std::move(p));
  }
  primitive_paths = std::move(new_paths);
  by_front.clear();
  by_back.clear();
  for (std::vector<std::size_t>& bucket : by_edge) bucket.clear();
  num_active_primitive_paths = 0;
  for (std::size_t idx = 0; idx < primitive_paths.size(); ++idx) {
    RegisterPrimitivePath(*this, idx);
  }
}

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

  std::unordered_map<PlainEdge, std::vector<FlatStep>> steps_per_edge;
  steps_per_edge.reserve(problem.required.size() * problem.required.size());
  for (const std::pair<PointInstant, PointInstant>& result_step :
       result_steps) {
    auto [a, b] = result_step;
    steps_per_edge[PlainEdge{a.s, b.s}].push_back(FlatStep{a.t, b.t});
  }

  std::unordered_map<PlainEdge, EmbiggenerEdge> result_edges;
  result_edges.reserve(steps_per_edge.size());
  for (auto& [edge, steps] : steps_per_edge) {
    std::sort(
        steps.begin(), steps.end(), [](const FlatStep& a, const FlatStep& b) {
          return a.origin_time < b.origin_time;
        }
    );
    for (size_t i = 1; i < steps.size(); ++i) {
      assert(steps[i - 1].destination_time <= steps[i].destination_time);
    }
    int min_duration = std::numeric_limits<int>::max();
    for (const FlatStep& step : steps) {
      min_duration = std::min(min_duration, step.DurationSeconds());
    }
    result_edges[edge] = EmbiggenerEdge{.weight = min_duration};
  }

  int max_stop_v = -1;
  for (const auto& [stop_id, _] : problem.stop_infos) {
    max_stop_v = std::max(max_stop_v, stop_id.v);
  }
  const int num_stops_for_edge_index = max_stop_v + 1;
  const std::size_t edge_index_size =
      static_cast<std::size_t>(num_stops_for_edge_index) *
      num_stops_for_edge_index;

  EmbiggenerState state{
      .required = problem.required,
      .edges = std::move(result_edges),
      .by_edge = std::vector<std::vector<std::size_t>>(edge_index_size),
      .num_stops_for_edge_index = num_stops_for_edge_index,
  };
  for (const auto& [edge, steps] : steps_per_edge) {
    int weight = state.edges.at(edge).weight;
    for (const FlatStep& step : steps) {
      std::size_t idx = state.primitive_paths.size();
      state.primitive_paths.push_back(
          LocalEmbiggenState{
              .path =
                  {PointInstant{edge.a, step.origin_time},
                   PointInstant{edge.b, step.destination_time}},
              .delta = step.DurationSeconds() - weight,
          }
      );
      RegisterPrimitivePath(state, idx);
    }
  }
  return state;
}

EmbiggenerState ConstrainEmbiggenerState(
    const ProblemState& problem,
    const EmbiggenerState& state,
    const std::vector<PointBound>& known_points,
    const std::unordered_set<PointInstant>& forbidden_points
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

  // Walks `path` starting from kpi=`start_kpi`. Returns the kpi after the walk
  // if every transition along the path satisfies the known/forbidden
  // constraints (mirroring the per-step rules in MakeEmbiggenerState's BFS),
  // or nullopt if any transition is rejected.
  auto WalkPath = [&](const std::vector<PointInstant>& path,
                      int start_kpi) -> std::optional<int> {
    int kpi = start_kpi;
    for (std::size_t i = 1; i < path.size(); ++i) {
      if (kpi == static_cast<int>(known_points.size()) - 1) {
        return std::nullopt;
      }
      const PointBound& next_kp = known_points[kpi + 1];
      const PointInstant& next = path[i];
      if (known_point_stops.contains(next.s) && next.s != next_kp.s) {
        return std::nullopt;
      }
      if (next.t > next_kp.t_hi) {
        return std::nullopt;
      }
      if (next.s == next_kp.s && next.t < next_kp.t_lo) {
        return std::nullopt;
      }
      if (forbidden_points.contains(next)) {
        return std::nullopt;
      }
      if (next.s == next_kp.s) {
        kpi += 1;
      }
    }
    return kpi;
  };

  // Phase 1: Forward BFS to discover all reachable states and primitive paths.
  // For each primitive path that starts at the current BFS state's point, walk
  // the path against the new constraints; if it survives, it produces a
  // transition to the new BFS state with the resulting kpi.
  std::vector<bool> primitive_path_reachable(
      state.primitive_paths.size(), false
  );
  std::vector<int> primitive_path_start_kpi(state.primitive_paths.size());
  std::vector<int> primitive_path_end_kpi(state.primitive_paths.size());
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
    auto by_front_it = state.by_front.find(cur_state.point);
    if (by_front_it == state.by_front.end()) continue;
    for (size_t primitive_path_i : by_front_it->second) {
      const LocalEmbiggenState& primitive_path =
          state.primitive_paths[primitive_path_i];
      if (primitive_path.path.empty()) {
        // tombstone
        continue;
      }
      std::optional<int> end_kpi =
          WalkPath(primitive_path.path, cur_state.last_known_point_index);
      if (!end_kpi.has_value()) continue;
      primitive_path_reachable[primitive_path_i] = true;
      primitive_path_start_kpi[primitive_path_i] =
          cur_state.last_known_point_index;
      primitive_path_end_kpi[primitive_path_i] = *end_kpi;
      BFSState next_state{primitive_path.Back(), *end_kpi};
      if (!discovered.contains(next_state)) {
        discovered.insert(next_state);
        worklist.push_back(next_state);
      }
    }
  }

  // Phase 2: Backwards BFS from end states to find all states that have a good
  // path (one that reaches an end state using only allowed primitive paths).
  std::unordered_set<BFSState> good_states;
  std::vector<BFSState> good_worklist;
  for (const BFSState& s : discovered) {
    if (s.last_known_point_index == static_cast<int>(known_points.size()) - 1) {
      good_states.insert(s);
      good_worklist.push_back(s);
    }
  }
  for (size_t gi = 0; gi < good_worklist.size(); ++gi) {
    BFSState cur_state = good_worklist[gi];
    auto by_back_it = state.by_back.find(cur_state.point);
    if (by_back_it == state.by_back.end()) continue;
    for (size_t pi : by_back_it->second) {
      if (!primitive_path_reachable[pi] ||
          primitive_path_end_kpi[pi] != cur_state.last_known_point_index) {
        continue;
      }
      BFSState good_state{
          state.primitive_paths[pi].Front(), primitive_path_start_kpi[pi]
      };
      if (!good_states.contains(good_state)) {
        good_states.insert(good_state);
        good_worklist.push_back(good_state);
      }
    }
  }

  // Phase 3: Keep reachable primitive paths that go to good states. Edges are
  // copied from the source state (their weights remain valid relaxations); any
  // edge that ends up with no surviving primitive path through it is dropped.
  EmbiggenerState result;
  result.required = state.required;
  result.num_stops_for_edge_index = state.num_stops_for_edge_index;
  result.by_edge.resize(state.by_edge.size());
  std::unordered_set<PlainEdge> used_edges;
  for (size_t pi = 0; pi < state.primitive_paths.size(); ++pi) {
    if (!primitive_path_reachable[pi]) continue;
    if (!good_states.contains(
            BFSState{
                state.primitive_paths[pi].Back(), primitive_path_end_kpi[pi]
            }
        )) {
      continue;
    }
    const LocalEmbiggenState& path = state.primitive_paths[pi];
    std::size_t new_idx = result.primitive_paths.size();
    result.primitive_paths.push_back(path);
    RegisterPrimitivePath(result, new_idx);
    for (std::size_t i = 0; i + 1 < path.path.size(); ++i) {
      used_edges.insert(PlainEdge{path.path[i].s, path.path[i + 1].s});
    }
  }
  for (const PlainEdge& edge : used_edges) {
    result.edges[edge] = state.edges.at(edge);
  }

  return result;
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

std::optional<int> LocalEmbiggenCorrect(
    const ProblemState& problem,
    EmbiggenerState& state,
    PlainEdge edge_to_embiggen
) {
  auto ExtendForwards =
      [&](const LocalEmbiggenState& target) -> std::vector<LocalEmbiggenState> {
    AssertOrRaise(target.path.back().s != problem.boundary.end);
    std::vector<LocalEmbiggenState> result;
    auto it = state.by_front.find(target.path.back());
    if (it == state.by_front.end()) return result;

    // Joined path = target.path + candidate.path[1:]. We need to detect whether
    // any stop in candidate.path[1:] also appears in target.path. Precompute
    // target.path as a set once.
    std::unordered_set<StopId> stops_to_avoid;
    for (const PointInstant& p : target.path) stops_to_avoid.insert(p.s);

    const bool target_starts_at_start =
        target.path.front().s == problem.boundary.start;

    for (std::size_t cand_idx : it->second) {
      const LocalEmbiggenState& candidate = state.primitive_paths[cand_idx];
      if (candidate.path.empty()) continue;  // tombstone

      // If the joined path goes from START to END it must touch all stops.
      if (target_starts_at_start &&
          candidate.path.back().s == problem.boundary.end &&
          target.path.size() + candidate.path.size() !=
              problem.required.size() + 1) {
        continue;
      }

      // Stop overlap check (skip candidate.path[0], which is the join point).
      bool overlaps = false;
      for (std::size_t i = 1; i < candidate.path.size(); ++i) {
        if (stops_to_avoid.contains(candidate.path[i].s)) {
          overlaps = true;
          break;
        }
      }
      if (overlaps) continue;

      std::vector<PointInstant> new_path;
      new_path.reserve(target.path.size() + candidate.path.size() - 1);
      new_path.insert(new_path.end(), target.path.begin(), target.path.end());
      new_path.insert(
          new_path.end(), candidate.path.begin() + 1, candidate.path.end()
      );
      result.push_back(
          LocalEmbiggenState{
              .path = std::move(new_path),
              .delta = target.delta + candidate.delta,
          }
      );
    }
    return result;
  };

  auto ExtendBackwards = [&](const LocalEmbiggenState& target) {
    AssertOrRaise(target.path.front().s != problem.boundary.start);
    std::vector<LocalEmbiggenState> result;
    auto it = state.by_back.find(target.path.front());
    if (it == state.by_back.end()) return result;

    // Joined path = candidate.path + target.path[1:]. We need to detect whether
    // any stop in target.path[1:] appears in candidate.path. Precompute
    // target.path[1:] as a set once.
    std::unordered_set<StopId> stops_to_avoid;
    for (std::size_t i = 1; i < target.path.size(); ++i) {
      stops_to_avoid.insert(target.path[i].s);
    }

    const bool target_ends_at_end =
        target.path.back().s == problem.boundary.end;

    for (std::size_t cand_idx : it->second) {
      const LocalEmbiggenState& candidate = state.primitive_paths[cand_idx];
      if (candidate.path.empty()) continue;  // tombstone

      // If the joined path goes from START to END it must touch all stops.
      if (candidate.path.front().s == problem.boundary.start &&
          target_ends_at_end &&
          candidate.path.size() + target.path.size() !=
              problem.required.size() + 1) {
        continue;
      }

      // Stop overlap check (full candidate.path; the join point at
      // candidate.path.back() is checked here because target.path[0] is NOT
      // in stops_to_avoid, so the shared join stop won't trigger a false hit).
      bool overlaps = false;
      for (const PointInstant& p : candidate.path) {
        if (stops_to_avoid.contains(p.s)) {
          overlaps = true;
          break;
        }
      }
      if (overlaps) continue;

      std::vector<PointInstant> new_path;
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
              .delta = candidate.delta + target.delta,
          }
      );
    }
    return result;
  };

  // Snapshot the indices of primitive paths that contain `edge_to_embiggen`,
  // since we will mutate the by_edge bucket while iterating. Skip tombstones.
  std::vector<std::size_t> through_edge;
  {
    const std::vector<std::size_t>& bucket =
        state.by_edge[state.EdgeIndex(edge_to_embiggen)];
    through_edge.reserve(bucket.size());
    for (std::size_t idx : bucket) {
      if (state.primitive_paths[idx].path.empty()) continue;
      through_edge.push_back(idx);
    }
  }

  int smallest_preextend_delta = std::numeric_limits<int>::max();
  for (std::size_t idx : through_edge) {
    smallest_preextend_delta =
        std::min(smallest_preextend_delta, state.primitive_paths[idx].delta);
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
  for (std::size_t idx : through_edge) {
    LocalEmbiggenState& candidate = state.primitive_paths[idx];
    if (candidate.delta == smallest_preextend_delta) {
      UnregisterPrimitivePath(state, idx);
      critical_paths.push_back(std::move(candidate));
      candidate.path.clear();  // tombstone
    } else {
      smallest_noncritical_delta =
          std::min(smallest_noncritical_delta, candidate.delta);
    }
  }
  AssertOrRaise(critical_paths.size() > 0);

  std::vector<LocalEmbiggenState> extensions;
  for (const LocalEmbiggenState& critical_path : critical_paths) {
    std::vector<LocalEmbiggenState> new_extensions;
    if (critical_path.path.front().s == problem.boundary.start &&
        critical_path.path.back().s == problem.boundary.end) {
      // This is a full tour, so it can't be extended further.
      AssertOrRaise(critical_path.path.size() == state.required.size());
      extensions.push_back(critical_path);
    } else if (critical_path.path.front().s == problem.boundary.start) {
      // Must extend forwards.
      new_extensions = ExtendForwards(critical_path);
    } else if (critical_path.path.back().s == problem.boundary.end) {
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
    int delta = extension.delta;
    std::size_t new_idx = state.primitive_paths.size();
    state.primitive_paths.push_back(std::move(extension));
    RegisterPrimitivePath(state, new_idx);
    smallest_delta = std::min(smallest_delta, delta);
  }

  // TODO: Consider. If smallest_delta==0, might it be better to leave
  // primitive_paths unchanged?
  // Hmm, seems not. It helps a lot to extend even ones that have
  // smalles_delta==0. Maybe it adds to other nearby ones and eventually helps
  // or something? Or like there are a bunch of delta>0 ones even if there is
  // one delta==0?

  if (smallest_delta == std::numeric_limits<int>::max()) {
    // This means that there are no feasible paths through `edge_to_embiggen` so
    // we can simply delete it from the state. (We've already deleted all
    // primitive paths through it, because we deleted all the critical primitive
    // paths and there are no non-critical primitive paths through it).
    state.edges.erase(edge_to_embiggen);
    return std::nullopt;
  }

  state.edges.at(edge_to_embiggen).weight += smallest_delta;
  for (std::size_t idx : state.by_edge[state.EdgeIndex(edge_to_embiggen)]) {
    LocalEmbiggenState& primitive_path = state.primitive_paths[idx];
    if (primitive_path.path.empty()) continue;  // tombstone
    AssertOrRaise(primitive_path.delta >= smallest_delta);
    primitive_path.delta -= smallest_delta;
  }

  return smallest_delta;
}

// Walks `tour` from `t0` and returns the trajectory of (stop, time) instants.
// Returns an empty vector if the tour is infeasible at the given t0.
std::vector<PointInstant> AnalyzeTour(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    TimeSinceServiceStart t0,
    const TspTourResult& tour
) {
  std::vector<PointInstant> trajectory;
  if (tour.tour_edges.empty()) {
    return trajectory;
  }
  TimeSinceServiceStart t_cur = t0;
  trajectory.push_back(PointInstant{tour.tour_edges[0].origin.stop, t_cur});
  for (const TarelEdge& edge : tour.tour_edges) {
    StopId origin = edge.origin.stop;
    StopId destination = edge.destination.stop;
    const StepGroup* g_next = nullptr;
    for (const StepGroup& g : completed.GetGroups(origin)) {
      if (g.destination_stop == destination) {
        g_next = &g;
        break;
      }
    }
    if (g_next == nullptr) {
      return {};
    }
    auto [t_next, _] = GetTNext(completed, *g_next, origin, t_cur);
    if (t_next.seconds == std::numeric_limits<int>::max()) {
      return {};
    }
    t_cur = t_next;
    trajectory.push_back(PointInstant{destination, t_cur});
  }
  return trajectory;
}

RefineResult DoRefine(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    EmbiggenerState& state,
    TimeSinceServiceStart t0,
    int ub_rel
) {
  std::map<std::vector<StopId>, int> tour_counts;

  int lb = 0;
  int ub = std::numeric_limits<int>::max();
  std::vector<PointInstant> ub_tour;

  int refine_round = 0;
  while (true) {
    if (refine_round >= 10) {
      return RefineResult{lb, ub, ub_tour};
    }

    std::cout << "===== REFINE ROUND " << refine_round << " =====\n";
    refine_round += 1;
    std::cout << "  primitive paths: " << state.num_active_primitive_paths
              << "\n";

    int effective_ub = ub_rel;
    for (const auto& [tour, count] : tour_counts) {
      int w = 0;
      bool feasible = true;
      for (std::size_t i = 0; i + 1 < tour.size(); ++i) {
        auto it = state.edges.find(PlainEdge{tour[i], tour[i + 1]});
        if (it == state.edges.end()) {
          feasible = false;
          break;
        }
        w += it->second.weight;
      }
      if (feasible) {
        effective_ub = std::min(effective_ub, w + 1);
      }
    }

    std::optional<TspTourResult> result = DoTSP(problem, state, effective_ub);
    AssertOrRaise(result.has_value());
    lb = result->optimal_value;

    std::vector<StopId> tour_stops;
    tour_stops.push_back(result->tour_edges[0].origin.stop);
    for (const TarelEdge& edge : result->tour_edges) {
      tour_stops.push_back(edge.destination.stop);
    }
    AssertOrRaise(tour_stops.front() == problem.boundary.start);
    AssertOrRaise(tour_stops.back() == problem.boundary.end);
    tour_counts[tour_stops] += 1;
    // std::cout << "  distinct tours: " << tour_counts.size() << "\n";

    std::vector<PointInstant> trajectory =
        AnalyzeTour(problem, completed, t0, *result);
    int t_actual = trajectory.empty()
                       ? std::numeric_limits<int>::max()
                       : trajectory.back().t.seconds - t0.seconds;
    std::cout << "  tsp result: "
              << TimeSinceServiceStart{result->optimal_value} << " / "
              << TimeSinceServiceStart{t_actual} << "\n";
    if (t_actual < ub) {
      ub = t_actual;
      ub_tour = std::move(trajectory);
    }

    std::optional<int> total_delta = 0;
    for (int round = 0; round < 1; ++round) {
      std::optional<int> round_delta = 0;
      for (const TarelEdge& edge : result->tour_edges) {
        PlainEdge target{edge.origin.stop, edge.destination.stop};
        std::optional<int> delta = LocalEmbiggenCorrect(problem, state, target);
        if (delta.has_value() && round_delta.has_value()) {
          *round_delta += *delta;
        } else {
          round_delta = std::nullopt;
        }
      }
      // std::cout << "  round " << round << " delta: ";
      // if (round_delta.has_value()) {
      //   std::cout << TimeSinceServiceStart{*round_delta};
      // } else {
      //   std::cout << "inf";
      // }
      // std::cout << "\n";
      if (round_delta.has_value() && total_delta.has_value()) {
        *total_delta += *round_delta;
      } else {
        total_delta = std::nullopt;
      }
    }
    // std::cout << "  total delta: ";
    // if (total_delta.has_value()) {
    //   std::cout << TimeSinceServiceStart{*total_delta};
    // } else {
    //   std::cout << "inf";
    // }
    // std::cout << "\n";
    if (total_delta == 0) {
      return RefineResult{lb, ub, ub_tour};
    }
  }
}

}  // namespace vats5
