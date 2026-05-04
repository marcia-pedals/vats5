#include "embiggener.h"

#include <algorithm>
#include <asio/execution/any_executor.hpp>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
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

void RegisterPrimitivePath(
    const PathCache& cache, EmbiggenerState& state, int idx
) {
  const LocalEmbiggenState& p = state.primitive_paths[idx];
  std::span<const StopId> stops = cache.Get(p.path_index);
  state.by_front[p.Front(cache)].push_back(idx);
  state.by_back[p.Back(cache)].push_back(idx);
  for (std::size_t i = 0; i + 1 < stops.size(); ++i) {
    state.by_edge[state.EdgeIndex(PlainEdge{stops[i], stops[i + 1]})].push_back(
        idx
    );
  }
  ++state.num_active_primitive_paths;
}

// Marks the slot as tombstoned. Index entries are NOT removed; iteration sites
// must skip tombstoned slots (LocalEmbiggenState::IsTombstone()).
void UnregisterPrimitivePath(EmbiggenerState& state, int /*idx*/) {
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

struct PathCache::Impl {
  // All interned paths concatenated.
  std::vector<StopId> storage;
  // offsets[i] is the start index of path i in `storage`. Path i occupies
  // storage[offsets[i] .. offsets[i+1]). Has size num_paths + 1, with the
  // sentinel offsets[0] = 0 set on construction.
  std::vector<int> offsets{0};

  // Hashes/compares paths by dereferencing into storage/offsets via a stable
  // pointer to *this Impl (which lives on the heap and never moves).
  struct Hash {
    const Impl* impl;
    std::size_t operator()(int i) const noexcept {
      const StopId* p = impl->storage.data() + impl->offsets[i];
      const StopId* e = impl->storage.data() + impl->offsets[i + 1];
      std::size_t seed = 0;
      while (p != e) {
        seed ^=
            std::hash<StopId>{}(*p) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        ++p;
      }
      return seed;
    }
  };
  struct Eq {
    const Impl* impl;
    bool operator()(int i, int j) const noexcept {
      int ia = impl->offsets[i], ib = impl->offsets[i + 1];
      int ja = impl->offsets[j], jb = impl->offsets[j + 1];
      if ((ib - ia) != (jb - ja)) return false;
      return std::equal(
          impl->storage.begin() + ia,
          impl->storage.begin() + ib,
          impl->storage.begin() + ja
      );
    }
  };
  std::unordered_set<int, Hash, Eq> index_set;

  Impl() : index_set(0, Hash{this}, Eq{this}) {}
};

PathCache::PathCache() : impl_(std::make_unique<Impl>()) {}
PathCache::~PathCache() = default;
PathCache::PathCache(PathCache&&) noexcept = default;
PathCache& PathCache::operator=(PathCache&&) noexcept = default;

int PathCache::Intern(std::span<const StopId> path) {
  Impl& im = *impl_;
  int new_idx = static_cast<int>(im.offsets.size()) - 1;
  // Tentatively append the path. (`path` must not alias `im.storage`.)
  im.storage.insert(im.storage.end(), path.begin(), path.end());
  im.offsets.push_back(static_cast<int>(im.storage.size()));
  auto [it, inserted] = im.index_set.insert(new_idx);
  if (!inserted) {
    // Existing entry matches; roll back the tentative append.
    im.storage.resize(im.offsets[new_idx]);
    im.offsets.pop_back();
  }
  return *it;
}

std::span<const StopId> PathCache::Get(int index) const {
  const Impl& im = *impl_;
  return std::span<const StopId>(
      im.storage.data() + im.offsets[index],
      im.storage.data() + im.offsets[index + 1]
  );
}

void PathCache::PrintMemStats() const {
  auto si = [](std::size_t n) -> std::string {
    double v = static_cast<double>(n);
    const char* prefix = "";
    if (n >= 1'000'000'000) {
      v /= 1e9;
      prefix = "G";
    } else if (n >= 1'000'000) {
      v /= 1e6;
      prefix = "M";
    } else if (n >= 1'000) {
      v /= 1e3;
      prefix = "k";
    } else {
      return std::to_string(n);
    }
    int precision = v >= 100 ? 0 : (v >= 10 ? 1 : 2);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << v << prefix;
    return oss.str();
  };

  const Impl& im = *impl_;
  std::size_t num_paths = im.offsets.size() - 1;
  std::cout << "  path cache: " << si(num_paths) << " paths\n";

  // storage: 4 bytes per StopId.
  std::size_t storage_bytes = 4 * im.storage.size();
  std::cout << "    storage: " << si(storage_bytes) << "b\n";

  // offsets: 4 bytes per int.
  std::size_t offsets_bytes = 4 * im.offsets.size();
  std::cout << "    offsets: " << si(offsets_bytes) << "b\n";

  // index_set: ~24 bytes per node (next ptr + cached hash + int with padding)
  // plus ~8 bytes per bucket slot at default load factor 1.0.
  std::size_t set_bytes = 32 * num_paths;
  std::cout << "    index_set: " << si(set_bytes) << "b\n";

  std::cout << "    TOTAL: " << si(storage_bytes + offsets_bytes + set_bytes)
            << "b\n";
}

void EmbiggenerState::Compact(const PathCache& cache) {
  std::vector<LocalEmbiggenState> new_paths;
  new_paths.reserve(num_active_primitive_paths);
  for (LocalEmbiggenState& p : primitive_paths) {
    if (p.IsTombstone()) continue;
    new_paths.push_back(std::move(p));
  }
  primitive_paths = std::move(new_paths);
  by_front.clear();
  by_back.clear();
  for (std::vector<int>& bucket : by_edge) bucket.clear();
  num_active_primitive_paths = 0;
  for (int idx = 0; idx < static_cast<int>(primitive_paths.size()); ++idx) {
    RegisterPrimitivePath(cache, *this, idx);
  }
}

void EmbiggenerState::PrintMemStats(const PathCache& cache) const {
  auto si = [](std::size_t n) -> std::string {
    double v = static_cast<double>(n);
    const char* prefix = "";
    if (n >= 1'000'000'000) {
      v /= 1e9;
      prefix = "G";
    } else if (n >= 1'000'000) {
      v /= 1e6;
      prefix = "M";
    } else if (n >= 1'000) {
      v /= 1e3;
      prefix = "k";
    } else {
      return std::to_string(n);
    }
    int precision = v >= 100 ? 0 : (v >= 10 ? 1 : 2);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << v << prefix;
    return oss.str();
  };

  std::cout << "  active paths: " << si(num_active_primitive_paths) << "\n";

  std::size_t data_bytes = 0;
  std::set<int> distinct_path_indexes;
  std::set<std::tuple<int, int, int>> distinct_active_keys;
  std::size_t active_count = 0;
  for (const LocalEmbiggenState& p : primitive_paths) {
    // path_index + delta + t_front + t_back = 16 bytes
    data_bytes += 16;
    if (p.IsTombstone()) continue;
    distinct_path_indexes.insert(p.path_index);
    distinct_active_keys.insert(
        std::make_tuple(p.path_index, p.t_front.seconds, p.t_back.seconds)
    );
    ++active_count;
  }
  std::cout << "    data: " << si(data_bytes) << "b\n";
  std::cout << "      distinct path count: " << si(distinct_path_indexes.size())
            << "\n";
  std::cout << "      duplicate active LES: "
            << si(active_count - distinct_active_keys.size()) << "\n";

  std::size_t edge_index_bytes = 0;
  for (const std::vector<int>& v : by_edge) {
    edge_index_bytes += 24 + 4 * v.size();
  }
  std::cout << "    edge index: " << si(edge_index_bytes) << "b\n";

  std::size_t endpoint_index_bytes = 0;
  for (const auto& [e, v] : by_front) {
    // 52 bytes of unordered_map bookkeeping
    // 24 bytes of vector bookkeeping
    // vector data = 4 bytes per entry
    endpoint_index_bytes += 52 + 24 + 4 * v.size();
  }
  for (const auto& [e, v] : by_back) {
    // 52 bytes of unordered_map bookkeeping
    // 24 bytes of vector bookkeeping
    // vector data = 4 bytes per entry
    endpoint_index_bytes += 52 + 24 + 4 * v.size();
  }
  std::cout << "    endpoint index: " << si(endpoint_index_bytes) << "b\n";

  std::cout << "    TOTAL: "
            << si(data_bytes + edge_index_bytes + endpoint_index_bytes)
            << "b\n";
}

EmbiggenerState MakeEmbiggenerState(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    PathCache& cache,
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
      .by_edge = std::vector<std::vector<int>>(edge_index_size),
      .num_stops_for_edge_index = num_stops_for_edge_index,
  };
  for (const auto& [edge, steps] : steps_per_edge) {
    int weight = state.edges.at(edge).weight;
    StopId edge_stops[2] = {edge.a, edge.b};
    int edge_path_index = cache.Intern(edge_stops);
    for (const FlatStep& step : steps) {
      int idx = static_cast<int>(state.primitive_paths.size());
      state.primitive_paths.push_back(
          LocalEmbiggenState{
              .path_index = edge_path_index,
              .t_front = step.origin_time,
              .t_back = step.destination_time,
              .delta = step.DurationSeconds() - weight,
          }
      );
      RegisterPrimitivePath(cache, state, idx);
    }
  }
  return state;
}

EmbiggenerState ConstrainEmbiggenerState(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    PathCache& cache,
    const EmbiggenerState& state,
    const std::vector<PointBound>& known_points,
    const std::unordered_set<PointInstant>& forbidden_points
) {
  assert(known_points.size() > 0);
  assert(known_points.front().s == problem.boundary.start);
  assert(known_points.back().s == problem.boundary.end);
  // Don't check the ->END edge because the END t_lo might be lower than what we
  // actually get to.
  // TODO: Cleaner way of doing this? e.g. increasing the END t_lo when we know
  // it's too low.
  for (int i = 0; i + 2 < known_points.size(); ++i) {
    if (!(known_points[i].t_lo <= known_points[i + 1].t_lo) ||
        !(known_points[i].t_hi <= known_points[i + 1].t_hi)) {
      std::cout << problem.StopName(known_points[i].s) << " "
                << problem.StopName(known_points[i + 1].s) << "\n"
                << TimeSinceServiceStart{known_points[i].t_lo} << " "
                << TimeSinceServiceStart{known_points[i + 1].t_lo} << "\n"
                << TimeSinceServiceStart{known_points[i].t_hi} << " "
                << TimeSinceServiceStart{known_points[i + 1].t_hi} << "\n";
      assert(false);
    }
  }

  std::unordered_set<StopId> known_point_stops;
  known_point_stops.reserve(known_points.size());
  for (const PointBound& known : known_points) {
    known_point_stops.insert(known.s);
  }

  // Walks `path` starting from kpi=`start_kpi`. Returns the kpi after the walk
  // if every transition along the path satisfies the known/forbidden
  // constraints (mirroring the per-step rules in MakeEmbiggenerState's BFS),
  // or nullopt if any transition is rejected. Intermediate times are recovered
  // by stepping forward from path.t_front via GetTNext.
  auto WalkPath = [&](const LocalEmbiggenState& path,
                      int start_kpi) -> std::optional<int> {
    std::span<const StopId> stops = cache.Get(path.path_index);
    int kpi = start_kpi;
    TimeSinceServiceStart t_cur = path.t_front;
    for (std::size_t i = 1; i < stops.size(); ++i) {
      if (kpi == static_cast<int>(known_points.size()) - 1) {
        return std::nullopt;
      }
      const StepGroup* g_next = nullptr;
      for (const StepGroup& g : completed.GetGroups(stops[i - 1])) {
        if (g.destination_stop == stops[i]) {
          g_next = &g;
          break;
        }
      }
      if (g_next == nullptr) return std::nullopt;
      auto [t_next, _] = GetTNext(completed, *g_next, stops[i - 1], t_cur);
      PointInstant next{stops[i], t_next};
      const PointBound& next_kp = known_points[kpi + 1];
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
      t_cur = t_next;
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
    for (int primitive_path_i : by_front_it->second) {
      const LocalEmbiggenState& primitive_path =
          state.primitive_paths[primitive_path_i];
      if (primitive_path.IsTombstone()) continue;
      std::optional<int> end_kpi =
          WalkPath(primitive_path, cur_state.last_known_point_index);
      if (!end_kpi.has_value()) continue;
      primitive_path_reachable[primitive_path_i] = true;
      primitive_path_start_kpi[primitive_path_i] =
          cur_state.last_known_point_index;
      primitive_path_end_kpi[primitive_path_i] = *end_kpi;
      BFSState next_state{primitive_path.Back(cache), *end_kpi};
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
    for (int pi : by_back_it->second) {
      if (!primitive_path_reachable[pi] ||
          primitive_path_end_kpi[pi] != cur_state.last_known_point_index) {
        continue;
      }
      BFSState good_state{
          state.primitive_paths[pi].Front(cache), primitive_path_start_kpi[pi]
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
                state.primitive_paths[pi].Back(cache),
                primitive_path_end_kpi[pi]
            }
        )) {
      continue;
    }
    const LocalEmbiggenState& path = state.primitive_paths[pi];
    int new_idx = static_cast<int>(result.primitive_paths.size());
    result.primitive_paths.push_back(path);
    RegisterPrimitivePath(cache, result, new_idx);
    std::span<const StopId> stops = cache.Get(path.path_index);
    for (std::size_t i = 0; i + 1 < stops.size(); ++i) {
      used_edges.insert(PlainEdge{stops[i], stops[i + 1]});
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
    PathCache& cache,
    EmbiggenerState& state,
    PlainEdge edge_to_embiggen
) {
  auto ExtendForwards =
      [&](const LocalEmbiggenState& target) -> std::vector<LocalEmbiggenState> {
    // Copy target stops because cache.Intern() below may reallocate the
    // PathCache backing storage and invalidate a span held across iterations.
    std::span<const StopId> target_span = cache.Get(target.path_index);
    std::vector<StopId> target_stops(target_span.begin(), target_span.end());
    AssertOrRaise(target_stops.back() != problem.boundary.end);
    std::vector<LocalEmbiggenState> result;
    auto it = state.by_front.find(target.Back(cache));
    if (it == state.by_front.end()) return result;

    // Joined path = target_stops + candidate_stops[1:]. We need to detect
    // whether any stop in candidate_stops[1:] also appears in target_stops.
    // Precompute target_stops as a set once.
    std::unordered_set<StopId> stops_to_avoid;
    for (StopId s : target_stops) stops_to_avoid.insert(s);

    const bool target_starts_at_start =
        target_stops.front() == problem.boundary.start;

    for (int cand_idx : it->second) {
      const LocalEmbiggenState& candidate = state.primitive_paths[cand_idx];
      if (candidate.IsTombstone()) continue;
      // Span is valid within this iteration: cache.Intern() only runs at the
      // end, after we've finished reading candidate_stops.
      std::span<const StopId> candidate_stops = cache.Get(candidate.path_index);

      // If the joined path goes from START to END it must touch all stops.
      if (target_starts_at_start &&
          candidate_stops.back() == problem.boundary.end &&
          target_stops.size() + candidate_stops.size() !=
              problem.required.size() + 1) {
        continue;
      }

      // Stop overlap check (skip candidate_stops[0], which is the join point).
      bool overlaps = false;
      for (std::size_t i = 1; i < candidate_stops.size(); ++i) {
        if (stops_to_avoid.contains(candidate_stops[i])) {
          overlaps = true;
          break;
        }
      }
      if (overlaps) continue;

      std::vector<StopId> new_path;
      new_path.reserve(target_stops.size() + candidate_stops.size() - 1);
      new_path.insert(new_path.end(), target_stops.begin(), target_stops.end());
      new_path.insert(
          new_path.end(), candidate_stops.begin() + 1, candidate_stops.end()
      );
      result.push_back(
          LocalEmbiggenState{
              .path_index = cache.Intern(new_path),
              .t_front = target.t_front,
              .t_back = candidate.t_back,
              .delta = target.delta + candidate.delta,
          }
      );
    }
    return result;
  };

  auto ExtendBackwards = [&](const LocalEmbiggenState& target) {
    // Copy target stops because cache.Intern() below may reallocate the
    // PathCache backing storage and invalidate a span held across iterations.
    std::span<const StopId> target_span = cache.Get(target.path_index);
    std::vector<StopId> target_stops(target_span.begin(), target_span.end());
    AssertOrRaise(target_stops.front() != problem.boundary.start);
    std::vector<LocalEmbiggenState> result;
    auto it = state.by_back.find(target.Front(cache));
    if (it == state.by_back.end()) return result;

    // Joined path = candidate_stops + target_stops[1:]. We need to detect
    // whether any stop in target_stops[1:] appears in candidate_stops.
    // Precompute target_stops[1:] as a set once.
    std::unordered_set<StopId> stops_to_avoid;
    for (std::size_t i = 1; i < target_stops.size(); ++i) {
      stops_to_avoid.insert(target_stops[i]);
    }

    const bool target_ends_at_end = target_stops.back() == problem.boundary.end;

    for (int cand_idx : it->second) {
      const LocalEmbiggenState& candidate = state.primitive_paths[cand_idx];
      if (candidate.IsTombstone()) continue;
      // Span is valid within this iteration: cache.Intern() only runs at the
      // end, after we've finished reading candidate_stops.
      std::span<const StopId> candidate_stops = cache.Get(candidate.path_index);

      // If the joined path goes from START to END it must touch all stops.
      if (candidate_stops.front() == problem.boundary.start &&
          target_ends_at_end &&
          candidate_stops.size() + target_stops.size() !=
              problem.required.size() + 1) {
        continue;
      }

      // Stop overlap check (full candidate_stops; the join point at
      // candidate_stops.back() is checked here because target_stops[0] is NOT
      // in stops_to_avoid, so the shared join stop won't trigger a false hit).
      bool overlaps = false;
      for (StopId s : candidate_stops) {
        if (stops_to_avoid.contains(s)) {
          overlaps = true;
          break;
        }
      }
      if (overlaps) continue;

      std::vector<StopId> new_path;
      new_path.reserve(target_stops.size() + candidate_stops.size() - 1);
      new_path.insert(
          new_path.end(), candidate_stops.begin(), candidate_stops.end()
      );
      new_path.insert(
          new_path.end(), target_stops.begin() + 1, target_stops.end()
      );
      result.push_back(
          LocalEmbiggenState{
              .path_index = cache.Intern(new_path),
              .t_front = candidate.t_front,
              .t_back = target.t_back,
              .delta = candidate.delta + target.delta,
          }
      );
    }
    return result;
  };

  // Snapshot the indices of primitive paths that contain `edge_to_embiggen`,
  // since we will mutate the by_edge bucket while iterating. Skip tombstones.
  std::vector<int> through_edge;
  {
    const std::vector<int>& bucket =
        state.by_edge[state.EdgeIndex(edge_to_embiggen)];
    through_edge.reserve(bucket.size());
    for (int idx : bucket) {
      if (state.primitive_paths[idx].IsTombstone()) continue;
      through_edge.push_back(idx);
    }
  }

  int smallest_preextend_delta = std::numeric_limits<int>::max();
  for (int idx : through_edge) {
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
  for (int idx : through_edge) {
    LocalEmbiggenState& candidate = state.primitive_paths[idx];
    if (candidate.delta == smallest_preextend_delta) {
      UnregisterPrimitivePath(state, idx);
      critical_paths.push_back(std::move(candidate));
      candidate.path_index = -1;  // tombstone
    } else {
      smallest_noncritical_delta =
          std::min(smallest_noncritical_delta, candidate.delta);
    }
  }
  AssertOrRaise(critical_paths.size() > 0);

  std::vector<LocalEmbiggenState> extensions;
  for (const LocalEmbiggenState& critical_path : critical_paths) {
    std::span<const StopId> critical_stops =
        cache.Get(critical_path.path_index);
    std::vector<LocalEmbiggenState> new_extensions;
    if (critical_stops.front() == problem.boundary.start &&
        critical_stops.back() == problem.boundary.end) {
      // This is a full tour, so it can't be extended further.
      AssertOrRaise(critical_stops.size() == state.required.size());
      extensions.push_back(critical_path);
    } else if (critical_stops.front() == problem.boundary.start) {
      // Must extend forwards.
      new_extensions = ExtendForwards(critical_path);
    } else if (critical_stops.back() == problem.boundary.end) {
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
    int new_idx = static_cast<int>(state.primitive_paths.size());
    state.primitive_paths.push_back(std::move(extension));
    RegisterPrimitivePath(cache, state, new_idx);
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
  for (int idx : state.by_edge[state.EdgeIndex(edge_to_embiggen)]) {
    LocalEmbiggenState& primitive_path = state.primitive_paths[idx];
    if (primitive_path.IsTombstone()) continue;
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

std::optional<RefineResult> DoRefine(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    PathCache& cache,
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
    if (refine_round >= 2) {
      return RefineResult{lb, ub, ub_tour};
    }

    std::cout << "===== REFINE ROUND " << refine_round << " =====\n";
    refine_round += 1;
    state.PrintMemStats(cache);
    cache.PrintMemStats();

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
    if (!result.has_value()) {
      // TODO: Consider whether this is an expected situation.
      return std::nullopt;
    }
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
        std::optional<int> delta =
            LocalEmbiggenCorrect(problem, cache, state, target);
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
