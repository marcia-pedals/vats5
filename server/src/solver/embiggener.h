#pragma once

#include <memory>
#include <random>
#include <set>
#include <span>
#include <unordered_map>
#include <unordered_set>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"
namespace vats5 {

// Interns stop sequences to contiguous integer indices: the same stop sequence
// always maps to the same index. Paths are stored once, in a single flat
// backing buffer; an index_set keyed by content (via a self-referential hasher)
// dedupes them.
struct PathCache {
  PathCache();
  ~PathCache();
  PathCache(PathCache&&) noexcept;
  PathCache& operator=(PathCache&&) noexcept;
  PathCache(const PathCache&) = delete;
  PathCache& operator=(const PathCache&) = delete;

  int Intern(std::span<const StopId> path);
  std::span<const StopId> Get(int index) const;

  void PrintMemStats() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct FlatStep {
  TimeSinceServiceStart origin_time;
  TimeSinceServiceStart destination_time;
  int DurationSeconds() const {
    return destination_time.seconds - origin_time.seconds;
  }
  bool operator==(const FlatStep&) const = default;
};

struct EmbiggenerEdge {
  int weight = 0;
  bool operator==(const EmbiggenerEdge&) const = default;
};

inline std::ostream& operator<<(std::ostream& os, const FlatStep& value) {
  return os << "FlatStep{" << value.origin_time.seconds << " -> "
            << value.destination_time.seconds << "}";
}

inline std::ostream& operator<<(std::ostream& os, const EmbiggenerEdge& value) {
  return os << "EmbiggenerEdge{w=" << value.weight << "}";
}

struct PointInstant {
  StopId s;
  TimeSinceServiceStart t;
  bool operator==(const PointInstant&) const = default;
  bool operator<(const PointInstant& other) const {
    if (s.v != other.s.v) return s.v < other.s.v;
    return t.seconds < other.t.seconds;
  }
};

struct TourCache {
  std::set<std::vector<PointInstant>> tours;
};

struct LocalEmbiggenState {
  // Index into a PathCache that holds the stop sequence. The path begins at
  // its first stop at t_front and ends at its last stop at t_back. Intermediate
  // times are not stored; recover them by walking forward from t_front via the
  // WalkStopsFromTFront helper (in embiggener.cpp). path_index == -1 indicates
  // a tombstoned slot that has been removed from the indexes.
  int path_index;
  TimeSinceServiceStart t_front;
  TimeSinceServiceStart t_back;

  // The excess duration of the actual path over the relaxed weight on the
  // graph.
  int delta;

  bool operator<(const LocalEmbiggenState& other) const {
    return delta > other.delta;
  }

  bool IsTombstone() const { return path_index < 0; }

  PointInstant Front(const PathCache& cache) const {
    return PointInstant{cache.Get(path_index).front(), t_front};
  }

  PointInstant Back(const PathCache& cache) const {
    return PointInstant{cache.Get(path_index).back(), t_back};
  }
};

}  // namespace vats5

template <>
struct std::hash<vats5::PointInstant> {
  std::size_t operator()(const vats5::PointInstant& v) const {
    std::size_t seed = std::hash<vats5::StopId>{}(v.s);
    seed ^=
        std::hash<int>{}(v.t.seconds) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

namespace vats5 {

struct EmbiggenerState {
  RequiredStops required;
  std::unordered_map<PlainEdge, EmbiggenerEdge> edges;

  // Storage for primitive paths. Slots may be tombstoned (path_index < 0) when
  // they have been removed from the indexes below.
  std::vector<LocalEmbiggenState> primitive_paths;

  // Indexes into primitive_paths. Buckets are append-only vectors: indices
  // added by RegisterPrimitivePath are never removed. Iteration sites must
  // skip tombstoned slots (IsTombstone()).
  // - by_front: maps path.front() to the indices whose path begins there at
  //   that time.
  // - by_back: maps path.back() similarly for path ends.
  // - by_edge: maps each PlainEdge to the indices of paths that contain it,
  //   indexed by EdgeIndex(edge). Buckets for edges that don't exist are
  //   simply empty.
  std::unordered_map<PointInstant, std::vector<int>> by_front;
  std::unordered_map<PointInstant, std::vector<int>> by_back;
  std::vector<std::vector<int>> by_edge;

  // Sized so that EdgeIndex(e) is a valid index into by_edge for every
  // PlainEdge e that appears in any primitive path. Set once by
  // MakeEmbiggenerState and then constant.
  int num_stops_for_edge_index = 0;

  std::size_t EdgeIndex(PlainEdge e) const {
    return static_cast<std::size_t>(e.a.v) * num_stops_for_edge_index + e.b.v;
  }

  // Number of non-tombstoned slots in primitive_paths. Maintained by
  // RegisterPrimitivePath / UnregisterPrimitivePath.
  std::size_t num_active_primitive_paths = 0;

  // Erases tombstoned slots from primitive_paths and rebuilds the indexes.
  void Compact(const PathCache& cache);

  void PrintMemStats(const PathCache& cache) const;
};

struct PointBound {
  StopId s;
  TimeSinceServiceStart t_lo;
  TimeSinceServiceStart t_hi;
};

struct StepId {
  StopId a;
  StopId b;
  TimeSinceServiceStart tb;
  bool operator==(const StepId&) const = default;
};

}  // namespace vats5

template <>
struct std::hash<vats5::StepId> {
  std::size_t operator()(const vats5::StepId& v) const {
    std::size_t seed = std::hash<vats5::StopId>{}(v.a);
    seed ^= std::hash<vats5::StopId>{}(v.b) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    seed ^=
        std::hash<int>{}(v.tb.seconds) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

template <typename A, typename B>
struct std::hash<std::pair<A, B>> {
  std::size_t operator()(const std::pair<A, B>& v) const {
    std::size_t seed = std::hash<A>{}(v.first);
    seed ^= std::hash<B>{}(v.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

namespace vats5 {

struct EmbiggenerOptions {
  bool include_nonzero_flex = false;
};

EmbiggenerState MakeEmbiggenerState(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    PathCache& cache,
    std::vector<PointBound> known_points,
    std::unordered_set<PointInstant> forbidden_points,
    EmbiggenerOptions options = {}
);

EmbiggenerState ConstrainEmbiggenerState(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    PathCache& cache,
    const EmbiggenerState& state,
    const std::vector<PointBound>& known_points,
    const std::unordered_set<PointInstant>& forbidden_points
);

std::optional<TspTourResult> DoTSP(
    const ProblemState& problem,
    const std::unordered_map<PlainEdge, EmbiggenerEdge>& edges,
    int ub_rel
);

// Embiggen `edge_to_embiggen` in `state`: returns the delta added to the
// edge's weight, updates `state.edges.at(edge_to_embiggen).weight` in place,
// and updates `state.primitive_paths` so subsequent calls remain correct.
//
// If it realizes that there are no feasible paths through `edge_to_embiggen`,
// returns nullopt. (i.e. treat nullopt as infinity).
std::optional<int> LocalEmbiggenCorrect(
    const ProblemState& problem,
    PathCache& cache,
    EmbiggenerState& state,
    PlainEdge edge_to_embiggen
);

struct RefineResult {
  int lb;  // the optimal value of the latest round DoTSP
  int ub;  // the lowest t_actual from any round
  std::vector<PointInstant> ub_tour;  // the tour stops (with t_cur times) from
                                      // the tour with the lowest t_actual
  std::vector<PointInstant> final_tour;  // the trajectory from the final refine
                                         // iteration (not necessarily ub_tour)
};

struct RandomWalkResult {
  // Sum of state.edges[e].weight over all edges in the walked path.
  int total_weight;
  // Time at which the walk reached boundary.end, minus t0.
  int duration_seconds;
};

// Samples a random walk from (boundary.start, t0) to boundary.end by repeatedly
// drawing a uniformly random primitive path from state.by_front at the current
// PointInstant and jumping to its back. Throws if no primitive path is
// available at some intermediate point.
RandomWalkResult SampleRandomWalk(
    const EmbiggenerState& state,
    const PathCache& cache,
    const ProblemBoundary& boundary,
    TimeSinceServiceStart t0,
    std::mt19937& rng
);

struct SampleStats {
  int p0;
  int p10;
  int p50;
  int p90;
  int p100;
  double mean;
};

struct RandomWalkSampleStats {
  int sample_count;
  SampleStats total_weight;
  SampleStats duration_seconds;
  SampleStats slack_seconds;  // duration_seconds - total_weight
};

// Draws `sample_count` random walks via SampleRandomWalk and returns summary
// stats over total_weight, duration_seconds, and (duration_seconds -
// total_weight).
RandomWalkSampleStats SampleRandomWalkStats(
    const EmbiggenerState& state,
    const PathCache& cache,
    const ProblemBoundary& boundary,
    TimeSinceServiceStart t0,
    int sample_count,
    std::mt19937& rng
);

std::optional<RefineResult> DoRefine(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    PathCache& cache,
    TourCache& tour_cache,
    EmbiggenerState& state,
    TimeSinceServiceStart t0,
    int ub_rel
);

int EstimateCost(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    const EmbiggenerState& state,
    const PathCache& cache
);

struct DeltaLBResult {
  int delta;
  std::vector<PointInstant> points;
};

DeltaLBResult DeltaLB(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    const EmbiggenerState& state,
    const PathCache& cache,
    TimeSinceServiceStart t0,
    int lb_rel,
    const std::vector<StopId>& forbid_repeat_stops
);

// Runs DeltaLB twice: first with no forbidden repeats; then re-runs with the
// two most-visited stops from the first result forbidden from being repeated.
DeltaLBResult DeltaLBWithRepeatFix(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    const EmbiggenerState& state,
    const PathCache& cache,
    TimeSinceServiceStart t0,
    int lb_rel
);

}  // namespace vats5
