#pragma once

#include <span>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

// Backtrack through the search results to reconstruct the full path.
// Returns the steps in order from origin to destination.
std::vector<Step> BacktrackPath(
    const std::vector<Step>& search_result, StopId dest
);

// Compute the merged step for a path, with proper origin time calculation.
// The origin time adjustment handles flex paths that transition to fixed trips.
Step ComputeMergedStep(const std::vector<Step>& path);

struct RelaxedDistances {
    // distance_to[dest_id][stop_id.v] is the relaxed distance from stop_id to dest_id.
  std::unordered_map<StopId, std::vector<int>> distance_to;
};

struct StopIdVectorHash {
  size_t operator()(const std::vector<StopId>& v) const {
    size_t seed = v.size();
    for (const StopId& s : v) {
      seed ^= std::hash<int>{}(s.v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// A cache of A*-like heuristics for FindShortestPathsAtTime.
struct HeuristicCache {
  const RelaxedDistances* relaxed_distances = nullptr;

  std::unordered_map<std::vector<StopId>, std::vector<int>, StopIdVectorHash>
      cache;

  HeuristicCache() = default;
  explicit HeuristicCache(const RelaxedDistances* rd)
      : relaxed_distances(rd) {}

  // Get or compute heuristic distances for a destination set.
  // Returns pointer to cached vector, or nullptr if no heuristic.
  const std::vector<int>* GetOrCompute(
      const std::unordered_set<StopId>& destinations
  );
};

// Find earliest times you can get to a set of stops from `origin` when starting
// at `time`.
//
// The return includes at least values for `destinations` and for all
// intermediate stops on the optimal routes to them. It may include values for
// other stops visited during the search.
//
// The returned steps may depart later than `time` from `origin`, but you should
// not rely on this to find the latest possible departure time (see the
// SuboptimalDepartureTimeExposure test).
//
// Returns a vector indexed by StopId.v. Unvisited stops have
// destination_time.seconds == numeric_limits<int>::max().
//
// If `smallest_next_departure_gap_from_flex` is specified, it'll be set to the
// smallest gap between any visited flex path from the origin and the next
// departure that departs strictly after the visited flex path visits the stop.
// This is useful if you are scanning forwards to find all best paths from A->B,
// and you get a flex result, then you know that the result continues to be flex
// up at least until `time + smallest_next_departure_gap_from_flex`. It gets set
// to `std::numeric_limits<int>::max()` if there are no departures after any
// visited flex paths.
//
// If `heuristic_cache` is specified, enables A* optimization.
std::vector<Step> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list,
    TimeSinceServiceStart time,
    StopId origin,
    const std::unordered_set<StopId>& destinations,
    int* smallest_next_departure_gap_from_flex = nullptr,
    HeuristicCache* heuristic_cache = nullptr
);

// Return a minimal set of paths from `origin` to destinations, with origin
// times in [origin_time_lb, origin_time_ub). Return value is a map from
// destination to the paths from `origin` to that destination.
//
// Minimal means that these properties hold:
// (a) Any path in `adjacency_list` from `origin` to a destination can be
// matched or beat by a path in the return. (b) If you remove any path from the
// return value, (a) no longer holds.
std::unordered_map<StopId, std::vector<Path>> FindMinimalPathSet(
    const StepsAdjacencyList& adjacency_list,
    StopId origin,
    const std::unordered_set<StopId>& destinations,
    TimeSinceServiceStart origin_time_lb = TimeSinceServiceStart{0},
    TimeSinceServiceStart origin_time_ub = TimeSinceServiceStart{36 * 3600},
    const RelaxedDistances* relaxed_distances = nullptr
);

// Return an adjacency list ("reduced list") with these properties:
// (a) Any path in `adjacency_list` between two `system_stops` [1] can be
// matched or beat by a path in the reduced list. (b) If you remove any step
// from the reduced list, (a) no longer holds.
//
// [1] Usually-unimportant qualification: All departures from `system_stops` in
// the path happen at <36:00.
StepPathsAdjacencyList ReduceToMinimalSystemPaths(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& system_stops
);

StepPathsAdjacencyList SplitPathsAt(
    const StepPathsAdjacencyList& paths,
    const std::unordered_set<StopId> intermediate_stops
);

// An edge in the relaxed adjacency list.
struct RelaxedEdge {
  StopId destination_stop;
  int weight_seconds;  // Duration of the shortest step between stops

  bool operator==(const RelaxedEdge& other) const {
    return destination_stop == other.destination_stop &&
           weight_seconds == other.weight_seconds;
  }
};

// A simple directed weighted graph on StopIds.
// Each edge weight is the duration in seconds of the shortest step between
// stops.
struct RelaxedAdjacencyList {
  // CSR (Compressed Sparse Row) representation.
  // edge_offsets[stop_id.v] is the start index into `edges` for that stop.
  std::vector<int> edge_offsets;

  // Flat vector of all edges. Edges for stop s are at indices
  // [edge_offsets[s.v], edge_offsets[s.v + 1]) for s.v < NumStops() - 1,
  // or [edge_offsets[s.v], edges.size()) for s.v == NumStops() - 1.
  std::vector<RelaxedEdge> edges;

  int NumStops() const { return static_cast<int>(edge_offsets.size()); }

  // Get the edges originating at the given stop.
  std::span<const RelaxedEdge> GetEdges(StopId stop) const {
    if (stop.v < 0 || stop.v >= NumStops()) {
      return {};
    }
    int start = edge_offsets[stop.v];
    int end = (stop.v + 1 < NumStops()) ? edge_offsets[stop.v + 1]
                                        : static_cast<int>(edges.size());
    return std::span<const RelaxedEdge>(edges.data() + start, end - start);
  }

  bool operator==(const RelaxedAdjacencyList& other) const {
    return edge_offsets == other.edge_offsets && edges == other.edges;
  }
};

// Create a relaxed adjacency list from a steps adjacency list.
// For each origin-destination pair, the weight is the duration in seconds
// of the shortest step (minimum destination_time - origin_time across all
// steps).
RelaxedAdjacencyList MakeRelaxedAdjacencyList(
    const StepsAdjacencyList& steps_list
);

// Find shortest paths from origin to all reachable stops using Dijkstra's
// algorithm. Returns a vector indexed by StopId.v containing the shortest
// distance in seconds from origin to each stop. Unreachable stops have distance
// std::numeric_limits<int>::max().
std::vector<int> FindShortestRelaxedPaths(
    const RelaxedAdjacencyList& adjacency_list, StopId origin
);

// Create a reversed relaxed adjacency list where all edges are reversed.
// An edge (u -> v, weight) becomes (v -> u, weight).
RelaxedAdjacencyList ReverseRelaxedAdjacencyList(
    const RelaxedAdjacencyList& adjacency_list
);

RelaxedDistances ComputeRelaxedDistances(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& destinations
);

// Return a modified `path` where any intermediate stops within
// `threshold_meters` of a stop in `stops` are replaced by that stop, and then
// steps that are thusly changed to be from a stop to itself are dropped.
//
// Note that this doesn't change any trip ids or times in the modified steps, so
// the steps become steps that may not actually be possible, so maybe this
// should be used only to clean up visualizations and not to do actual
// computations.
std::vector<Step> SnapToStops(
    const DataGtfsMapping& mapping,
    const std::unordered_set<StopId>& stops,
    double threshold_meters,
    const std::vector<Step>& path
);

// Snap all paths in the adjacency list to the stops that are keys of the
// adjacency list.
StepPathsAdjacencyList AdjacencyListSnapToStops(
    const DataGtfsMapping& mapping,
    double threshold_meters,
    const StepPathsAdjacencyList& paths
);

}  // namespace vats5
