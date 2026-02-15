#pragma once

#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

// Backtrack through the search results to reconstruct the full path.
// Returns the steps in order from origin to destination.
std::vector<Step> BacktrackPath(
    const std::vector<Step>& search_result, StopId dest
);

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
  explicit HeuristicCache(const RelaxedDistances* rd) : relaxed_distances(rd) {}

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
    const RelaxedDistances* relaxed_distances = nullptr,
    bool keep_through_other_destination = false
);

// Return a "minimal" graph with these properties:
//
// (a) Any path on `adjacency_list` between two `system_stops` [1] can be
// matched or beat by a path on the minimal graph.
//
// (b) If you remove any "step" from the minimal graph, (a) no longer holds. (Here
// "step" is used in the sense of one element of PathsBetween(a, b) even though
// that's technically a Path, to disambiguate from the concept of a "path" made
// up of multiple steps on a graph).
//
// [1] Usually-unimportant qualification: All departures from `system_stops` in
// the path happen at <36:00.
StepPathsAdjacencyList ReduceToMinimalSystemPaths(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& system_stops
);

// Return a "completed" graph where the "steps" from a to b are a minimal cover
// of all the paths from a to b on `adjacency_list`. ("Steps" used in the same
// sense as in (b) above).
StepPathsAdjacencyList CompleteShortestPathsGraph(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& system_stops
);

}  // namespace vats5
