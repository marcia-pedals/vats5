#pragma once

#include <optional>
#include <ostream>
#include <vector>

#include "solver/gtsp.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct GtspSolution {
  // GTSP vertex ids in visit order, from Gtsp::kStart to Gtsp::kEnd
  // inclusive. Consecutive entries are joined by GTSP edges.
  std::vector<int> tour;

  // Sum of the edge weights along `tour`. For a Gtsp from BuildGtsp this is
  // the time of the last (stop, time) vertex minus the time of the first.
  int cost_seconds;
};

// The cluster of every GTSP vertex, derived from the required stop groups: a
// vertex's cluster is its stop's group representative. Throws
// std::invalid_argument if a vertex's stop is not required.
std::vector<int> GtspClustersFromRequired(
    const Gtsp& gtsp, const RequiredStops& required
);

// The cheapest GTSP tour that visits the given stops in the given order
// (`stops` runs from the START stop to the END stop, naming exactly one stop
// per cluster in between). Returns nullopt if no tour follows that order.
// Useful for turning a stop order from a heuristic into a starting tour.
std::optional<GtspSolution> GtspTourAlongStops(
    const Gtsp& gtsp, const std::vector<StopId>& stops
);

// Solves the GTSP exactly with Concorde: a cheapest cycle
// START -> ... -> END -> START through exactly one vertex of every cluster.
// `cluster_of_vertex[v]` is the cluster of vertex v; START and END must be the
// only members of their clusters. Returns nullopt if no such cycle exists.
//
// `initial_tour`, if given, must be a valid tour (one vertex per cluster,
// consecutive vertices joined by edges); it is handed to Concorde as its
// starting tour and upper bound instead of Concorde's own heuristic tour.
//
// The GTSP is turned into an ATSP by the Noon-Bean transformation, then into a
// symmetric TSP by vertex doubling, and solved as a sparse instance (only the
// transformed edges exist). This is exact for a Gtsp from BuildGtsp; see
// gtsp_concorde.cpp for what the encoding relies on. If tsp_log is non-null,
// encoding statistics and Concorde's output are written to it.
//
// Not thread-safe: the call temporarily changes the process's cwd and
// redirects stdout/stderr while Concorde runs.
std::optional<GtspSolution> SolveGtspWithConcorde(
    const Gtsp& gtsp,
    const std::vector<int>& cluster_of_vertex,
    const std::optional<GtspSolution>& initial_tour = std::nullopt,
    std::ostream* tsp_log = nullptr
);

}  // namespace vats5
