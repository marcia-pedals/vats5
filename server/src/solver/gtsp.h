#pragma once

#include <optional>
#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct GtspVertex {
  StopId stop;
  TimeSinceServiceStart time;

  bool operator==(const GtspVertex&) const = default;
};

struct GtspEdge {
  int from;
  int to;
  int weight_seconds;

  // The step realizing the edge: its duration (without the wait at the
  // destination that weight_seconds also includes) and its destination
  // partition. Zero and StepPartitionId::NONE for the START/END edges.
  int travel_seconds = 0;
  StepPartitionId partition = StepPartitionId::NONE;

  bool operator==(const GtspEdge&) const = default;
};

// A generalized TSP over (stop, time) vertices built from the scheduled steps
// of a StepsAdjacencyList; flex steps are disregarded. See BuildGtsp for the
// construction. Clusters are not part of this struct: a solver is given the
// cluster of every vertex separately (see gtsp_concorde.h).
struct Gtsp {
  // Vertex ids of START and END. The other vertices are (stop, time) pairs.
  static constexpr int kStart = 0;
  static constexpr int kEnd = 1;

  // Indexed by vertex id. The START and END entries carry the boundary stops
  // and time 0.
  std::vector<GtspVertex> vertices;
  std::vector<GtspEdge> edges;
};

// Vertices are START, END, and every distinct (stop, time) where time is the
// departure of a scheduled step from that stop or the arrival of a scheduled
// step at that stop. A step a->b departing s and arriving t_arr yields edges
// from (a, s) to (b, t_arr) and to every (b, t) where t is the first departure
// at or after t_arr of a step from b to some stop, each of weight t - s. So
// an edge's weight is the travel plus waiting time, and the weights of a path
// telescope to (time of its last vertex) - (time of its first vertex).
// Every vertex has a zero-cost edge to END, and END has a zero-cost edge to
// START. Without `start_time`, START has a zero-cost edge to every vertex, so
// a tour may begin anywhere at any time and its cost is (time of its last
// vertex) - (time of its first). With `start_time`, the tour begins at that
// time: START has an edge to, for every stop b and every stop c reachable
// from b, the vertex of b's first departure towards c at or after
// start_time, weighted by the wait from start_time, so a tour's cost is
// (time of its last vertex) - start_time.
Gtsp BuildGtsp(
    const StepsAdjacencyList& list,
    ProblemBoundary boundary,
    std::optional<TimeSinceServiceStart> start_time = std::nullopt
);

}  // namespace vats5
