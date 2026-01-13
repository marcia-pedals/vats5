#pragma once

#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

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

struct RelaxedDistances {
  // distance_to[dest_id][stop_id.v] is the relaxed distance from stop_id to
  // dest_id.
  std::unordered_map<StopId, std::vector<int>> distance_to;
};

// Create a relaxed adjacency list from a steps adjacency list.
// For each origin-destination pair, the weight is the duration in seconds
// of the shortest step (minimum destination_time - origin_time across all
// steps).
RelaxedAdjacencyList MakeRelaxedAdjacencyList(
    const StepsAdjacencyList& steps_list
);

// Create a reversed relaxed adjacency list where all edges are reversed.
// An edge (u -> v, weight) becomes (v -> u, weight).
RelaxedAdjacencyList ReverseRelaxedAdjacencyList(
    const RelaxedAdjacencyList& adjacency_list
);

}  // namespace vats5
