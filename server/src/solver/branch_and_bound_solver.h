#pragma once

#include <ostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/data.h"
#include "solver/shortest_path.h"

namespace vats5 {

// Bidirectional mapping between original stop IDs and remapped consecutive IDs.
struct StopIdRemapping {
  // Maps original StopId -> new consecutive StopId
  std::unordered_map<int, StopId> original_to_new;

  // Maps new consecutive StopId -> original StopId
  std::vector<StopId> new_to_original;
};

// Result of remapping stop IDs to consecutive integers starting from 0.
struct RemappedAdjacencyList {
  StepsAdjacencyList adj;
  StopIdRemapping mapping;
};

// Remaps all stop IDs in the adjacency list to consecutive integers [0, N)
// where N is the number of unique stops that have outgoing edges or are
// destinations of edges.
RemappedAdjacencyList RemapStopIds(const StepsAdjacencyList& adj);

// Output a RelaxedAdjacencyList as a Concorde TSP instance using UPPER_ROW format.
// The graph is symmetrized by taking min(weight(i,j), weight(j,i)).
// Missing edges get a large weight.
void OutputConcordeTsp(std::ostream& out, const RelaxedAdjacencyList& relaxed);

// Result of solving TSP with Concorde.
struct ConcordeSolution {
  std::vector<StopId> tour;

  // Optimal tour cost as reported by Concorde (rounded to int).
  int optimal_value;
};

// Solves TSP using Concorde and returns the tour.
ConcordeSolution SolveTspWithConcorde(const RelaxedAdjacencyList& relaxed);

// Result of computing the actual path for a tour.
struct ActualPathResult {
  int duration_seconds;
};

// Given a tour (indices into remapped), compute the actual path by finding
// minimal paths between consecutive stops and merging steps.
// Returns the smallest duration among all valid departure times.
ActualPathResult ComputeActualPath(
    const std::vector<int>& tour,
    const RemappedAdjacencyList& remapped,
    const StepsAdjacencyList& minimal_steps,
    const DataGtfsMapping& mapping);

void DoIt(
    const DataGtfsMapping& mapping,
    const StepsAdjacencyList& adj,
    const std::unordered_set<StopId>& target_stops
);

}  // namespace vats5
