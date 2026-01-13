#pragma once

#include <vector>

#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"

namespace vats5 {

// Result of solving TSP with Concorde.
struct ConcordeSolution {
  std::vector<StopId> tour;

  // Optimal tour cost as reported by Concorde (rounded to int).
  int optimal_value;
};

// Solves TSP using Concorde and returns the tour.
// The tour visits all stops in the relaxed adjacency list exactly once.
ConcordeSolution SolveTspWithConcorde(const RelaxedAdjacencyList& relaxed);

}  // namespace vats5
