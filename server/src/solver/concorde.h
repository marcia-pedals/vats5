#pragma once

#include <optional>
#include <ostream>
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
// If tsp_log is non-null, Concorde's output is written to it.
// Returns nullopt if the optimal tour uses a forbidden edge (no valid tour exists).
std::optional<ConcordeSolution> SolveTspWithConcorde(const RelaxedAdjacencyList& relaxed, std::ostream* tsp_log = nullptr);

}  // namespace vats5
