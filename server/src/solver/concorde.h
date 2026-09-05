#pragma once

#include <exception>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"

namespace vats5 {

// Thrown when a tour does not follow the structure required of tours according
// to a reduction.
class InvalidTourStructure : public std::exception {
 public:
  explicit InvalidTourStructure(std::string message)
      : message_(std::move(message)) {}
  const char* what() const noexcept override { return message_.c_str(); }

 private:
  std::string message_;
};

// Thrown when an edge weight is too large for kForbiddenEdgeWeight to
// distinguish it from a truly absent edge.
class EdgeWeightOverflow : public std::exception {
 public:
  explicit EdgeWeightOverflow(std::string message)
      : message_(std::move(message)) {}
  const char* what() const noexcept override { return message_.c_str(); }

 private:
  std::string message_;
};

// Thrown when Concorde crashes (e.g., SIGABRT). Retrying may succeed.
class ConcordeCrash : public std::exception {
 public:
  explicit ConcordeCrash(std::string message) : message_(std::move(message)) {}
  const char* what() const noexcept override { return message_.c_str(); }

 private:
  std::string message_;
};

// Result of solving TSP with Concorde.
struct ConcordeSolution {
  std::vector<StopId> tour;

  // Optimal tour cost as reported by Concorde (rounded to int).
  int optimal_value;
};

// Solves TSP using Concorde and returns the tour.
// The tour visits all stops in the relaxed adjacency list exactly once.
// If tsp_log is non-null, Concorde's output is written to it.
// Returns nullopt if the optimal tour uses a forbidden edge (no valid tour
// exists).
//
// `weight_scale` declares that the caller has multiplied all edge weights by
// this factor (e.g. to add sub-second tie-breaking perturbations); the
// internal inter-vertex and forbidden-edge offsets are scaled to match, and
// `optimal_value` is returned in the caller's scaled units.
// Largest (already scaled) edge weight SolveTspWithConcorde accepts for
// `weight_scale`, given the smallest scaled weight `min_weight` in the graph
// (<= 0; used to size the negative-weight offset). Callers that scale weights
// must cap them at this value; capping only lowers costs, so a TSP lower
// bound computed on capped weights stays valid.
int MaxConcordeEdgeWeight(int weight_scale, int min_weight);

std::optional<ConcordeSolution> SolveTspWithConcorde(
    const RelaxedAdjacencyList& relaxed,
    std::optional<int> ub = std::nullopt,
    std::ostream* tsp_log = nullptr,
    int weight_scale = 1
);

}  // namespace vats5
