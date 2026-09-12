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

// Thrown when Concorde reports an internal failure. Retrying with a different
// seed may succeed.
class ConcordeFailure : public std::exception {
 public:
  explicit ConcordeFailure(std::string message)
      : message_(std::move(message)) {}
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

// Solves TSP using Concorde (linked in as a library) and returns the tour.
// The tour visits all stops in the relaxed adjacency list exactly once.
// If tsp_log is non-null, Concorde's output is written to it.
// If ub is set, only tours with cost strictly less than ub are returned.
// Returns nullopt if the optimal tour uses a forbidden edge (no valid tour
// exists) or no tour beats ub.
//
// Not thread-safe: the call temporarily changes the process's cwd and
// redirects stdout/stderr while Concorde runs.
std::optional<ConcordeSolution> SolveTspWithConcorde(
    const RelaxedAdjacencyList& relaxed,
    std::optional<int> ub = std::nullopt,
    std::ostream* tsp_log = nullptr
);

// An edge with positive value in an LP relaxation's solution.
struct ConcordeSupportEdge {
  StopId from;
  StopId to;
  double x;
};

// Result of solving only the root LP relaxation (cutting planes, no
// branching) with Concorde.
struct ConcordeRootLp {
  // The final LP objective, in the input's units (construction offsets
  // removed). Fractional; a lower bound on the optimal tour up to floating
  // point error.
  double lp_bound;

  // A rigorous integer lower bound on the optimal tour cost: Concorde's
  // exactly priced bound, offsets removed, rounded up.
  int lower_bound;

  // Input-graph edges with positive LP value, in input stop ids.
  std::vector<ConcordeSupportEdge> support;

  // Number of LP-positive edges that are not edges of the input graph (the
  // LP paid the forbidden-edge sentinel for them). Usually zero.
  int num_forbidden_support_edges;
};

// Solves the root LP relaxation of the TSP on `relaxed` with Concorde: the
// full root cutting-plane loop, without branching. Returns nullopt if Concorde
// proves the LP infeasible (no tour exists). Small instances (below the brute
// force threshold) are solved exactly instead; their support is the optimal
// tour.
//
// Not thread-safe, for the same reasons as SolveTspWithConcorde.
std::optional<ConcordeRootLp> SolveTspRootLpWithConcorde(
    const RelaxedAdjacencyList& relaxed, std::ostream* tsp_log = nullptr
);

}  // namespace vats5
