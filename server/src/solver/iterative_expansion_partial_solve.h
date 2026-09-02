#pragma once

#include <iosfwd>
#include <limits>
#include <unordered_set>
#include <vector>

#include "solver/branch_and_bound.h"
#include "solver/data.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"
#include "solver/two_opt.h"

namespace vats5 {

// A single path along with the tour of required stops that generated it.
struct PartialSolutionPath {
  // A path that achieves the minimum duration in the partial problem.
  // Includes START and END. All original-problem stops that this path passes
  // through are included as intermediate stops.
  Path path;

  // The tour of the required subset that generates `path`. Includes START and
  // END.
  std::vector<StopId> subset_tour;
};

// The number of required groups that `path` visits.
int CountRequiredStops(const Path& path, const RequiredStops& required);

// A "partial problem" is a problem where the paths are required to visit a
// certain subset of the required stops. This is a solution to such a problem.
struct PartialSolution {
  // Lower bound proven by the solver. Defaults to the trivial bound.
  int lb = 0;

  // Upper bound proven by the solver. INT_MAX means no solution was found.
  int ub = std::numeric_limits<int>::max();

  // At least one path achieving `ub`.
  std::vector<PartialSolutionPath> paths;

  // Returns the path that visits the most required stops.
  // Returns paths.end() if no paths are available.
  std::vector<PartialSolutionPath>::const_iterator BestPathByRequiredStops(
      const RequiredStops& required
  ) const;
};

ProblemState MakePartialProblemState(
    std::unordered_set<StopId> required_subset,
    const ProblemState& original_problem
);

// Solves the partial problem of `original_problem` that
// MakePartialProblemState built, by branch and bound over it.
//
// `search_log` may be null; `on_event` is called on each search event.
PartialSolution PartialSolveBranchAndBound(
    const ProblemState& partial_problem,
    const ProblemState& original_problem,
    int known_lb,
    std::ostream* search_log,
    const SearchEventCallback& on_event = nullptr
);

// Solves the same partial problem as PartialSolveBranchAndBound, by Held-Karp
// dynamic programming over the required groups.
//
// `search_log` may be null.
PartialSolution PartialSolveHeldKarp(
    const ProblemState& partial_problem,
    const ProblemState& original_problem,
    int known_lb,
    std::ostream* search_log = nullptr
);

// Solves the same partial problem as PartialSolveBranchAndBound, by 2-opt
// local search over the required groups. Unlike the other two this is a
// heuristic: the returned paths are exact optima for their visit order, but
// the order itself may be suboptimal. The search returns as soon as it finds
// a tour achieving `known_lb`, a lower bound on the partial problem's optimum
// that the caller has already proven (0 if nothing is known).
//
// `search_log` may be null.
PartialSolution PartialSolveTwoOpt(
    const ProblemState& partial_problem,
    const ProblemState& original_problem,
    int known_lb,
    const TwoOptOptions& options = {},
    std::ostream* search_log = nullptr
);

// The path sets along the parts of a tour, so that evaluating a stop inserted
// into it costs a few merges instead of recomputing the whole tour.
//
// prefixes[i] runs from tour[0] to tour[i] and suffixes[i] from tour[i] to the
// last stop, so splicing at position i is prefixes[i - 1] then the two new
// edges then suffixes[i]. The empty ends (prefixes[0] and the last suffix) are
// unused: a path set spanning no edges isn't representable.
struct TourPathSets {
  std::vector<std::vector<Path>> prefixes;
  std::vector<std::vector<Path>> suffixes;

  static TourPathSets Compute(
      const std::vector<StopId>& tour, MinimalPathSetCache& cache
  );
};

// Solves the partial problem of visiting `partial_solution_tour` plus
// `new_stop`, restricted to the tours that insert `new_stop` into
// `partial_solution_tour` without reordering it.
//
// `new_stop` is never inserted first or last, since those positions belong to
// the boundary stops. `tour_paths` must be TourPathSets::Compute of
// `partial_solution_tour`; it and `cache` must come from the same graph.
PartialSolution NaivelyExtendPartialSolution(
    const std::vector<StopId>& partial_solution_tour,
    const TourPathSets& tour_paths,
    StopId new_stop,
    MinimalPathSetCache& cache
);

}  // namespace vats5
