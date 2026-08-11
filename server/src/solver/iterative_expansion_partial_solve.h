#pragma once

#include <functional>
#include <iosfwd>
#include <unordered_set>
#include <vector>

#include "solver/branch_and_bound.h"
#include "solver/data.h"
#include "solver/tarel_graph.h"

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
  std::vector<PartialSolutionPath> paths;

  // Returns the path that visits the most required stops.
  // Returns paths.end() if no paths are available.
  std::vector<PartialSolutionPath>::const_iterator BestPathByRequiredStops(
      const RequiredStops& required
  ) const;
};

// Solves the partial problem of `original_problem` in which the paths must
// visit `required_subset`, by branch and bound over the problem reduced to
// that subset.
//
// `search_log` may be null; `on_event` is called on each search event.
PartialSolution PartialSolveBranchAndBound(
    std::unordered_set<StopId> required_subset,
    const ProblemState& original_problem,
    std::ostream* search_log,
    const SearchEventCallback& on_event = nullptr
);

// Solves the same partial problem as PartialSolveBranchAndBound, by trying
// every order in which `required_subset` can be visited.
//
// `check_deadline` is called as the search proceeds and may throw to abandon
// it.
PartialSolution PartialSolveBruteForce(
    const std::unordered_set<StopId>& required_subset,
    const ProblemState& original_problem,
    const std::function<void()>& check_deadline = [] {}
);

}  // namespace vats5
