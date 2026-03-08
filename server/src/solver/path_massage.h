#pragma once

#include <vector>

#include "solver/data.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"

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

PartialSolution NaivelyExtendPartialSolution(
    const ProblemState& original_problem,
    const std::vector<StopId>& partial_solution_tour,
    StopId new_stop
);

PartialSolutionPath GreedilyExtendAsMuchAsPossibleWithoutIncreasingDuration(
    const ProblemState& original_problem,
    const PartialSolutionPath& partial_path
);

}  // namespace vats5
