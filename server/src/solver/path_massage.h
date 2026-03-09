#pragma once

#include <vector>

#include "solver/data.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"

namespace vats5 {

// A single path along with the tour of required stops that generated it.
struct PartialSolutionPath {
  // A path in the partial problem, including START and END.
  Path partial_problem_path;

  // The corresponding path in the original problem, including START and END.
  Path original_problem_path;
};

int CountRequiredStops(const Path& path, const RequiredStops& required);

// A "partial problem" is a problem where the paths are required to visit a
// certain subset of the required stops. This is a solution to such a problem.
struct PartialSolution {
  std::vector<PartialSolutionPath> paths;
};

std::optional<PartialSolutionPath> NaivelyExtendPartialSolution(
    const ProblemState& original_problem,
    const PartialSolutionPath& partial_solution_path,
    StopId new_stop
);

PartialSolutionPath GreedilyExtendAsMuchAsPossibleWithoutIncreasingDuration(
    const ProblemState& original_problem, const PartialSolutionPath& path
);

}  // namespace vats5
