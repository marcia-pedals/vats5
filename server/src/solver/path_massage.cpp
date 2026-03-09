#include "solver/path_massage.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/steps_shortest_path.h"

namespace vats5 {

int CountRequiredStops(const Path& path, const RequiredStops& required) {
  std::unordered_set<StopId> required_rep_visited;
  path.VisitAllStops([&](StopId stop) {
    if (required.Contains(stop)) {
      required_rep_visited.insert(required.Representative(stop));
    }
  });
  return required_rep_visited.size();
}

std::optional<PartialSolutionPath> NaivelyExtendPartialSolution(
    const ProblemState& original_problem,
    const PartialSolutionPath& partial_solution_path,
    StopId new_stop
) {
  const Path& partial_problem_path = partial_solution_path.partial_problem_path;
  const Path& original_problem_path =
      partial_solution_path.original_problem_path;

  // Try to insert `new_stop` at each position in
  // `partial_solution_path.partial_problem_path`. Each iteration inserts the
  // stop "in between"
  // `partial_solution_path.partial_problem_path.steps[insert_index]`.
  for (int insert_index = 1;
       insert_index + 1 < partial_problem_path.steps.size();
       ++insert_index) {
    const Step& prev_step = partial_problem_path.steps[insert_index - 1];
    const Step& insert_in_step = partial_problem_path.steps[insert_index];
    const Step& next_step = partial_problem_path.steps[insert_index + 1];

    auto original_problem_insert_begin = std::find_if(
        original_problem_path.steps.begin(),
        original_problem_path.steps.end(),
        [&](const Step& s) { return s.origin == insert_in_step.origin; }
    );
    assert(original_problem_insert_begin != original_problem_path.steps.end());
    auto original_problem_insert_end = std::find_if(
        original_problem_path.steps.begin(),
        original_problem_path.steps.end(),
        [&](const Step& s) { return s.origin == next_step.origin; }
    );
    assert(original_problem_insert_end != original_problem_path.steps.end());

    std::vector<Step> to_new_stop = FindShortestPathsAtTime(
        original_problem.minimal,
        prev_step.destination.time,
        prev_step.destination.stop,
        {new_stop}
    );
    const Step& step_to_new_stop = to_new_stop[new_stop.v];
    if (step_to_new_stop.destination.time > next_step.origin.time) {
      continue;
    }

    std::vector<Step> from_new_stop = FindShortestPathsAtTime(
        original_problem.minimal,
        step_to_new_stop.destination.time,
        new_stop,
        {next_step.origin.stop}
    );
    const Step& step_from_new_stop = from_new_stop[next_step.origin.stop.v];
    if (step_from_new_stop.destination.time > next_step.origin.time) {
      continue;
    }

    // We can splice this in without disrupting the rest of the path!!
    std::vector<Step> steps_to_new_stop = BacktrackPath(to_new_stop, new_stop);
    Path path_to_new_stop{
        .merged_step = ConsecutiveMergedSteps(steps_to_new_stop),
        .steps = steps_to_new_stop,
    };

    std::vector<Step> steps_from_new_stop =
        BacktrackPath(from_new_stop, next_step.origin.stop);
    Path path_from_new_stop = {
        .merged_step = ConsecutiveMergedSteps(steps_from_new_stop),
        .steps = steps_from_new_stop,
    };

    PartialSolutionPath result = partial_solution_path;
    result.partial_problem_path.steps[insert_index] =
        path_to_new_stop.merged_step;
    result.partial_problem_path.steps.insert(
        result.partial_problem_path.steps.begin() + insert_index + 1,
        path_from_new_stop.merged_step
    );

    result.original_problem_path.steps.clear();
    result.original_problem_path.steps.insert(
        result.original_problem_path.steps.end(),
        original_problem_path.steps.begin(),
        original_problem_insert_begin
    );
    NormalizeConsecutiveSteps(result.original_problem_path.steps);
    result.original_problem_path.steps.insert(
        result.original_problem_path.steps.end(),
        steps_to_new_stop.begin(),
        steps_to_new_stop.end()
    );
    NormalizeConsecutiveSteps(result.original_problem_path.steps);
    result.original_problem_path.steps.insert(
        result.original_problem_path.steps.end(),
        steps_from_new_stop.begin(),
        steps_from_new_stop.end()
    );
    NormalizeConsecutiveSteps(result.original_problem_path.steps);

    result.original_problem_path.steps.insert(
        result.original_problem_path.steps.end(),
        original_problem_insert_end,
        original_problem_path.steps.end()
    );
    NormalizeConsecutiveSteps(result.original_problem_path.steps);

    return result;
  }

  return std::nullopt;
}

PartialSolutionPath GreedilyExtendAsMuchAsPossibleWithoutIncreasingDuration(
    const ProblemState& original_problem,
    const PartialSolutionPath& partial_solution_path
) {
  PartialSolutionPath result = partial_solution_path;
  std::unordered_set<StopId> candidates = original_problem.required.AllFlat();

  while (candidates.size() > 0) {
    result.original_problem_path.VisitAllStops([&](StopId stop) {
      StopId visited_rep = original_problem.required.Representative(stop);
      original_problem.required.VisitGroupStops(visited_rep, [&](StopId s) {
        candidates.erase(s);
      });
    });

    std::vector<PartialSolutionPath> improved;
    std::unordered_set<StopId> cur_candidates = candidates;
    for (StopId new_stop : cur_candidates) {
      std::optional<PartialSolutionPath> extended =
          NaivelyExtendPartialSolution(
              original_problem, partial_solution_path, new_stop
          );
      if (extended.has_value()) {
        result = std::move(*extended);
      } else {
        candidates.erase(new_stop);
      }
    }
  }

  return result;
}

}  // namespace vats5
