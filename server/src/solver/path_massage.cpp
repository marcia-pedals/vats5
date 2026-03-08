#include "solver/path_massage.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

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

std::vector<PartialSolutionPath>::const_iterator
PartialSolution::BestPathByRequiredStops(const RequiredStops& required) const {
  return std::ranges::max_element(
      paths, {}, [&](const PartialSolutionPath& sol_path) {
        return CountRequiredStops(sol_path.path, required);
      }
  );
}

PartialSolution NaivelyExtendPartialSolution(
    const ProblemState& original_problem,
    const std::vector<StopId>& partial_solution_tour,
    StopId new_stop
) {
  // Create an extended tour with the new stop inserted at index 0.
  std::vector<StopId> extended_tour;
  extended_tour.reserve(partial_solution_tour.size() + 1);
  extended_tour.push_back(new_stop);
  extended_tour.append_range(partial_solution_tour);

  int best_duration = std::numeric_limits<int>::max();
  std::vector<PartialSolutionPath> best_paths;

  // Cache FindMinimalPathSet results across insertion positions: most edges
  // in the tour stay the same when we swap new_stop to a different position.
  PathCache path_cache;

  // Figure out the duration of the extended tour with the new stop in each
  // position, by swapping it forwards. Intentionally don't try the new stop
  // first or last because first and last should always be START and END.
  for (int new_stop_index = 1; new_stop_index + 1 < extended_tour.size();
       ++new_stop_index) {
    // Swap forwards.
    std::swap(extended_tour[new_stop_index - 1], extended_tour[new_stop_index]);
    assert(extended_tour[new_stop_index] == new_stop);

    std::vector<Path> paths = ComputeMinimalFeasiblePathsAlong(
        extended_tour, original_problem.minimal, path_cache
    );
    auto best_path_it = std::ranges::min_element(
        paths, {}, [](const Path& path) { return path.DurationSeconds(); }
    );
    if (best_path_it == paths.end()) {
      continue;
    }
    const Path& best_path = *best_path_it;
    if (best_path.DurationSeconds() < best_duration) {
      best_duration = best_path.DurationSeconds();
      best_paths.clear();
    }
    if (best_path.DurationSeconds() == best_duration) {
      for (const Path& path : paths) {
        if (path.DurationSeconds() > best_path.DurationSeconds()) {
          continue;
        }
        best_paths.push_back({
            .path = path,
            .subset_tour = extended_tour,
        });
      }
    }
  }

  return PartialSolution{.paths = best_paths};
}

PartialSolutionPath GreedilyExtendAsMuchAsPossibleWithoutIncreasingDuration(
    const ProblemState& original_problem,
    const PartialSolutionPath& partial_path
) {
  PartialSolutionPath result = partial_path;

  while (true) {
    std::unordered_set<StopId> unvisited = original_problem.required.AllFlat();
    result.path.VisitAllStops([&](StopId stop) {
      StopId visited_rep = original_problem.required.Representative(stop);
      original_problem.required.VisitGroupStops(visited_rep, [&](StopId s) {
        unvisited.erase(s);
      });
    });

    std::vector<PartialSolutionPath> improved;
    for (StopId new_stop : unvisited) {
      PartialSolution extended = NaivelyExtendPartialSolution(
          original_problem, result.subset_tour, new_stop
      );
      auto best_extended_it =
          extended.BestPathByRequiredStops(original_problem.required);
      if (best_extended_it == extended.paths.end() ||
          best_extended_it->path.DurationSeconds() >
              result.path.DurationSeconds()) {
        continue;
      }
      improved.push_back(*best_extended_it);
    }

    auto best_improved_it = std::ranges::max_element(
        improved, {}, [&](const PartialSolutionPath& path) {
          return CountRequiredStops(path.path, original_problem.required);
        }
    );
    if (best_improved_it == improved.end()) {
      break;
    }

    result = *best_improved_it;
  }

  return result;
}

}  // namespace vats5
