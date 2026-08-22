#include "solver/iterative_expansion_partial_solve.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <ranges>
#include <set>
#include <span>
#include <unordered_map>

#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tour_paths.h"

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

namespace {

// Filters `original_problem.required` down to `required_subset` (which must
// contain whole groups).
RequiredStops FilterRequired(
    const ProblemState& original_problem,
    const std::unordered_set<StopId>& required_subset
) {
  RequiredStops partial_required = original_problem.required;
  std::erase_if(partial_required.representative, [&](const auto& pair) {
    return !required_subset.contains(pair.first);
  });
  // Assert each group is entirely present or entirely absent.
  for (StopId rep : original_problem.required.GroupRepresentatives()) {
    bool present_in_subset = required_subset.contains(rep);
    original_problem.required.VisitGroupStops(rep, [&](StopId group_stop) {
      assert(required_subset.contains(group_stop) == present_in_subset);
    });
  }
  return partial_required;
}

}  // namespace

// Maps branch-and-bound result paths back to original-problem paths.
PartialSolution PartialSolutionFromBnbResult(
    const BranchAndBoundResult& bb_result, const ProblemState& original_problem
) {
  // Find the original problem paths corresponding to the partial problem paths.
  std::vector<PartialSolutionPath> paths;
  std::set<std::vector<StopId>> seen_tours;
  for (const Path& bb_path : bb_result.best_paths) {
    // Reconstruct the tour of partial problem stops.
    std::vector<StopId> tour;
    bb_path.VisitAllStops([&](StopId bb_result_stop) {
      ExpandStop(bb_result_stop, bb_result.original_edges, tour);
    });

    assert(tour.size() >= 2);
    assert(*(tour.begin()) == original_problem.boundary.start);
    assert(*(tour.end() - 1) == original_problem.boundary.end);

    // If we have already seen this tour then we don't need to process it again.
    if (!seen_tours.insert(tour).second) {
      continue;
    }

    // Reconstruct the paths through all original problem stops.
    std::vector<Path> more_original_paths =
        ComputeMinimalFeasiblePathsAlong(tour, original_problem.minimal);

    // Some of these paths might have duration longer than the bb_path.
    // Disregard these.
    std::erase_if(more_original_paths, [&](const Path& path) {
      return path.DurationSeconds() > bb_path.DurationSeconds();
    });

    // There must be paths left with duration <= the bb_result path because all
    // paths in the partial problem are also paths in the full problem.
    assert(more_original_paths.size() > 0);

    // All original problem paths much have duration == the bb_result path
    // because otherwise the bb_result path isn't the best path in the partial
    // problem.
    for (const Path& path : more_original_paths) {
      assert(path.merged_step.origin.stop == original_problem.boundary.start);
      assert(
          path.merged_step.destination.stop == original_problem.boundary.end
      );
      assert(path.DurationSeconds() == bb_path.DurationSeconds());
      paths.push_back(
          PartialSolutionPath{
              .path = path,
              .subset_tour = tour,
          }
      );
    }
  }

  // TODO: Think about wither `paths` could contain duplicate
  // paths or other non-minimality.

  return PartialSolution{.paths = std::move(paths)};
}


ProblemState MakeReducedPartialProblem(
    const ProblemState& original_problem,
    std::unordered_set<StopId> required_subset
) {
  required_subset.insert(original_problem.boundary.start);
  required_subset.insert(original_problem.boundary.end);

  RequiredStops partial_required =
      FilterRequired(original_problem, required_subset);

  return MakeProblemState(
      MakeAdjacencyList(
          ReduceToMinimalSystemPaths(
              original_problem.minimal,
              required_subset,
              HorizonCoveringAllDepartures(original_problem.minimal)
          )
              .AllMergedSteps()
      ),
      original_problem.boundary,
      std::move(partial_required),
      original_problem.stop_infos,
      original_problem.step_partition_names,
      original_problem.original_edges
  );
}

PartialSolution PartialSolveBranchAndBound(
    std::unordered_set<StopId> required_subset,
    const ProblemState& original_problem,
    std::ostream* search_log,
    const SearchEventCallback& on_event,
    const SearchSeeds* seeds
) {
  ProblemState partial_problem =
      MakeReducedPartialProblem(original_problem, std::move(required_subset));

  auto bb_result = BranchAndBoundSolve(
      partial_problem, search_log, std::nullopt, -1, on_event, nullptr, seeds
  );
  if (bb_result.best_paths.empty()) {
    return PartialSolution{};
  }

  return PartialSolutionFromBnbResult(bb_result, original_problem);
}

PartialSolution PartialSolveBranchAndCut(
    std::unordered_set<StopId> required_subset,
    const ProblemState& original_problem,
    std::ostream* search_log,
    const SearchEventCallback& on_event,
    const LazyRequiredStops& lazy
) {
  required_subset.insert(original_problem.boundary.start);
  required_subset.insert(original_problem.boundary.end);

  RequiredStops partial_required =
      FilterRequired(original_problem, required_subset);

  // The full, unreduced problem with only the subset required: the required
  // set grows during the search, so the graph must already contain every stop
  // it can grow to.
  ProblemState full_problem = original_problem.WithRequired(partial_required);

  auto bb_result = BranchAndBoundSolve(
      full_problem, search_log, std::nullopt, -1, on_event, &lazy
  );
  if (bb_result.best_paths.empty()) {
    return PartialSolution{};
  }

  return PartialSolutionFromBnbResult(bb_result, original_problem);
}

PartialSolution PartialSolveBruteForce(
    const std::unordered_set<StopId>& required_subset,
    const ProblemState& original_problem,
    const std::function<void()>& check_deadline
) {
  StopId start = original_problem.boundary.start;
  StopId end = original_problem.boundary.end;

  // Visiting any one stop of a group satisfies the whole group, so an order is
  // a permutation of the groups together with a choice of stop within each.
  std::map<StopId, std::vector<StopId>> stops_by_representative;
  for (StopId stop : required_subset) {
    if (stop == start || stop == end) {
      continue;
    }
    stops_by_representative[original_problem.required.Representative(stop)]
        .push_back(stop);
  }
  std::vector<std::vector<StopId>> groups;
  for (auto& [representative, stops] : stops_by_representative) {
    std::ranges::sort(stops);
    groups.push_back(std::move(stops));
  }

  // All the paths between the stops a tour can visit, so that evaluating one
  // more stop of an order is just a merge.
  std::unordered_set<StopId> tour_stops = required_subset;
  tour_stops.insert(start);
  tour_stops.insert(end);
  StepPathsAdjacencyList completed = CompleteShortestPathsGraph(
      original_problem.minimal,
      tour_stops,
      HorizonCoveringAllDepartures(original_problem.minimal)
  );

  auto min_duration = [](const std::vector<Path>& paths) {
    int result = std::numeric_limits<int>::max();
    for (const Path& path : paths) {
      result = std::min(result, path.DurationSeconds());
    }
    return result;
  };

  int best_duration = std::numeric_limits<int>::max();
  std::vector<PartialSolutionPath> best_paths;

  // The order being searched, and the groups it has visited so far.
  std::vector<StopId> tour{start};
  std::vector<bool> visited(groups.size(), false);
  size_t num_visited = 0;

  // The paths from START along `tour`, extended to `next`.
  auto extend = [&](const std::vector<Path>& paths, StopId next) {
    std::span<const Path> edge_paths =
        completed.PathsBetween(tour.back(), next);
    if (tour.size() == 1) {
      return std::vector<Path>(edge_paths.begin(), edge_paths.end());
    }
    return ExtendMinimalFeasiblePaths(paths, edge_paths);
  };

  // Visits every completion of `tour`, whose paths from START are `paths`.
  auto search = [&](this auto&& self, const std::vector<Path>& paths) -> void {
    check_deadline();

    if (num_visited == groups.size()) {
      std::vector<Path> complete = extend(paths, end);
      // Same as ComputeMinimalFeasiblePathsAlong: paths that depart before
      // 00:00:00 don't count.
      std::erase_if(complete, [](const Path& path) {
        return path.merged_step.origin.time < TimeSinceServiceStart{0};
      });
      int duration = min_duration(complete);
      if (duration > best_duration) {
        return;
      }
      if (duration < best_duration) {
        best_duration = duration;
        best_paths.clear();
      }
      tour.push_back(end);
      for (Path& path : complete) {
        if (path.DurationSeconds() != best_duration) {
          continue;
        }
        NormalizeConsecutiveSteps(path.steps);
        assert(ConsecutiveMergedSteps(path.steps) == path.merged_step);
        best_paths.push_back(
            PartialSolutionPath{.path = std::move(path), .subset_tour = tour}
        );
      }
      tour.pop_back();
      return;
    }

    for (size_t group = 0; group < groups.size(); ++group) {
      if (visited[group]) {
        continue;
      }
      visited[group] = true;
      num_visited++;
      for (StopId stop : groups[group]) {
        std::vector<Path> extended = extend(paths, stop);
        // Extending these paths any further only makes them longer.
        if (extended.empty() || min_duration(extended) > best_duration) {
          continue;
        }
        tour.push_back(stop);
        self(extended);
        tour.pop_back();
      }
      visited[group] = false;
      num_visited--;
    }
  };
  search({});

  return PartialSolution{.paths = std::move(best_paths)};
}

TourPathSets TourPathSets::Compute(
    const std::vector<StopId>& tour, MinimalPathSetCache& cache
) {
  size_t n = tour.size();
  TourPathSets result{
      .prefixes = std::vector<std::vector<Path>>(n),
      .suffixes = std::vector<std::vector<Path>>(n),
  };
  for (size_t i = 1; i < n; ++i) {
    std::span<const Path> edge = cache.Between(tour[i - 1], tour[i]);
    result.prefixes[i] =
        i == 1 ? std::vector<Path>(edge.begin(), edge.end())
               : ExtendMinimalFeasiblePaths(result.prefixes[i - 1], edge);
  }
  for (size_t i = n - 1; i-- > 0;) {
    std::span<const Path> edge = cache.Between(tour[i], tour[i + 1]);
    result.suffixes[i] =
        i + 2 == n ? std::vector<Path>(edge.begin(), edge.end())
                   : ExtendMinimalFeasiblePaths(edge, result.suffixes[i + 1]);
  }
  return result;
}

PartialSolution NaivelyExtendPartialSolution(
    const std::vector<StopId>& partial_solution_tour,
    const TourPathSets& tour_paths,
    StopId new_stop,
    MinimalPathSetCache& cache
) {
  int best_duration = std::numeric_limits<int>::max();
  std::vector<PartialSolutionPath> best_paths;

  // Figure out the duration of the extended tour with the new stop in each
  // position. Intentionally don't try the new stop first or last because first
  // and last should always be START and END.
  size_t n = partial_solution_tour.size();
  for (size_t position = 1; position < n; ++position) {
    std::span<const Path> to_new =
        cache.Between(partial_solution_tour[position - 1], new_stop);
    if (to_new.empty()) {
      continue;
    }
    std::vector<Path> paths = ExtendMinimalFeasiblePaths(
        to_new, cache.Between(new_stop, partial_solution_tour[position])
    );
    if (position > 1) {
      paths =
          ExtendMinimalFeasiblePaths(tour_paths.prefixes[position - 1], paths);
    }
    if (position + 1 < n) {
      paths = ExtendMinimalFeasiblePaths(paths, tour_paths.suffixes[position]);
    }
    // Same as ComputeMinimalFeasiblePathsAlong: paths that depart before
    // 00:00:00 don't count.
    std::erase_if(paths, [](const Path& path) {
      return path.merged_step.origin.time < TimeSinceServiceStart{0};
    });

    auto best_path_it = std::ranges::min_element(
        paths, {}, [](const Path& path) { return path.DurationSeconds(); }
    );
    if (best_path_it == paths.end() ||
        best_path_it->DurationSeconds() > best_duration) {
      continue;
    }
    if (best_path_it->DurationSeconds() < best_duration) {
      best_duration = best_path_it->DurationSeconds();
      best_paths.clear();
    }

    std::vector<StopId> extended_tour = partial_solution_tour;
    extended_tour.insert(extended_tour.begin() + position, new_stop);
    for (Path& path : paths) {
      if (path.DurationSeconds() != best_duration) {
        continue;
      }
      NormalizeConsecutiveSteps(path.steps);
      assert(ConsecutiveMergedSteps(path.steps) == path.merged_step);
      best_paths.push_back({
          .path = std::move(path),
          .subset_tour = extended_tour,
      });
    }
  }

  return PartialSolution{.paths = best_paths};
}

}  // namespace vats5
