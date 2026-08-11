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

PartialSolution PartialSolveBranchAndBound(
    std::unordered_set<StopId> required_subset,
    const ProblemState& original_problem,
    std::ostream* search_log,
    const SearchEventCallback& on_event
) {
  required_subset.insert(original_problem.boundary.start);
  required_subset.insert(original_problem.boundary.end);

  // Filter required to just this subset. Each group must be entirely present
  // or entirely absent, since we add stops by group via VisitGroupStops.
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

  ProblemState partial_problem = MakeProblemState(
      MakeAdjacencyList(
          ReduceToMinimalSystemPaths(original_problem.minimal, required_subset)
              .AllMergedSteps()
      ),
      original_problem.boundary,
      std::move(partial_required),
      original_problem.stop_infos,
      original_problem.step_partition_names,
      original_problem.original_edges
  );

  auto bb_result = BranchAndBoundSolve(
      partial_problem, search_log, std::nullopt, -1, on_event
  );
  if (bb_result.best_paths.empty()) {
    return PartialSolution{};
  }

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
  StepPathsAdjacencyList completed =
      CompleteShortestPathsGraph(original_problem.minimal, tour_stops);

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

}  // namespace vats5
