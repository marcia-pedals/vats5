#include "solver/iterative_expansion_partial_solve.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <optional>
#include <ranges>
#include <set>
#include <span>
#include <unordered_map>

#include "solver/held_karp_dp.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tour_paths.h"
#include "solver/two_opt.h"

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

ProblemState MakePartialProblemState(
    std::unordered_set<StopId> required_subset,
    const ProblemState& original_problem
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
    const ProblemState& partial_problem,
    const ProblemState& original_problem,
    int known_lb,
    std::ostream* search_log,
    const SearchEventCallback& on_event
) {
  auto bb_result = BranchAndBoundSolve(
      partial_problem, known_lb, search_log, std::nullopt, -1, on_event
  );
  if (bb_result.best_paths.empty()) {
    // The search ran to completion without finding a path, so the partial
    // problem is infeasible.
    return PartialSolution{
        .lb = std::numeric_limits<int>::max(),
        .ub = std::numeric_limits<int>::max(),
    };
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

  return PartialSolution{
      // Branch and bound only terminates successfully if it finds the optimal
      // solution, so we can treat it as a lb.
      // TODO: We probably want bnb with a budget that reports a lower bound
      // when it runs out of time.
      .lb = bb_result.best_ub,
      .ub = bb_result.best_ub,
      .paths = std::move(paths)
  };
}

PartialSolution PartialSolveHeldKarp(
    const ProblemState& partial_problem,
    const ProblemState& original_problem,
    int known_lb,
    std::ostream* search_log
) {
  // The DP solves the boundary-to-boundary problem, so its visit order is
  // already a full tour from START to END.
  HeldKarpDPResult hk = HeldKarpDPSolve(partial_problem, known_lb, search_log);
  if (hk.best_tour.empty()) {
    // The DP is exhaustive, so no tour means the partial problem is
    // infeasible.
    return PartialSolution{
        .lb = std::numeric_limits<int>::max(),
        .ub = std::numeric_limits<int>::max(),
    };
  }
  const std::vector<StopId>& tour = hk.best_tour;
  assert(tour.front() == original_problem.boundary.start);
  assert(tour.back() == original_problem.boundary.end);

  // Reconstruct the paths through all original problem stops.
  std::vector<Path> original_paths =
      ComputeMinimalFeasiblePathsAlong(tour, original_problem.minimal);
  // There must be paths along a tour the DP found, because all paths in the
  // partial problem are also paths in the full problem.
  assert(!original_paths.empty());

  int best_duration = std::numeric_limits<int>::max();
  for (const Path& path : original_paths) {
    best_duration = std::min(best_duration, path.DurationSeconds());
  }
  // The best path along the tour must achieve exactly the DP's optimum,
  // because otherwise hk.best_val isn't the best duration in the partial
  // problem.
  assert(best_duration == hk.best_val);

  std::vector<PartialSolutionPath> paths;
  for (Path& path : original_paths) {
    if (path.DurationSeconds() != best_duration) {
      continue;
    }
    assert(path.merged_step.origin.stop == original_problem.boundary.start);
    assert(path.merged_step.destination.stop == original_problem.boundary.end);
    paths.push_back(
        PartialSolutionPath{
            .path = std::move(path),
            .subset_tour = tour,
        }
    );
  }

  return PartialSolution{
      .lb = hk.best_val, .ub = hk.best_val, .paths = std::move(paths)
  };
}

PartialSolution PartialSolveTwoOpt(
    const ProblemState& partial_problem,
    const ProblemState& original_problem,
    int known_lb,
    const TwoOptOptions& options,
    std::ostream* search_log
) {
  TwoOptResult two_opt =
      TwoOptSolve(partial_problem, known_lb, options, search_log);
  if (two_opt.best_tour.empty()) {
    // 2-opt is a heuristic, so finding nothing proves nothing.
    return PartialSolution{};
  }
  const std::vector<StopId>& tour = two_opt.best_tour;
  assert(tour.front() == original_problem.boundary.start);
  assert(tour.back() == original_problem.boundary.end);

  // Reconstruct the paths through all original problem stops.
  std::vector<Path> original_paths =
      ComputeMinimalFeasiblePathsAlong(tour, original_problem.minimal);
  // There must be paths along a tour the search found, because all paths in
  // the partial problem are also paths in the full problem.
  assert(!original_paths.empty());

  int best_duration = std::numeric_limits<int>::max();
  for (const Path& path : original_paths) {
    best_duration = std::min(best_duration, path.DurationSeconds());
  }
  // The best path along the tour must achieve exactly the search's value,
  // because that value is the exact optimum for this visit order.
  assert(best_duration == two_opt.best_val);

  std::vector<PartialSolutionPath> paths;
  for (Path& path : original_paths) {
    if (path.DurationSeconds() != best_duration) {
      continue;
    }
    assert(path.merged_step.origin.stop == original_problem.boundary.start);
    assert(path.merged_step.destination.stop == original_problem.boundary.end);
    paths.push_back(
        PartialSolutionPath{
            .path = std::move(path),
            .subset_tour = tour,
        }
    );
  }

  return PartialSolution{
      // 2-opt does not prove a lower bound.
      .lb = 0,
      .ub = two_opt.best_val,
      .paths = std::move(paths)
  };
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

  // Insertion is a heuristic, so it proves no lower bound.
  return PartialSolution{.ub = best_duration, .paths = std::move(best_paths)};
}

}  // namespace vats5
