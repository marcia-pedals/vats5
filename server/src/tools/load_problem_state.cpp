#include <fstream>
#include <iostream>

#include <limits>
#include <nlohmann/json.hpp>
#include <unordered_set>

#include "solver/branch_and_bound.h"
#include "solver/data.h"
#include "solver/graph_util.h"
#include "solver/relaxed_adjacency_list.h"
#include "solver/step_merge.h"
#include "solver/tarel_graph.h"

using namespace vats5;

const Path* FindShortestPath(std::span<const Path> paths) {
  if (paths.empty()) {
    return nullptr;
  }
  const Path* shortest = &paths[0];
  for (const Path& path : paths) {
    if (path.DurationSeconds() < shortest->DurationSeconds()) {
      shortest = &path;
    }
  }
  return shortest;
}

int RequiredStopsDistance(const Path& path, const std::unordered_set<StopId>& required_stops) {
  int distance = 0;
  path.VisitIntermediateStops([&](StopId x) {
    if (required_stops.contains(x)) {
      distance += 1;
    }
  });
  return distance;
}

PlainEdge FindFarthestPair(const ProblemState& state) {
  PlainEdge farthest_pair{.a=StopId{-1}, .b=StopId{-1}};
  int farthest_pair_distance = -1;
  for (StopId a : state.required_stops) {
    for (StopId b : state.required_stops) {
      const Path* shortest_path = FindShortestPath(state.completed.PathsBetween(a, b));
      if (!shortest_path) {
        continue;
      }
      int distance = RequiredStopsDistance(*shortest_path, state.required_stops);
      if (distance > farthest_pair_distance) {
        farthest_pair = PlainEdge{.a=a, .b=b};
        farthest_pair_distance = distance;
      }
    }
  }

  std::cout << state.StopName(farthest_pair.a) << " -> " << state.StopName(farthest_pair.b) << ": " << farthest_pair_distance << "\n";

  return farthest_pair;
}


void FindFarthestFromSet(const ProblemState& state, const std::unordered_set<StopId> stops) {
  StopId farthest_from_set;
  int farthest_from_set_distance = -1;

  for (StopId a : state.required_stops) {
    int closest_in_set_distance = std::numeric_limits<int>::max();
    for (StopId b : stops) {
      const Path* shortest_path = FindShortestPath(state.completed.PathsBetween(a, b));
      if (!shortest_path) {
        continue;
      }
      int distance = RequiredStopsDistance(*shortest_path, state.required_stops);
      if (distance < closest_in_set_distance) {
        closest_in_set_distance = distance;
      }
    }
    if (closest_in_set_distance == std::numeric_limits<int>::max()) {
      // TODO: Think about this.
      continue;
    }

    if (closest_in_set_distance > farthest_from_set_distance) {
      farthest_from_set = a;
      farthest_from_set_distance = closest_in_set_distance;
    }
  }

  std::cout << state.StopName(farthest_from_set) << ": " << farthest_from_set_distance << "\n";
}

struct Solution {
  int duration_seconds;
  std::vector<StopId> perm;
};

std::vector<Solution> FindBestPermutation(const ProblemState& state) {
  std::vector<StopId> stops(state.required_stops.begin(), state.required_stops.end());

  // Remove boundary stops
  std::erase(stops, state.boundary.start);
  std::erase(stops, state.boundary.end);

  // Sort for std::next_permutation
  std::sort(stops.begin(), stops.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });

  int best_duration = std::numeric_limits<int>::max();
  std::vector<Solution> best_permutations;
  int permutation_count = 0;

  do {
    ++permutation_count;

    // Build route: START -> stops[0] -> ... -> stops[n-1] -> END
    std::vector<StopId> route;
    route.push_back(state.boundary.start);
    for (StopId stop : stops) {
      route.push_back(stop);
    }
    route.push_back(state.boundary.end);

    // Compute paths along this route
    std::vector<Step> feasible_steps;
    feasible_steps.push_back(Step::PrimitiveFlex(
        state.boundary.start, state.boundary.start, 0, TripId::NOOP));

    bool valid = true;
    for (size_t i = 0; i + 1 < route.size(); ++i) {
      StopId from = route[i];
      StopId to = route[i + 1];
      auto paths = state.completed.PathsBetween(from, to);
      if (paths.empty()) {
        valid = false;
        break;
      }
      std::vector<Step> next_steps;
      for (const Path& p : paths) {
        next_steps.push_back(p.merged_step);
      }
      feasible_steps = PairwiseMergedSteps(feasible_steps, next_steps);
    }

    if (!valid || feasible_steps.empty()) {
      continue;
    }

    // Remove steps with negative start time
    std::erase_if(feasible_steps, [](const Step& step) {
      return step.origin.time < TimeSinceServiceStart{0};
    });

    if (feasible_steps.empty()) {
      continue;
    }

    // Find minimum duration for this permutation
    auto best_step_it = std::min_element(feasible_steps.begin(), feasible_steps.end(),
        [](const Step& a, const Step& b) {
          return a.DurationSeconds() < b.DurationSeconds();
        });

    int duration = best_step_it->DurationSeconds();
    if (duration < best_duration) {
      best_duration = duration;
      best_permutations.clear();
      best_permutations.push_back(Solution{
        .duration_seconds=duration,
        .perm=stops,
      });
    } else if (duration == best_duration) {
      best_permutations.push_back(Solution{
        .duration_seconds=duration,
        .perm=stops,
      });
    }
  } while (std::next_permutation(stops.begin(), stops.end(),
      [](StopId a, StopId b) { return a.v < b.v; }));

  return best_permutations;
}

std::vector<Path> SolveWith(
  const ProblemState& state,
  std::unordered_set<StopId> stops
) {
  stops.insert(state.boundary.start);
  stops.insert(state.boundary.end);
  ProblemState reduced = ReduceToMinimalRequiredStops(state, stops);

  std::vector<Solution> solutions = FindBestPermutation(reduced);
  // BranchAndBoundResult bnb_solution = BranchAndBoundSolve(reduced, &std::cout);
  // std::vector<StopId> bnb_perm;
  // bnb_solution.path->VisitIntermediateStops([&](StopId x) {
  //   bnb_perm.push_back(x);
  // });
  // std::vector<Solution> solutions = {Solution{
  //   .duration_seconds=bnb_solution.best_ub,
  //   .perm=bnb_perm,
  // }};

  std::vector<Path> all_paths;
  for (const Solution& solution : solutions) {
    std::vector<StopId> perm = solution.perm;
    perm.insert(perm.begin(), state.boundary.start);
    perm.insert(perm.end(), state.boundary.end);

    // TODO: This should really be a shared function.
    auto first_step_paths = state.completed.PathsBetween(perm[0], perm[1]);
    std::vector<Path> paths(first_step_paths.begin(), first_step_paths.end());
    std::vector<Step> steps = state.completed.MergedStepsBetween(perm[0], perm[1]);
    for (int i = 1; i < perm.size() - 1; ++i) {
      std::vector<StepProvenance> provenance;
      auto next_paths = state.completed.PathsBetween(perm[i], perm[i + 1]);
      std::vector<Step> next_steps = state.completed.MergedStepsBetween(perm[i], perm[i + 1]);
      std::vector<Step> new_steps = PairwiseMergedSteps(steps, next_steps, &provenance);
      std::vector<Path> new_paths;
      for (int j = 0; j < new_steps.size(); ++j) {
        new_paths.push_back(Path{new_steps[j], paths[provenance[j].ab_index].steps});
        for (const Step& step : next_paths[provenance[j].bc_index].steps) {
          new_paths.back().steps.push_back(step);
        }
      }

      paths = std::move(new_paths);
      steps = std::move(new_steps);
    }

    std::erase_if(paths, [&](const Path& path) {
      return path.DurationSeconds() > solution.duration_seconds;
    });

    for (Path& path : paths) {
      all_paths.push_back(std::move(path));
    }
  }

  return all_paths;
}

struct GreedyAddResult {
  StopId best_stop;
  int best_stop_hit_count;
  Path solution;
};

std::vector<GreedyAddResult> GreedyAddStop(const ProblemState& state, const std::unordered_set<StopId>& stops) {
  std::vector<GreedyAddResult> results;
  std::unordered_set<StopId> stops_in_results;
  std::unordered_set<StopId> stops_seen_in_solutions;
  int best_hit_count = -1;

  for (StopId x : state.required_stops) {
    if (x == state.boundary.start || x == state.boundary.end || stops.contains(x)) {
      continue;
    }
    if (stops_seen_in_solutions.contains(x)) {
      continue;
    }
    std::unordered_set<StopId> stops_with_x = stops;
    stops_with_x.insert(x);

    std::vector<Path> paths = SolveWith(state, stops_with_x);

    for (const Path& path : paths) {
      std::unordered_set<StopId> solution_stops;
      path.VisitIntermediateStops([&](const StopId s) {
        solution_stops.insert(s);
      });
      solution_stops.insert(path.steps[0].origin.stop);
      solution_stops.insert(path.steps.back().destination.stop);
      int hit_count = static_cast<int>(solution_stops.size());
      if (hit_count > best_hit_count) {
        best_hit_count = hit_count;
        results.clear();
        stops_in_results.clear();
        results.push_back(GreedyAddResult{
          .best_stop = x,
          .best_stop_hit_count = hit_count,
          .solution = path,
        });
        stops_in_results.insert(x);
        for (StopId s : solution_stops) {
          stops_seen_in_solutions.insert(s);
        }
      } else if (hit_count == best_hit_count) {
        for (StopId s : solution_stops) {
          stops_seen_in_solutions.insert(s);
        }
        if (!stops_in_results.contains(x)) {
          results.push_back(GreedyAddResult{
            .best_stop = x,
            .best_stop_hit_count = hit_count,
            .solution = path,
          });
          stops_in_results.insert(x);
        }
      }
    }
  }

  return results;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: load_problem_state <input_path.json>\n";
        return 1;
    }

    std::string input_path = argv[1];
    std::cout << "Loading problem state from: " << input_path << "\n";

    std::ifstream in(input_path);
    if (!in.is_open()) {
        std::cerr << "Error: could not open " << input_path << "\n";
        return 1;
    }

    nlohmann::json j = nlohmann::json::parse(in);
    ProblemState state = j.get<ProblemState>();

    PlainEdge farthest_pair = FindFarthestPair(state);
    std::unordered_set<StopId> stops = {farthest_pair.a, farthest_pair.b};

    // std::unordered_set<StopId> stops = {
    //   state.StopIdFromName("Millbrae BART"),
    //   state.StopIdFromName("Richmond BART/Amtrak"),
    //   state.StopIdFromName("Antioch"),
    //   state.StopIdFromName("Dublin / Pleasanton BART"),
    //   state.StopIdFromName("OAK"),
    //   state.StopIdFromName("Berryessa / North San Jose"),
    // };
    std::vector<GreedyAddResult> results;
    do {
      results = GreedyAddStop(state, stops);
      std::cout << "Equally-best options (" << results.size() << "):\n";
      for (const GreedyAddResult& r : results) {
        std::cout
          << "  " << state.StopName(r.best_stop)
          << ": " << r.best_stop_hit_count
          << " [" << TimeSinceServiceStart{r.solution.DurationSeconds()}.ToString() << "]\n";
      }
      const GreedyAddResult& chosen = results[0];
      std::cout << "Choosing: " << state.StopName(chosen.best_stop) << "\n";
      std::cout << "  Path: ";
      std::cout << state.StopName(chosen.solution.steps[0].origin.stop);
      for (const Step& step : chosen.solution.steps) {
        std::cout << " -> " << state.StopName(step.destination.stop);
      }
      std::cout << "\n";
      stops.insert(chosen.best_stop);
    } while (results[0].best_stop_hit_count < state.required_stops.size());

    return 0;
}
