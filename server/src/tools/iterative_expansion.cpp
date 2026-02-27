#include <CLI/CLI.hpp>
#include <algorithm>
#include <chrono>
#include <climits>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "algorithm/union_find.h"
#include "solver/branch_and_bound.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"
#include "visualization/sqlite_wrapper.h"
#include "visualization/visualization.h"

using namespace vats5;

struct StopDistance {
  int distance;
  StopId<> unvisited_stop;
  StopId<> nearest_path_stop;

  bool operator<(const StopDistance& o) const { return distance < o.distance; }
};

struct VizStep {
  std::string origin_stop_id;
  std::string destination_stop_id;
  int depart_time;
  int arrive_time;
  int is_flex;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VizStep, origin_stop_id, destination_stop_id, depart_time, arrive_time, is_flex)

struct VizPath {
  std::vector<VizStep> steps;
  int duration;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VizPath, steps, duration)

struct PartialSolutionData {
  std::vector<std::string> leaves;
  std::vector<VizPath> paths;
  VizPath best_path;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PartialSolutionData, leaves, paths, best_path)


// Builds an MST over the required stops (excluding start/end) using min
// duration as edge weight, and returns the leaves (degree-1 nodes).
std::unordered_set<StopId<>> MstLeaves(const ProblemState<>& state) {
  // Collect required stops, excluding START and END.
  std::vector<StopId<>> stops(
      state.required_stops.begin(), state.required_stops.end()
  );
  std::erase_if(stops, [&](StopId<> s) {
    return s == state.boundary.start || s == state.boundary.end;
  });
  std::ranges::sort(stops);
  int n = static_cast<int>(stops.size());

  // Build weighted undirected edges: for each pair of required stops, the
  // weight is the min duration step between them in either direction.
  struct Edge {
    int u, v;  // indices into stops vector
    int weight;
  };
  std::vector<Edge> edges;

  for (int i = 0; i < n; i++) {
    for (int k = i + 1; k < n; k++) {
      auto paths = state.completed.PathsBetweenBidirectional(stops[i], stops[k]);
      auto it = std::ranges::min_element(paths, {}, [](const auto& path) { return path.DurationSeconds(); });
      if (it != paths.end()) {
        edges.push_back({i, k, it->DurationSeconds()});
      }
    }
  }

  // Kruskal's MST: sort edges by weight, greedily add via union-find.
  std::ranges::sort(edges, {}, [](const Edge& edge) { return edge.weight; });

  UnionFind uf(n);
  std::vector<int> degree(n, 0);

  for (const Edge& e : edges) {
    if (uf.Unite(e.u, e.v)) {
      degree[e.u]++;
      degree[e.v]++;
    }
  }

  // Collect leaves (degree 1 in the MST).
  std::unordered_set<StopId<>> leaves;
  for (int i = 0; i < n; i++) {
    if (degree[i] == 1) {
      leaves.insert(stops[i]);
    }
  }
  return leaves;
}

// A single path along with the tour of required stops that generated it.
struct PartialSolutionPath {
  // A path that achieves the minimum duration in the partial problem.
  // Includes START and END. All original-problem stops that this path passes
  // through are included as intermediate stops.
  Path path;

  // The tour of the required subset that generates `path`. Includes START and END.
  std::vector<StopId<>> subset_tour;
};

int CountRequiredStops(const Path& path, const std::unordered_set<StopId<>>& required_stops) {
  std::unordered_set<StopId<>> required_visited;
  path.VisitAllStops([&](StopId<> stop) {
    if (required_stops.contains(stop)) {
      required_visited.insert(stop);
    }
  });
  return required_visited.size();
}

// A "partial problem" is a problem where the paths are required to visit a
// certain subset of the required stops. This is a solution to such a problem.
struct PartialSolution {
  std::vector<PartialSolutionPath> paths;

  // Returns the path that visits the most required stops.
  // Returns paths.end() if no paths are available.
  std::vector<PartialSolutionPath>::const_iterator BestPathByRequiredStops(
      const std::unordered_set<StopId<>>& required_stops) const {
    return std::ranges::max_element(paths, {}, [&](const PartialSolutionPath& sol_path) {
      return CountRequiredStops(sol_path.path, required_stops);
    });
  }
};

PartialSolution PartialSolveBranchAndBound(
  std::unordered_set<StopId<>> required_subset,
  const ProblemState<>& original_problem
) {
  required_subset.insert(original_problem.boundary.start);
  required_subset.insert(original_problem.boundary.end);
  ProblemState<> partial_problem = MakeProblemState(
    MakeAdjacencyList(ReduceToMinimalSystemPaths(original_problem.minimal, required_subset).AllMergedSteps()),
    original_problem.boundary,
    required_subset,
    original_problem.stop_infos,
    original_problem.step_partition_names,
    original_problem.original_edges
  );

  auto bb_result = BranchAndBoundSolve(partial_problem, &std::cout);
  if (bb_result.best_paths.empty()) {
    return PartialSolution{};
  }

  // Find the original problem paths corresponding to the partial problem paths.
  std::vector<PartialSolutionPath> paths;
  std::set<std::vector<StopId<>>> seen_tours;
  for (const Path& bb_path : bb_result.best_paths) {
    // Reconstruct the tour of partial problem stops.
    std::vector<StopId<>> tour;
    bb_path.VisitAllStops([&](StopId<> bb_result_stop) {
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
        ComputeMinimalFeasiblePathsAlong(tour, original_problem.completed);

    // Some of these paths might have duration longer than the bb_path. Disregard these.
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
      assert(path.merged_step.destination.stop == original_problem.boundary.end);
      assert(path.DurationSeconds() == bb_path.DurationSeconds());
      paths.push_back(PartialSolutionPath{
        .path = path,
        .subset_tour = tour,
      });
    }
  }

  // TODO: Think about wither `paths` could contain duplicate
  // paths or other non-minimality.

  return PartialSolution{.paths = std::move(paths)};
}

PartialSolution NaivelyExtendPartialSolution(
  const ProblemState<>& original_problem,
  const std::vector<StopId<>>& partial_solution_tour,
  StopId<> new_stop
) {
  // Create an extended tour with the new stop inserted at index 0.
  std::vector<StopId<>> extended_tour;
  extended_tour.reserve(partial_solution_tour.size() + 1);
  extended_tour.push_back(new_stop);
  extended_tour.append_range(partial_solution_tour);

  int best_duration = std::numeric_limits<int>::max();
  std::vector<PartialSolutionPath> best_paths;

  // Figure out the duration of the extended tour with the new stop in each
  // position, by swapping it forwards. Intentionally don't try the new stop
  // first or last because first and last should always be START and END.
  for (int new_stop_index = 1; new_stop_index + 1 < extended_tour.size(); ++new_stop_index) {
    // Swap forwards.
    std::swap(extended_tour[new_stop_index - 1], extended_tour[new_stop_index]);
    assert(extended_tour[new_stop_index] == new_stop);

    std::vector<Path> paths =
        ComputeMinimalFeasiblePathsAlong(extended_tour, original_problem.completed);
    auto best_path_it = std::ranges::min_element(paths, {}, [](const Path& path) {
      return path.DurationSeconds();
    });
    if (best_path_it == paths.end()) {
      continue;
    }
    const Path& best_path = *best_path_it;
    if (best_path.DurationSeconds() < best_duration) {
      best_duration = best_path.DurationSeconds();
      best_paths.clear();
    }
    if (best_path.DurationSeconds() == best_duration) {
      // TODO: A lot of allocation and copying here, and this might be a pretty
      // hot loop. And then we throw it all away if we find a better duration.
      for (const Path& path : paths) {
        if (path.DurationSeconds() > best_path.DurationSeconds()) {
          continue;
        }
        best_paths.push_back({
          .path=path,
          .subset_tour=extended_tour,
        });
      }
    }
  }

  return PartialSolution{.paths=best_paths};
}

PartialSolutionPath GreedilyExtendAsMuchAsPossibleWithoutIncreasingDuration(const ProblemState<>& original_problem, const PartialSolutionPath& partial_path) {
  PartialSolutionPath result = partial_path;

  while (true) {
    std::unordered_set<StopId<>> unvisited(original_problem.required_stops);
    result.path.VisitAllStops([&](StopId<> stop) {
      unvisited.erase(stop);
    });

    std::vector<PartialSolutionPath> improved;
    for (StopId<> new_stop : unvisited) {
      PartialSolution extended = NaivelyExtendPartialSolution(original_problem, result.subset_tour, new_stop);
      auto best_extended_it = extended.BestPathByRequiredStops(original_problem.required_stops);
      if (
        best_extended_it == extended.paths.end() ||
        best_extended_it->path.DurationSeconds() > result.path.DurationSeconds()
      ) {
        continue;
      }
      improved.push_back(*best_extended_it);
    }

    auto best_improved_it = std::ranges::max_element(improved, {}, [&](const PartialSolutionPath& path) {
      return CountRequiredStops(path.path, original_problem.required_stops);
    });
    if (best_improved_it == improved.end()) {
      break;
    }

    result = *best_improved_it;
  }

  return result;
}

// // Tries all permutations of `required_subset` as intermediate stops between start and
// // end, and returns the permutation yielding the shortest feasible path.
// PartialSolution PartialSolveBruteForce(
//     std::unordered_set<StopId<>> required_subset,
//     const ProblemState<>& state) {
//   std::vector<StopId<>> candidate_perm(required_subset.begin(), required_subset.end());
//   std::sort(candidate_perm.begin(), candidate_perm.end());

//   int best_duration = INT_MAX;
//   PartialSolution best;

//   do {
//     std::vector<StopId<>> sequence;
//     sequence.push_back(state.boundary.start);
//     sequence.insert(sequence.end(), candidate_perm.begin(), candidate_perm.end());
//     sequence.push_back(state.boundary.end);

//     std::vector<Path> paths =
//         ComputeMinimalFeasiblePathsAlong(sequence, state.completed);

//     auto min_it = std::min_element(
//         paths.begin(), paths.end(),
//         [](const Path& a, const Path& b) {
//           return a.DurationSeconds() < b.DurationSeconds();
//         });
//     if (min_it != paths.end() && min_it->DurationSeconds() < best_duration) {
//       best_duration = min_it->DurationSeconds();
//       std::erase_if(paths, [&](const Path& p) {
//         return p.DurationSeconds() != best_duration;
//       });
//       best.paths = std::move(paths);
//     }
//   } while (std::next_permutation(candidate_perm.begin(), candidate_perm.end()));

//   return best;
// }

// For each required stop not on the path, computes its shortest "distance" to
// the path. Distance from required stop x to path stop p: find the
// min-duration path from x to p in state.completed, then count how many
// required stops are on that path (including both endpoints). The distance
// from x to the overall path is the minimum across all path stops p.
// Returns results sorted by distance (ascending).
std::vector<StopDistance> RequiredStopDistances(
    const Path& path,
    const ProblemState<>& state) {
  std::unordered_set<StopId<>> visited;
  path.VisitAllStops([&](StopId<> s) { visited.insert(s); });

  std::vector<StopDistance> distances;
  for (StopId<> x : state.required_stops) {
    if (visited.contains(x)) {
      continue;
    }
    int min_dist = INT_MAX;
    StopId<> nearest_stop{};
    for (StopId<> p : visited) {
      if (p == state.boundary.start || p == state.boundary.end) {
        continue;
      }
      auto paths_xp = state.completed.PathsBetween(x, p);
      const Path* shortest = nullptr;
      for (const Path& candidate : paths_xp) {
        if (!shortest ||
            candidate.DurationSeconds() < shortest->DurationSeconds()) {
          shortest = &candidate;
        }
      }
      if (shortest) {
        // Count required stops on the connecting path, including endpoints.
        int count = 0;
        shortest->VisitAllStops([&](StopId<> s) {
          if (state.required_stops.contains(s)) {
            count++;
          }
        });
        if (count < min_dist) {
          min_dist = count;
          nearest_stop = p;
        }
      }
    }
    distances.push_back({min_dist, x, nearest_stop});
  }
  std::sort(distances.begin(), distances.end());
  return distances;
}

int main(int argc, char* argv[]) {
  CLI::App app{"Iterative expansion tool"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to problem state JSON")
      ->required();

  CLI11_PARSE(app, argc, argv);

  std::string viz_sqlite_path = viz::VizSqlitePath(input_path);

  // Load problem state.
  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << "\n";
    return 1;
  }

  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState<> state = j.get<ProblemState<>>();

  std::unordered_set<StopId<>> required_subset = MstLeaves(state);

  // Generate run timestamp.
  auto now = std::chrono::system_clock::now();
  auto tt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ts;
  ts << std::put_time(std::localtime(&tt), "%Y-%m-%dT%H:%M:%S");
  std::string run_timestamp = ts.str();

  auto ToVizPath = [&state](const Path& path) -> VizPath {
    VizPath vp;
    vp.duration = path.DurationSeconds();
    for (const Step& s : path.steps) {
      vp.steps.push_back({
        state.stop_infos.at(s.origin.stop).gtfs_stop_id.v,
        state.stop_infos.at(s.destination.stop).gtfs_stop_id.v,
        s.origin.time.seconds,
        s.destination.time.seconds,
        s.is_flex ? 1 : 0,
      });
    }
    return vp;
  };

  for (int iteration = 0; ; iteration++) {
    std::cout << "=== Iteration " << iteration << ": branch and bound on "
              << required_subset.size() << " leaves ===\n";
    auto solution = PartialSolveBranchAndBound(required_subset, state);

    // Choose the path that visits the most required stops.
    auto best_solution_path_it = solution.BestPathByRequiredStops(state.required_stops);
    if (best_solution_path_it == solution.paths.end()) {
      std::cout << "No feasible paths found.\n";
      return 1;
    }

    PartialSolutionPath best_solution_path = *best_solution_path_it;
    std::cout << "Before greedy improve: " << best_solution_path.path.IntermediateStopCount() << "\n";
    best_solution_path = GreedilyExtendAsMuchAsPossibleWithoutIncreasingDuration(state, best_solution_path);
    std::cout << "After greedy improve: " << best_solution_path.path.IntermediateStopCount() << "\n";

    const Path& best_path = best_solution_path.path;
    const std::vector<StopDistance> distances = RequiredStopDistances(best_path, state);

    // Write partial solution to viz SQLite.
    {
      PartialSolutionData data;
      for (StopId<> leaf : required_subset) {
        data.leaves.push_back(state.stop_infos.at(leaf).gtfs_stop_id.v);
      }
      for (const PartialSolutionPath& sol_path : solution.paths) {
        data.paths.push_back(ToVizPath(sol_path.path));
      }
      data.best_path = ToVizPath(best_path);

      viz::SqliteDb db(viz_sqlite_path);
      viz::SqliteStmt stmt(db,
          "INSERT INTO partial_solutions (run_timestamp, iteration, data) "
          "VALUES (?, ?, ?)");
      stmt.bind_text(1, run_timestamp.c_str());
      stmt.bind_int(2, iteration);
      std::string data_str = nlohmann::json(data).dump();
      stmt.bind_text(3, data_str.c_str());
      stmt.step_and_reset();
    }

    std::cout << "\nBest duration: " << TimeSinceServiceStart{best_path.DurationSeconds()}.ToString() << "\n";

    std::cout << "Path (" << best_path.steps.size() << " steps):\n";
    for (const Step& step : best_path.steps) {
      std::cout << "  " << state.StopName(step.origin.stop) << " ("
                << step.origin.time.ToString() << ") -> "
                << state.StopName(step.destination.stop) << " ("
                << step.destination.time.ToString() << ")\n";
    }

    if (distances.empty()) {
      std::cout << "\nAll required stops are visited.\n";
      break;
    }

    std::cout << "\nRequired stops NOT visited (" << distances.size() << "):\n";
    for (const auto& sd : distances) {
      std::cout << "  " << state.StopName(sd.unvisited_stop) << " (distance: " << sd.distance
                << ", nearest: " << state.StopName(sd.nearest_path_stop) << ")\n";
    }

    // Add the farthest unvisited stop to leaves for the next iteration.
    StopId<> farthest = distances.back().unvisited_stop;
    std::cout << "\nAdding farthest stop: " << state.StopName(farthest) << "\n\n";
    required_subset.insert(farthest);
  }

  return 0;
}
