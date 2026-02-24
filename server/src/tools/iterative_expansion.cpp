#include <CLI/CLI.hpp>
#include <algorithm>
#include <chrono>
#include <climits>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
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
  StopId unvisited_stop;
  StopId nearest_path_stop;

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
std::unordered_set<StopId> MstLeaves(const ProblemState& state) {
  // Collect required stops, excluding START and END.
  std::vector<StopId> stops(
      state.required_stops.begin(), state.required_stops.end()
  );
  std::erase_if(stops, [&](StopId s) {
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
  std::unordered_set<StopId> leaves;
  for (int i = 0; i < n; i++) {
    if (degree[i] == 1) {
      leaves.insert(stops[i]);
    }
  }
  return leaves;
}

// A "partial problem" is a problem where the paths are required to visit a
// certain subset of the required stops. This is a solution to such a problem.
struct PartialSolution {
  // Paths that achieve the minimum duraiton in the partial problem.
  //
  // All original-problem stops that these paths pass through are included as
  // intermediate stops.
  //
  // TODO: Specify more precisely which paths. Is it all of them? Or all of them
  // satisfying some property?
  std::vector<Path> paths;
};


PartialSolution PartialSolveBranchAndBound(
  std::unordered_set<StopId> required_subset,
  const ProblemState& original_problem
) {
  required_subset.insert(original_problem.boundary.start);
  required_subset.insert(original_problem.boundary.end);
  ProblemState partial_problem = MakeProblemState(
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
  std::vector<Path> original_problem_paths;
  std::set<std::vector<StopId>> seen_sequences;
  for (const Path& bb_path : bb_result.best_paths) {
    // Reconstruct the sequence of partial problem stops.
    std::vector<StopId> sequence;
    auto AppendStop = [&](StopId bb_result_stop) {
      ExpandStop(bb_result_stop, bb_result.original_edges, sequence);
    };
    AppendStop(bb_path.merged_step.origin.stop);
    bb_path.VisitIntermediateStops(AppendStop);
    AppendStop(bb_path.merged_step.destination.stop);

    // If we have already seen this sequence then we don't need to process it
    // again.
    if (!seen_sequences.insert(sequence).second) {
      continue;
    }

    // Reconstruct the paths through all original problem stops.
    std::vector<Path> more_original_paths =
        ComputeMinimalFeasiblePathsAlong(sequence, original_problem.completed);

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
      assert(path.DurationSeconds() == bb_path.DurationSeconds());
    }

    original_problem_paths.append_range(more_original_paths);
  }

  // TODO: Think about wither `original_problem_paths` could contain duplicate
  // paths or other non-minimality.

  return PartialSolution{std::move(original_problem_paths)};
}

// Tries all permutations of `required_subset` as intermediate stops between start and
// end, and returns the permutation yielding the shortest feasible path.
PartialSolution PartialSolveBruteForce(
    std::unordered_set<StopId> required_subset,
    const ProblemState& state) {
  std::vector<StopId> candidate_perm(required_subset.begin(), required_subset.end());
  std::sort(candidate_perm.begin(), candidate_perm.end());

  int best_duration = INT_MAX;
  PartialSolution best;

  do {
    std::vector<StopId> sequence;
    sequence.push_back(state.boundary.start);
    sequence.insert(sequence.end(), candidate_perm.begin(), candidate_perm.end());
    sequence.push_back(state.boundary.end);

    std::vector<Path> paths =
        ComputeMinimalFeasiblePathsAlong(sequence, state.completed);

    auto min_it = std::min_element(
        paths.begin(), paths.end(),
        [](const Path& a, const Path& b) {
          return a.DurationSeconds() < b.DurationSeconds();
        });
    if (min_it != paths.end() && min_it->DurationSeconds() < best_duration) {
      best_duration = min_it->DurationSeconds();
      std::erase_if(paths, [&](const Path& p) {
        return p.DurationSeconds() != best_duration;
      });
      best.paths = std::move(paths);
    }
  } while (std::next_permutation(candidate_perm.begin(), candidate_perm.end()));

  return best;
}

// For each required stop not on the path, computes its shortest "distance" to
// the path. Distance from required stop x to path stop p: find the
// min-duration path from x to p in state.completed, then count how many
// required stops are on that path (including both endpoints). The distance
// from x to the overall path is the minimum across all path stops p.
// Returns results sorted by distance (ascending).
std::vector<StopDistance> RequiredStopDistances(
    const Path& path,
    const ProblemState& state) {
  std::unordered_set<StopId> visited;
  visited.insert(path.steps.front().origin.stop);
  path.VisitIntermediateStops([&](StopId s) { visited.insert(s); });
  visited.insert(path.steps.back().destination.stop);

  std::vector<StopDistance> distances;
  for (StopId x : state.required_stops) {
    if (visited.contains(x)) {
      continue;
    }
    int min_dist = INT_MAX;
    StopId nearest_stop{};
    for (StopId p : visited) {
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
        if (state.required_stops.contains(x)) count++;
        shortest->VisitIntermediateStops([&](StopId s) {
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
  ProblemState state = j.get<ProblemState>();

  std::unordered_set<StopId> required_subset = MstLeaves(state);

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
    auto best_path_it = std::ranges::max_element(solution.paths, {}, [&](const Path& path) {
      int num_required_visited = 0;
      path.VisitIntermediateStops([&](StopId stop) {
        if (state.required_stops.contains(stop)) {
          num_required_visited += 1;
        }
      });
      return num_required_visited;
    });

    if (best_path_it == solution.paths.end()) {
      std::cout << "No feasible paths found.\n";
      return 1;
    }

    const Path& best_path = *best_path_it;
    const std::vector<StopDistance> distances = RequiredStopDistances(best_path, state);

    // Write partial solution to viz SQLite.
    {
      PartialSolutionData data;
      for (StopId leaf : required_subset) {
        data.leaves.push_back(state.stop_infos.at(leaf).gtfs_stop_id.v);
      }
      for (const Path& p : solution.paths) {
        data.paths.push_back(ToVizPath(p));
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
    StopId farthest = distances.back().unvisited_stop;
    std::cout << "\nAdding farthest stop: " << state.StopName(farthest) << "\n\n";
    required_subset.insert(farthest);
  }

  return 0;
}
