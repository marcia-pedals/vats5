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
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "algorithm/union_find.h"
#include "gtfs/gtfs.h"
#include "solver/branch_and_bound.h"
#include "solver/data.h"
#include "solver/step_merge.h"
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
  // Flex/walking trips have no GTFS route+direction.
  std::optional<std::string> route_direction_id;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VizStep,
    origin_stop_id,
    destination_stop_id,
    depart_time,
    arrive_time,
    is_flex,
    route_direction_id
)

struct VizPath {
  std::vector<VizStep> steps;           // Collapsed steps (grouped by trip)
  std::vector<VizStep> original_steps;  // Original uncollapsed steps
  int duration;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VizPath, steps, original_steps, duration)

struct PartialSolutionData {
  std::vector<std::string> leaves;
  std::vector<VizPath> paths;
  VizPath best_path;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PartialSolutionData, leaves, paths, best_path
)

// Builds an MST over the required groups (excluding start/end) using min
// duration as edge weight, and returns the leaves (degree-1 nodes).
//
// Returns flattened groups (all stops in all groups rather than just the
// representatives).
std::unordered_set<StopId> MstLeaves(const ProblemState& state) {
  // Collect all stops, excluding START and END.
  std::vector<StopId> stops;
  stops.reserve(state.minimal.NumStops());
  for (int i = 0; i < state.minimal.NumStops(); ++i) {
    stops.push_back(StopId{i});
  }
  int n = static_cast<int>(stops.size());
  std::unordered_map<StopId, int> stop_index;
  for (int i = 0; i < n; ++i) {
    stop_index[stops[i]] = i;
  }

  std::vector weights(n * n, std::numeric_limits<int>::max());
  weights.reserve(n * n);
  // TODO: Change it to use minimal. BUT we might need to first reduce it to be
  // a graph only on required stops so that these "intermediate stops" never
  // appear as leaves.
  for (const Step& step : state.completed.AllMergedSteps()) {
    if (step.origin.stop == state.boundary.start ||
        step.origin.stop == state.boundary.end ||
        step.destination.stop == state.boundary.start ||
        step.destination.stop == state.boundary.end) {
      continue;
    }
    int a = stop_index.at(state.required.Representative(step.origin.stop));
    int b = stop_index.at(state.required.Representative(step.destination.stop));
    if (a > b) {
      std::swap(a, b);
    }
    int weight = step.DurationSeconds();
    weights[a * n + b] = std::min(weights[a * n + b], weight);
  }

  // Kruskal's MST: sort edges by weight, greedily add via union-find.
  std::vector<int> ordered_edges(n * n);
  std::iota(ordered_edges.begin(), ordered_edges.end(), 0);
  std::erase_if(ordered_edges, [&](int edge_index) {
    return weights[edge_index] == std::numeric_limits<int>::max();
  });
  std::ranges::sort(ordered_edges, {}, [&](int edge_index) {
    return weights[edge_index];
  });

  UnionFind uf(n);
  std::vector<int> degree(n, 0);
  for (int edge_index : ordered_edges) {
    int a = edge_index / n;
    int b = edge_index % n;
    if (uf.Unite(a, b)) {
      degree[a]++;
      degree[b]++;
    }
  }

  // Collect leaves (degree 1 in the MST).
  std::unordered_set<StopId> leaves;
  for (int i = 0; i < n; i++) {
    if (degree[i] == 1) {
      state.required.VisitGroupStops(stops[i], [&](StopId s) {
        leaves.insert(s);
      });
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

  // The tour of the required subset that generates `path`. Includes START and
  // END.
  std::vector<StopId> subset_tour;
};

int CountRequiredStops(const Path& path, const RequiredStops& required) {
  std::unordered_set<StopId> required_rep_visited;
  path.VisitAllStops([&](StopId stop) {
    if (required.Contains(stop)) {
      required_rep_visited.insert(required.Representative(stop));
    }
  });
  return required_rep_visited.size();
}

// A "partial problem" is a problem where the paths are required to visit a
// certain subset of the required stops. This is a solution to such a problem.
struct PartialSolution {
  std::vector<PartialSolutionPath> paths;
  ProblemState partial_problem;

  // Returns the path that visits the most required stops.
  // Returns paths.end() if no paths are available.
  std::vector<PartialSolutionPath>::const_iterator BestPathByRequiredStops(
      const RequiredStops& required
  ) const {
    return std::ranges::max_element(
        paths, {}, [&](const PartialSolutionPath& sol_path) {
          return CountRequiredStops(sol_path.path, required);
        }
    );
  }
};

PartialSolution PartialSolveBranchAndBound(
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

  auto bb_result = BranchAndBoundSolve(partial_problem, &std::cout);
  if (bb_result.best_paths.empty()) {
    return PartialSolution{.partial_problem = std::move(partial_problem)};
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
        ComputeMinimalFeasiblePathsAlong(tour, original_problem.completed);

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
      .paths = std::move(paths),
      .partial_problem = std::move(partial_problem),
  };
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

  // Figure out the duration of the extended tour with the new stop in each
  // position, by swapping it forwards. Intentionally don't try the new stop
  // first or last because first and last should always be START and END.
  for (int new_stop_index = 1; new_stop_index + 1 < extended_tour.size();
       ++new_stop_index) {
    // Swap forwards.
    std::swap(extended_tour[new_stop_index - 1], extended_tour[new_stop_index]);
    assert(extended_tour[new_stop_index] == new_stop);

    std::vector<Path> paths = ComputeMinimalFeasiblePathsAlong(
        extended_tour, original_problem.completed
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
      // TODO: A lot of allocation and copying here, and this might be a pretty
      // hot loop. And then we throw it all away if we find a better duration.
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

// // Tries all permutations of `required_subset` as intermediate stops between
// start and
// // end, and returns the permutation yielding the shortest feasible path.
// PartialSolution PartialSolveBruteForce(
//     std::unordered_set<StopId> required_subset,
//     const ProblemState& state) {
//   std::vector<StopId> candidate_perm(required_subset.begin(),
//   required_subset.end()); std::sort(candidate_perm.begin(),
//   candidate_perm.end());

//   int best_duration = INT_MAX;
//   PartialSolution best;

//   do {
//     std::vector<StopId> sequence;
//     sequence.push_back(state.boundary.start);
//     sequence.insert(sequence.end(), candidate_perm.begin(),
//     candidate_perm.end()); sequence.push_back(state.boundary.end);

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
//   } while (std::next_permutation(candidate_perm.begin(),
//   candidate_perm.end()));

//   return best;
// }

// For each required stop not on the path, computes its shortest "distance" to
// the path. Distance from required stop x to path stop p: find the
// min-duration path from x to p in state.completed, then count how many
// required stops are on that path (including both endpoints). The distance
// from x to the overall path is the minimum across all path stops p.
// Returns results sorted by distance (ascending).
//
// Takes the min over all stops in groups and returns it as distance to
// representative.
std::vector<StopDistance> RequiredStopDistances(
    const Path& path, const ProblemState& state
) {
  std::unordered_set<StopId> visited;
  std::unordered_set<StopId> visited_reps;
  path.VisitAllStops([&](StopId s) {
    visited.insert(s);
    visited_reps.insert(state.required.Representative(s));
  });

  std::unordered_map<StopId, StopDistance> rep_to_distance;
  for (StopId x : state.required.AllFlat()) {
    StopId rep = state.required.Representative(x);
    if (visited_reps.contains(rep)) {
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
        shortest->VisitAllStops([&](StopId s) {
          if (state.required.Contains(s)) {
            count++;
          }
        });
        if (count < min_dist) {
          min_dist = count;
          nearest_stop = p;
        }
      }
    }

    StopDistance distance{min_dist, x, nearest_stop};
    auto [it, inserted] = rep_to_distance.try_emplace(rep, distance);
    if (it->second.distance > distance.distance) {
      it->second = distance;
    }
  }

  std::vector<StopDistance> distances;
  distances.reserve(rep_to_distance.size());
  for (const auto& [rep, distance] : rep_to_distance) {
    distances.push_back(distance);
  }
  std::sort(distances.begin(), distances.end());
  return distances;
}

struct RefinementResult {
  int old_tour_new_weight;
  ProblemState state;
  StopId a, b;
};

RefinementResult Refine(
    const ProblemState& state,
    StopId a,
    StopId b,
    const std::vector<TarelEdge>& old_tour
) {
  // Mutable copies of all the `ProblemState` fields we'll be mutating.
  std::vector<Step> steps = state.minimal.AllSteps();
  ProblemBoundary boundary = state.boundary;
  RequiredStops required = state.required;
  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos =
      state.stop_infos;
  std::unordered_map<StopId, PlainEdge> original_edges = state.original_edges;
  StopId next_stop_id{state.minimal.NumStops()};

  assert(a != boundary.start);
  assert(a != boundary.end);
  assert(b != boundary.start);
  assert(b != boundary.end);

  std::unordered_map<StopId, std::vector<Step>> xs_to_a;
  std::unordered_map<StopId, std::vector<Step>> b_to_xs;
  std::vector<Step> a_to_b;
  for (const Step& step : steps) {
    if (step.destination.stop == a) {
      xs_to_a[step.origin.stop].push_back(step);
    }
    if (step.origin.stop == b) {
      b_to_xs[step.destination.stop].push_back(step);
    }
    if (step.origin.stop == a && step.destination.stop == b) {
      a_to_b.push_back(step);
    }
  }

  const ProblemStateStopInfo& info_a = stop_infos[a];
  const ProblemStateStopInfo& info_b = stop_infos[b];

  StopId join_a = next_stop_id;
  next_stop_id.v += 1;
  required.representative[join_a] = required.Representative(a);
  stop_infos[join_a] = ProblemStateStopInfo{
      GtfsStopId{"[" + info_a.gtfs_stop_id.v + "]-" + info_b.gtfs_stop_id.v},
      "[" + info_a.stop_name + "]-" + info_b.stop_name,
  };

  StopId join_b = next_stop_id;
  next_stop_id.v += 1;
  required.representative[join_b] = required.Representative(b);
  stop_infos[join_b] = ProblemStateStopInfo{
      GtfsStopId{info_a.gtfs_stop_id.v + "-[" + info_b.gtfs_stop_id.v + "]"},
      info_a.stop_name + "-[" + info_b.stop_name + "]",
  };

  for (const auto& [x, x_to_a] : xs_to_a) {
    std::vector<StepProvenance> prov;
    std::vector<Step> x_to_b = PairwiseMergedSteps(x_to_a, a_to_b, &prov);
    for (int i = 0; i < x_to_b.size(); ++i) {
      Step step = x_to_b[i];

      step.destination.stop = join_a;
      step.destination.partition =
          x_to_a[prov[i].ab_index].destination.partition;
      steps.push_back(step);

      Step continue_step{
          .origin = step.destination,
          .destination = step.destination,
          .is_flex = step.is_flex
      };
      continue_step.destination.stop = join_b;
      continue_step.destination.partition =
          a_to_b[prov[i].bc_index].destination.partition;
      steps.push_back(continue_step);
    }
  }

  for (const auto& [x, b_to_x] : b_to_xs) {
    for (Step step : b_to_x) {
      step.origin.stop = join_b;
      steps.push_back(step);
    }
  }

  std::erase_if(steps, [&](const Step& step) {
    return step.origin.stop == a && step.destination.stop == b;
  });

  // Build the new problem state from the stuff we've been mutating.
  ProblemState new_problem = MakeProblemState(
      MakeAdjacencyList(steps),
      std::move(boundary),
      std::move(required),
      std::move(stop_infos),
      state.step_partition_names,
      std::move(original_edges)
  );

  // Compute weight of old tour in new problem!
  std::vector<TarelEdge> new_problem_all_edges =
      MakeTarelEdges(new_problem.completed);
  std::vector<TarelEdge> new_tour;
  for (const TarelEdge& old_edge : old_tour) {
    TarelState old_origin = old_edge.origin;
    if (old_origin.stop == a) {
      old_origin.stop = join_a;
    }
    if (old_origin.stop == b) {
      old_origin.stop = join_b;
    }
    TarelState old_dest = old_edge.destination;
    if (old_dest.stop == a) {
      old_dest.stop = join_a;
    }
    if (old_dest.stop == b) {
      old_dest.stop = join_b;
    }
    auto edge_it = std::find_if(
        new_problem_all_edges.begin(),
        new_problem_all_edges.end(),
        [&](const TarelEdge& candidate) {
          return candidate.origin == old_origin &&
                 candidate.destination == old_dest;
        }
    );
    if (edge_it == new_problem_all_edges.end()) {
      std::cout << "Not found! " << new_problem.StopName(old_origin.stop)
                << " (" << new_problem.PartitionName(old_origin.partition)
                << ") -> " << new_problem.StopName(old_dest.stop) << " ("
                << new_problem.PartitionName(old_dest.partition) << ")\n";
      assert(false);
    }
    new_tour.push_back(*edge_it);
  }

  int old_tour_new_weight = 0;
  for (const TarelEdge& edge : new_tour) {
    old_tour_new_weight += edge.weight;
  }

  return {
      .old_tour_new_weight = old_tour_new_weight,
      .state = new_problem,
      .a = a,
      .b = b,
  };
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
  std::cout << "MST leaves:\n";
  for (StopId stop : required_subset) {
    std::cout << "  " << state.StopName(stop) << "\n";
  }

  // Generate run timestamp.
  auto now = std::chrono::system_clock::now();
  auto tt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ts;
  ts << std::put_time(std::localtime(&tt), "%Y-%m-%dT%H:%M:%S");
  std::string run_timestamp = ts.str();

  // Build trip_id -> route_direction_id mapping from the viz SQLite trips
  // table.
  std::unordered_map<int, std::string> trip_to_route;
  {
    viz::SqliteDb db(viz_sqlite_path);
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(
        db.handle(),
        "SELECT trip_id, route_direction_id FROM trips",
        -1,
        &stmt,
        nullptr
    );
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      int trip_id = sqlite3_column_int(stmt, 0);
      const char* route_direction_id =
          reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
      trip_to_route[trip_id] = route_direction_id;
    }
    sqlite3_finalize(stmt);
  }

  auto LookupRouteId =
      [&trip_to_route](TripId trip) -> std::optional<std::string> {
    auto it = trip_to_route.find(trip.v);
    if (it != trip_to_route.end()) return it->second;
    return std::nullopt;
  };

  auto StepToVizStep = [&state, &LookupRouteId](const Step& s) -> VizStep {
    return {
        state.stop_infos.at(s.origin.stop).gtfs_stop_id.v,
        state.stop_infos.at(s.destination.stop).gtfs_stop_id.v,
        s.origin.time.seconds,
        s.destination.time.seconds,
        s.is_flex ? 1 : 0,
        LookupRouteId(s.destination.trip),
    };
  };

  auto ToVizPath = [&StepToVizStep](const Path& path) -> VizPath {
    VizPath vp;
    vp.duration = path.DurationSeconds();

    // First, store the original steps (uncollapsed)
    for (const Step& s : path.steps) {
      vp.original_steps.push_back(StepToVizStep(s));
    }

    // Then, group consecutive steps by trip and merge each group
    std::vector<Step> merged_steps;

    size_t si = 0;
    while (si < path.steps.size()) {
      size_t group_end = si + 1;
      while (group_end < path.steps.size() &&
             path.steps[group_end].destination.trip ==
                 path.steps[si].destination.trip) {
        group_end++;
      }

      std::vector<Step> group_steps(
          path.steps.begin() + si, path.steps.begin() + group_end
      );
      merged_steps.push_back(ConsecutiveMergedSteps(group_steps));
      si = group_end;
    }

    // Normalize flex step times if necessary
    NormalizeConsecutiveSteps(merged_steps);

    // Store the collapsed steps
    for (const Step& s : merged_steps) {
      vp.steps.push_back(StepToVizStep(s));
    }
    return vp;
  };

  for (int iteration = 0;; iteration++) {
    std::cout << "=== Iteration " << iteration << ": branch and bound on "
              << required_subset.size() << " leaves ===\n";
    auto solution = PartialSolveBranchAndBound(required_subset, state);

    // Choose the path that visits the most required stops.
    auto best_solution_path_it =
        solution.BestPathByRequiredStops(state.required);
    if (best_solution_path_it == solution.paths.end()) {
      std::cout << "No feasible paths found.\n";
      return 1;
    }

    PartialSolutionPath best_solution_path = *best_solution_path_it;
    std::cout << "Before greedy improve: "
              << best_solution_path.path.IntermediateStopCount() << "\n";
    best_solution_path =
        GreedilyExtendAsMuchAsPossibleWithoutIncreasingDuration(
            state, best_solution_path
        );
    std::cout << "After greedy improve: "
              << best_solution_path.path.IntermediateStopCount() << "\n";

    const Path& best_path = best_solution_path.path;
    const std::vector<StopDistance> distances =
        RequiredStopDistances(best_path, state);

    // Write partial solution to viz SQLite.
    {
      PartialSolutionData data;
      for (StopId leaf : required_subset) {
        data.leaves.push_back(state.stop_infos.at(leaf).gtfs_stop_id.v);
      }
      for (const PartialSolutionPath& sol_path : solution.paths) {
        data.paths.push_back(ToVizPath(sol_path.path));
      }
      data.best_path = ToVizPath(best_path);

      viz::SqliteDb db(viz_sqlite_path);
      viz::SqliteStmt stmt(
          db,
          "INSERT INTO partial_solutions (run_timestamp, iteration, data) "
          "VALUES (?, ?, ?)"
      );
      stmt.bind_text(1, run_timestamp.c_str());
      stmt.bind_int(2, iteration);
      std::string data_str = nlohmann::json(data).dump();
      stmt.bind_text(3, data_str.c_str());
      stmt.step_and_reset();
    }

    // Write per-iteration viz file for the partial problem.
    {
      std::string iter_viz_path;
      {
        size_t dot_pos = viz_sqlite_path.rfind('.');
        if (dot_pos != std::string::npos) {
          iter_viz_path = viz_sqlite_path.substr(0, dot_pos) + "-iter-" +
                          std::to_string(iteration) +
                          viz_sqlite_path.substr(dot_pos);
        } else {
          iter_viz_path =
              viz_sqlite_path + "-iter-" + std::to_string(iteration);
        }
      }
      viz::WriteProblemStateVisualizationSqlite(
          solution.partial_problem, viz_sqlite_path, iter_viz_path
      );
      std::cout << "Wrote partial problem viz: " << iter_viz_path << "\n";
    }

    std::cout << "\nBest duration: "
              << TimeSinceServiceStart{best_path.DurationSeconds()}.ToString()
              << "\n";

    std::cout << "Path (" << best_path.steps.size() << " steps):\n";
    for (const Step& step : best_path.steps) {
      std::cout << "  " << state.StopName(step.origin.stop) << " ("
                << step.origin.time.ToString() << ") -> "
                << state.StopName(step.destination.stop) << " ("
                << step.destination.time.ToString() << ")\n";
    }

    {
      std::cout << "\nWhee let's go refining!!!\n";
      ProblemState cur_state = solution.partial_problem;

      int iteration = 0;
      while (true) {
        std::cout << "=== Iteration " << iteration << " ===\n";
        iteration += 1;

        std::optional<TspTourResult> tarel = ComputeTarelLowerBound(cur_state);
        if (!tarel.has_value()) {
          std::cout << "  infeasible!?";
          break;
        }
        std::cout << "  cur path:\n";
        for (int i = 0; i < tarel->tour_edges.size(); ++i) {
          std::cout
              << "    " << cur_state.StopName(tarel->tour_edges[i].origin.stop)
              << " -> "
              << cur_state.StopName(tarel->tour_edges[i].destination.stop)
              << ": "
              << TimeSinceServiceStart{tarel->tour_edges[i].weight}.ToString()
              << "\n";
        }
        std::cout << "  cur lb: "
                  << TimeSinceServiceStart{tarel->optimal_value}.ToString()
                  << "\n";

        if (tarel->optimal_value >= solution.paths[0].path.DurationSeconds()) {
          std::cout << "  solved!\n";
          break;
        }

        RefinementResult best_refinement;
        best_refinement.old_tour_new_weight = 0;
        for (int i = 1; i + 1 < tarel->tour_edges.size(); ++i) {
          const TarelEdge& edge = tarel->tour_edges[i];
          RefinementResult refined = Refine(
              cur_state,
              edge.origin.stop,
              edge.destination.stop,
              tarel->tour_edges
          );
          if (refined.old_tour_new_weight >
              best_refinement.old_tour_new_weight) {
            best_refinement = std::move(refined);
          }
        }
        std::cout << "  best refinement: "
                  << cur_state.StopName(best_refinement.a) << " -> "
                  << cur_state.StopName(best_refinement.b) << "\n";
        std::cout << "  best refinement weight: "
                  << TimeSinceServiceStart{best_refinement.old_tour_new_weight}
                         .ToString()
                  << "\n";

        if (best_refinement.old_tour_new_weight <= tarel->optimal_value) {
          std::cout << "  refinement not an improvement!?\n";
          break;
        }

        cur_state = best_refinement.state;
      }
    }

    // TEMP
    break;

    if (distances.empty()) {
      std::cout << "\nAll required stops are visited.\n";
      break;
    }

    std::cout << "\nRequired stops NOT visited (" << distances.size() << "):\n";
    for (const auto& sd : distances) {
      std::cout << "  " << state.StopName(sd.unvisited_stop)
                << " (distance: " << sd.distance
                << ", nearest: " << state.StopName(sd.nearest_path_stop)
                << ")\n";
    }

    // Add the farthest unvisited stop to leaves for the next iteration.
    StopId farthest = distances.back().unvisited_stop;
    std::cout << "\nAdding farthest stop: " << state.StopName(farthest)
              << "\n\n";
    state.required.VisitGroupStops(farthest, [&](StopId s) {
      required_subset.insert(s);
    });
  }

  return 0;
}
