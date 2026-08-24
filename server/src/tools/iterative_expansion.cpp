#include <CLI/CLI.hpp>
#include <algorithm>
#include <chrono>
#include <climits>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
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
#include "solver/branch_and_bound.h"
#include "solver/data.h"
#include "solver/iterative_expansion_partial_solve.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"
#include "util/trace.h"
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

// One stop of the problem, reported via --solution_json so that a viewer can
// place the solution path on a map without the problem state or the viz
// SQLite.
struct SolutionStop {
  std::string id;  // GTFS stop id, which is what a VizStep refers to
  std::string name;
  double lat;
  double lon;
  // Whether the problem requires visiting this stop, as opposed to it only
  // being somewhere a step can pass through.
  bool required;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SolutionStop, id, name, lat, lon, required)

// One GTFS route+direction a step of the solution path travels on.
struct SolutionRoute {
  std::string id;  // Matches VizStep::route_direction_id
  std::string name;
  std::string color;       // GTFS route_color, without the "#"; may be empty
  std::string text_color;  // GTFS route_text_color; may be empty
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SolutionRoute, id, name, color, text_color)

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

  // Compute the minimal graph on the required stops. We can't use
  // `state.minimal` directly because it may include intermediate non-required
  // stops and we don't want those to participate in the MST.
  StepPathsAdjacencyList minimal = ReduceToMinimalSystemPaths(
      state.minimal,
      state.required.AllFlat(),
      HorizonCoveringAllDepartures(state.minimal)
  );

  std::vector weights(n * n, std::numeric_limits<int>::max());
  weights.reserve(n * n);
  for (const Step& step : minimal.AllMergedSteps()) {
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

// Thrown when --timeout elapses. Checked between outer iterations and, more
// finely, on every branch-and-bound search event.
struct SolveTimeout : std::runtime_error {
  SolveTimeout() : std::runtime_error("solve timed out") {}
};

PartialSolutionPath GreedilyExtendAsMuchAsPossibleWithoutIncreasingDuration(
    const ProblemState& original_problem,
    const PartialSolutionPath& partial_path,
    MinimalPathSetCache& cache
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

    // Every candidate is spliced into the same tour, so its path sets are
    // computed once here rather than once per candidate.
    TourPathSets tour_paths = TourPathSets::Compute(result.subset_tour, cache);

    std::vector<PartialSolutionPath> improved;
    for (StopId new_stop : unvisited) {
      PartialSolution extended = NaivelyExtendPartialSolution(
          result.subset_tour, tour_paths, new_stop, cache
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

// For each required stop not on the path, computes its shortest "distance" to
// the path. Distance from required stop x to path stop p: find the
// min-duration path from x to p in state.minimal, then count how many
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

  // Filter visited stops to exclude boundary.
  std::unordered_set<StopId> path_stops;
  for (StopId p : visited) {
    if (p != state.boundary.start && p != state.boundary.end) {
      path_stops.insert(p);
    }
  }

  std::unordered_map<StopId, StopDistance> rep_to_distance;
  for (StopId x : state.required.AllFlat()) {
    StopId rep = state.required.Representative(x);
    if (visited_reps.contains(rep)) {
      continue;
    }
    // Compute minimal paths from x to all path stops on-demand.
    auto paths_from_x = FindMinimalPathSet(state.minimal, x, path_stops);

    int min_dist = INT_MAX;
    StopId nearest_stop{};
    for (StopId p : path_stops) {
      auto it = paths_from_x.find(p);
      if (it == paths_from_x.end()) {
        continue;
      }
      const Path* shortest = nullptr;
      for (const Path& candidate : it->second) {
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

void WriteRequiredSubsetToml(
    const ProblemState& state,
    const std::string& dir,
    int iteration,
    const std::unordered_set<StopId>& subset
) {
  if (dir.empty()) return;

  std::string path =
      dir + "/required_iteration_" + std::to_string(iteration) + ".toml";
  std::ofstream out(path);

  // Collect GTFS stop IDs and names, sorted by GTFS ID for determinism.
  struct StopEntry {
    std::string gtfs_id;
    std::string name;
  };
  std::vector<StopEntry> entries;
  for (StopId stop : subset) {
    // Skip START and END boundary stops.
    if (!state.stop_infos.contains(stop)) continue;
    const auto& info = state.stop_infos.at(stop);
    entries.push_back({info.gtfs_stop_id.v, info.stop_name});
  }
  std::ranges::sort(entries, {}, &StopEntry::gtfs_id);

  // Figure out which stop groups have members in the subset.
  std::vector<std::vector<StopEntry>> groups;
  for (const auto& group : state.required.Groups()) {
    if (group.size() <= 1) continue;
    std::vector<StopEntry> group_entries;
    for (StopId s : group) {
      if (!subset.contains(s)) continue;
      const auto& info = state.stop_infos.at(s);
      group_entries.push_back({info.gtfs_stop_id.v, info.stop_name});
    }
    if (group_entries.size() <= 1) continue;
    groups.push_back(std::move(group_entries));
  }

  out << "# Iteration " << iteration << " required subset\n";
  out << "# Number of stops: " << entries.size() << "\n\n";
  out << "stop_ids = [\n";
  for (const auto& entry : entries) {
    out << "  \"" << entry.gtfs_id << "\", # " << entry.name << "\n";
  }
  out << "]\n";

  if (!groups.empty()) {
    out << "\nstop_groups = [\n";
    for (const auto& group : groups) {
      out << "  [";
      for (size_t i = 0; i < group.size(); ++i) {
        if (i > 0) out << ", ";
        out << "\"" << group[i].gtfs_id << "\"";
      }
      out << "], # ";
      for (size_t i = 0; i < group.size(); ++i) {
        if (i > 0) out << "; ";
        out << group[i].name;
      }
      out << "\n";
    }
    out << "]\n";
  }
}

// Up to this many required groups, PartialSolveBruteForce beats
// PartialSolveBranchAndBound. Measured on the bart and vtalr instances, where
// brute force is 2-5x faster at 10 groups and 2x slower at 11.
constexpr int kDefaultBruteForceMaxGroups = 10;

int main(int argc, char* argv[]) {
  CLI::App app{"Iterative expansion tool"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to problem state JSON")
      ->required();

  std::string intermediate_output_dir;
  app.add_option(
      "--intermediate_output_dir",
      intermediate_output_dir,
      "Directory to write each iteration's required_iteration_{n}.toml and "
      "problem_state_iteration_{n}.json: the stops it was asked to visit and "
      "the problem it solved, the latter in the format benchmark_bnb reads, "
      "so that one iteration can be re-run on its own"
  );

  std::string solution_json_path;
  app.add_option(
      "--solution_json", solution_json_path, "Path to write solution info JSON"
  );

  int brute_force_max_groups = kDefaultBruteForceMaxGroups;
  app.add_option(
      "--brute_force_max_groups",
      brute_force_max_groups,
      "Solve an iteration by brute force rather than branch and bound when it "
      "has at most this many required groups"
  );

  double timeout_seconds = 0.0;
  app.add_option(
      "--timeout",
      timeout_seconds,
      "Give up after this many seconds (0 = no timeout). Checked between "
      "iterations and on each branch-and-bound search event, so a single "
      "long-running Concorde solve can overshoot it."
  );

  CLI11_PARSE(app, argc, argv);

  // Reported via --solution_json, so that the pipeline can store it alongside
  // the timings it measures itself.
  Trace trace("iterative_expansion");

  std::optional<std::chrono::steady_clock::time_point> deadline;
  if (timeout_seconds > 0.0) {
    deadline = std::chrono::steady_clock::now() +
               std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   std::chrono::duration<double>(timeout_seconds)
               );
  }
  auto check_deadline = [&deadline]() {
    if (deadline && std::chrono::steady_clock::now() > *deadline) {
      throw SolveTimeout{};
    }
  };
  SearchEventCallback on_search_event = nullptr;
  if (deadline) {
    on_search_event = [&check_deadline](const SearchEvent&) {
      check_deadline();
    };
  }

  std::string viz_sqlite_path = viz::VizSqlitePath(input_path);

  // Load problem state.
  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << "\n";
    return 1;
  }

  std::cout << "Loading problem...\n";
  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState state = j.get<ProblemState>();

  std::cout << "Computing MST...\n";
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
  // table, plus the stop and route metadata reported via --solution_json.
  std::unordered_map<int, std::string> trip_to_route;
  // The stops a step of this problem can start or end at: "original" stops are
  // the rest of the feed, which no step of the problem state touches.
  std::vector<SolutionStop> solution_stops;
  std::unordered_map<std::string, SolutionRoute> routes_by_id;
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

    stmt = nullptr;
    sqlite3_prepare_v2(
        db.handle(),
        "SELECT stop_id, stop_name, lat, lon, stop_type FROM stops "
        "WHERE stop_type != 'original' ORDER BY stop_id",
        -1,
        &stmt,
        nullptr
    );
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      auto column_text = [&stmt](int column) {
        return std::string(
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, column))
        );
      };
      solution_stops.push_back({
          .id = column_text(0),
          .name = column_text(1),
          .lat = sqlite3_column_double(stmt, 2),
          .lon = sqlite3_column_double(stmt, 3),
          .required = column_text(4) == "required",
      });
    }
    sqlite3_finalize(stmt);

    stmt = nullptr;
    sqlite3_prepare_v2(
        db.handle(),
        "SELECT route_direction_id, route_name, route_color, route_text_color "
        "FROM routes",
        -1,
        &stmt,
        nullptr
    );
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      auto column_text = [&stmt](int column) {
        return std::string(
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, column))
        );
      };
      SolutionRoute route{
          .id = column_text(0),
          .name = column_text(1),
          .color = column_text(2),
          .text_color = column_text(3),
      };
      routes_by_id[route.id] = route;
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

  // Same, without the synthetic START->first and last->END edges. They are
  // zero-duration, so dropping them keeps the path's duration, and their
  // boundary stop has no GTFS id to place on a map.
  auto ToReportedVizPath = [&ToVizPath](const Path& path) -> VizPath {
    VizPath vp = ToVizPath(path);
    auto is_boundary = [](const VizStep& step) {
      return step.origin_stop_id.empty() || step.destination_stop_id.empty();
    };
    std::erase_if(vp.steps, is_boundary);
    std::erase_if(vp.original_steps, is_boundary);
    return vp;
  };

  if (!intermediate_output_dir.empty()) {
    std::filesystem::create_directories(intermediate_output_dir);
  }

  // Kept across iterations: the tours they extend share most of their stop
  // pairs, so a pair computed in one iteration is usually asked for again.
  MinimalPathSetCache path_cache(state.minimal);

  // Outcome of the loop below, reported via --solution_json.
  std::string status = "solved";
  std::optional<int> optimal_duration_seconds;
  std::optional<VizPath> solution_path;

  // The previous iteration's optimal duration. Each iteration's required
  // subset contains the previous one's, so the optimum is non-decreasing
  // across iterations, and a search whose incumbent UB reaches the previous
  // optimum can stop immediately: that UB is optimal.
  std::optional<int> prev_iteration_optimal;

  try {
    for (int iteration = 0;; iteration++) {
      check_deadline();
      // How many groups the subset is made of, which is what the cost of
      // solving it depends on: a tour visits one stop of each group.
      std::unordered_set<StopId> subset_representatives;
      for (StopId stop : required_subset) {
        subset_representatives.insert(state.required.Representative(stop));
      }
      bool brute_force =
          subset_representatives.size() <=
          static_cast<size_t>(std::max(brute_force_max_groups, 0));

      TraceSpan iteration_span(trace, "iter " + std::to_string(iteration));
      iteration_span.SetMetadata("stops", required_subset.size());
      iteration_span.SetMetadata("solver", brute_force ? "brute" : "bnb");
      WriteRequiredSubsetToml(
          state, intermediate_output_dir, iteration, required_subset
      );
      std::cout << "=== Iteration " << iteration << ": "
                << (brute_force ? "brute force" : "branch and bound") << " on "
                << required_subset.size() << " leaves in "
                << subset_representatives.size() << " groups ===\n";
      ProblemState partial_problem =
          MakePartialProblemState(required_subset, state);
      if (!intermediate_output_dir.empty()) {
        std::string path = intermediate_output_dir +
                           "/problem_state_iteration_" +
                           std::to_string(iteration) + ".json";
        std::ofstream out(path);
        if (!out.is_open()) {
          std::cerr << "Error: could not write " << path << "\n";
          return 1;
        }
        out << nlohmann::json(partial_problem).dump() << "\n";
        std::cout << "Wrote iteration problem state to: " << path << "\n";
      }

      PartialSolution solution =
          brute_force
              ? PartialSolveBruteForce(required_subset, state, check_deadline)
              : PartialSolveBranchAndBound(
                    partial_problem,
                    state,
                    &std::cout,
                    on_search_event,
                    prev_iteration_optimal
                );

      // Choose the path that visits the most required stops.
      auto best_solution_path_it =
          solution.BestPathByRequiredStops(state.required);
      if (best_solution_path_it == solution.paths.end()) {
        std::cout << "No feasible paths found.\n";
        status = "infeasible";
        break;
      }

      PartialSolutionPath best_solution_path = *best_solution_path_it;
      std::cout << "Before greedy improve: "
                << best_solution_path.path.IntermediateStopCount() << "\n";

      best_solution_path =
          GreedilyExtendAsMuchAsPossibleWithoutIncreasingDuration(
              state, best_solution_path, path_cache
          );
      std::cout << "After greedy improve: "
                << best_solution_path.path.IntermediateStopCount() << "\n";

      const Path& best_path = best_solution_path.path;
      prev_iteration_optimal = best_path.DurationSeconds();
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

      std::cout << "\n" << best_path.Debug(state);

      if (distances.empty()) {
        std::cout << "\nAll required stops are visited.\n";
        optimal_duration_seconds = best_path.DurationSeconds();
        solution_path = ToReportedVizPath(best_path);
        break;
      }

      std::cout << "\nRequired stops NOT visited (" << distances.size()
                << "):\n";
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
  } catch (const SolveTimeout&) {
    std::cout << "\nTimed out after " << timeout_seconds << " seconds.\n";
    status = "timeout";
  }

  if (!solution_json_path.empty()) {
    nlohmann::json solution_info;
    solution_info["status"] = status;
    if (optimal_duration_seconds.has_value()) {
      solution_info["optimal_duration_seconds"] = *optimal_duration_seconds;
    }
    if (status == "timeout") {
      solution_info["timeout_seconds"] = timeout_seconds;
    }
    solution_info["stops"] = solution_stops;
    if (solution_path.has_value()) {
      solution_info["solution_path"] = *solution_path;
      // Only the routes the path travels on: the rest of the feed's routes
      // would be dead weight in every stored solution.
      std::set<std::string> used_route_ids;
      for (const VizStep& step : solution_path->steps) {
        if (step.route_direction_id.has_value()) {
          used_route_ids.insert(*step.route_direction_id);
        }
      }
      std::vector<SolutionRoute> used_routes;
      for (const std::string& route_id : used_route_ids) {
        used_routes.push_back(routes_by_id.at(route_id));
      }
      solution_info["routes"] = used_routes;
    }
    solution_info["trace"] = trace.ToJson();
    std::ofstream solution_out(solution_json_path);
    if (!solution_out.is_open()) {
      std::cerr << "Error: could not write " << solution_json_path << "\n";
      return 1;
    }
    solution_out << solution_info.dump(2) << "\n";
    std::cout << "Saved solution info to: " << solution_json_path << "\n";
  }

  return status == "solved" ? 0 : 1;
}
