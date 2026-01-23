#include <algorithm>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

struct BranchEdge {
  StopId a;
  StopId b;

  bool operator==(const BranchEdge& other) const = default;
};

template <>
struct std::hash<BranchEdge> {
  std::size_t operator()(const BranchEdge& e) const noexcept {
    std::size_t h1 = std::hash<int>{}(e.a.v);
    std::size_t h2 = std::hash<int>{}(e.b.v);
    return h1 ^ (h2 << 1);
  }
};

std::string GetTimestampDir() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::localtime(&time_t_now);
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::optional<TspTourResult> DoSolve(const DataGtfsMapping& mapping, const ProblemState& state, const std::string& tour_dir, std::optional<int> ub) {
  std::filesystem::create_directory(tour_dir);
  std::ofstream log(tour_dir + "/log");

  std::unordered_set<StopId> stops_without_origin = state.required_stops;
  std::unordered_set<StopId> stops_without_destination = state.required_stops;
  stops_without_destination.erase(state.boundary.start);
  stops_without_origin.erase(state.boundary.end);

  std::unordered_map<Step, std::string> step_to_last_scheduled_route_desc;
  for (const Path& p : state.completed.AllPaths()) {
    std::string route_desc = "flex";
    for (const Step& s : p.steps) {
      // TODO: Fix the !=-2 badness.
      if (!s.is_flex && s.destination.trip.v != -2) {
        if (mapping.trip_id_to_route_desc.find(s.destination.trip) == mapping.trip_id_to_route_desc.end()) {
          std::cout << s.destination.trip << "\n";
        }
        route_desc = mapping.trip_id_to_route_desc.at(s.destination.trip);
      }
    }
    step_to_last_scheduled_route_desc[p.merged_step] = route_desc;

    stops_without_origin.erase(p.merged_step.origin.stop);
    stops_without_destination.erase(p.merged_step.destination.stop);
  }

  if (stops_without_origin.size() > 0 || stops_without_destination.size() > 0) {
    std::cout << "Stops without origin:\n";
    for (StopId s : stops_without_origin) {
      std::cout << "  " << state.StopName(s) << "\n";
    }
    std::cout << "Stops without destination:\n";
    for (StopId s : stops_without_destination) {
      std::cout << "  " << state.StopName(s) << "\n";
    }
    return std::nullopt;
  }

  std::unordered_map<std::string, StepPartitionId> partition_to_id;
  std::unordered_map<StepPartitionId, std::string> id_to_partition;
  auto GetId = [&](const std::string& partition) -> StepPartitionId {
    auto [it, inserted] = partition_to_id.try_emplace(partition, StepPartitionId{static_cast<int>(partition_to_id.size())});
    if (inserted) {
      id_to_partition[it->second] = it->first;
    }
    return it->second;
  };

  std::vector<TarelEdge> raw_tarel_edges = MakeTarelEdges(
    state.completed,
    std::function<StepPartitionId(Step)>([&](const Step& s) -> StepPartitionId {
      if (s.is_flex) {
        return GetId("flex");
      }
      auto it = step_to_last_scheduled_route_desc.find(s);
      assert(it != step_to_last_scheduled_route_desc.end());
      return GetId(it->second);
    })
  );

  std::vector<TarelEdge> tarel_edges = MergeEquivalentTarelStates(raw_tarel_edges);

  log << "Active partitions:\n";
  std::vector<std::string> active_partitions;
  for (const auto& [_, x] : id_to_partition) {
    active_partitions.push_back(x);
  }
  std::ranges::sort(active_partitions);
  for (const std::string& x : active_partitions) {
    log << "  " << x << "\n";
  }
  log << std::flush;

  TspGraphData graph = MakeTspGraphEdges(tarel_edges, state.boundary);
  std::ofstream tsp_log(tour_dir + "/tsp_log");
  std::optional<TspTourResult> tour_result = SolveTspAndExtractTour(tarel_edges, graph, state.boundary, ub, &tsp_log);
  if (!tour_result.has_value()) {
    log << "No valid TSP tour exists (uses forbidden edges)\n";
    return std::nullopt;
  }
  std::vector<Path> feasible_paths = ComputeMinDurationFeasiblePaths(*tour_result, state);
  if (feasible_paths.size() > 0) {
    log << feasible_paths.size() << " feasible paths with start times:";
    for (const Path& p : feasible_paths) {
      log << " " << p.merged_step.origin.time.ToString();
    }
    log << "\n";
    log << "Duration: " << TimeSinceServiceStart{feasible_paths[0].DurationSeconds()}.ToString() << "\n";
    PrintTarelTourResults(log, *tour_result, state, feasible_paths[0], id_to_partition);
  } else {
    log << "No feasible path?!\n";
  }
  log << std::flush;

  return tour_result;
}

// SolutionState BranchRequire(const SolutionState& state, StopId branch_a, StopId branch_b) {
//   // Make it so that the only incoming edges to branch_b are those from branch_a.
//   std::vector<Step> branched_steps = state.adj.AllSteps();
//   std::erase_if(branched_steps, [&](const Step& s) -> bool {
//     return s.destination.stop == branch_b && s.origin.stop != branch_a;
//   });
//   SolutionState branched_state = state;
//   branched_state.adj = MakeAdjacencyList(branched_steps);
//   branched_state.stops.erase(branch_a);
//   if (branch_a == state.boundary.start) {
//     branched_state.boundary.start = branch_b;
//   }
//   return branched_state;
// }

// SolutionState BranchRequire(const SolutionState& state, StopId branch_a, StopId branch_b) {
//   // Remove `branch_b` from the required stops but make it so that the only
//   // outgoing edges from `branch_a` go to `branch_b`.
//   std::vector<Step> branched_steps = state.adj.AllSteps();
//   std::erase_if(branched_steps, [&](const Step& s) -> bool {
//     StopId actually = branch_a;
//     while (state.actually_ends_up_at.find(actually) != state.actually_ends_up_at.end()) {
//       actually = state.actually_ends_up_at.at(actually);
//     }
//     return s.origin.stop == actually && s.destination.stop != branch_b;
//   });
//   SolutionState branched_state = state;
//   branched_state.adj = MakeAdjacencyList(branched_steps);
//   branched_state.stops.erase(branch_b);
//   if (branch_b == state.boundary.end) {
//     branched_state.boundary.end = branch_a;
//   }
//   branched_state.actually_ends_up_at[branch_a] = branch_b;
//   return branched_state;
// }

ProblemState BranchRequire(const ProblemState& state, StopId branch_a, StopId branch_b) {
  // Remove `branch_b` from the required stops but replace all the steps out of
  // `branch_a` with those combined with the steps out of `branch_b`.
  std::unordered_set<StopId> new_stops = state.required_stops;
  new_stops.erase(branch_b);

  std::unordered_map<StopId, std::string> new_stop_names = state.stop_names;
  new_stop_names[branch_a] = (
    new_stop_names.at(branch_a) + "-> (" +
    new_stop_names.at(branch_b) + ")"
  );

  // First pull the steps out into a few pieces.
  std::vector<Step> a_to_b_steps;
  std::unordered_map<StopId, std::vector<Step>> b_to_star_steps;
  std::unordered_map<StopId, std::vector<Step>> a_to_star_steps;
  std::unordered_map<StopId, std::vector<Step>> star_to_a_steps;
  std::vector<Step> result_steps;
  for (const Step& s : state.minimal.AllSteps()) {
    if (s.origin.stop == branch_a && s.destination.stop == branch_b) {
      a_to_b_steps.push_back(s);
    } else if (s.origin.stop == branch_b) {
      b_to_star_steps[s.destination.stop].push_back(s);
    } else if (s.origin.stop == branch_a) {
      a_to_star_steps[s.destination.stop].push_back(s);
    } else if (s.destination.stop == branch_a) {
      star_to_a_steps[s.origin.stop].push_back(s);
      result_steps.push_back(s);
    } else {
      result_steps.push_back(s);
    }
  }

  // Then build up the combined steps and put them in `other_steps`.
  for (const auto& [star, b_to_star] : b_to_star_steps) {
    for (const Step& s : PairwiseMergedSteps(a_to_b_steps, b_to_star)) {
      result_steps.push_back(s);
    }
  }
  for (const auto& [x, x_to_a] : star_to_a_steps) {
    for (const auto& [y, a_to_y] : a_to_star_steps) {
      for (const Step& s : PairwiseMergedSteps(x_to_a, a_to_y)) {
        result_steps.push_back(s);
      }
    }
  }

  return MakeProblemState(
    MakeAdjacencyList(result_steps),
    state.boundary,
    new_stops,
    new_stop_names
  );
}

ProblemState BranchForbid(const ProblemState& state, StopId branch_a, StopId branch_b) {
  std::vector<Step> branched_steps = state.minimal.AllSteps();
  std::erase_if(branched_steps, [&](const Step& s) -> bool {
    return s.destination.stop == branch_b && s.origin.stop == branch_a;
  });
  return MakeProblemState(
    MakeAdjacencyList(branched_steps),
    state.boundary,
    state.required_stops,
    state.stop_names
  );
}

struct SearchNode {
  int id;
  int lb;
  ProblemState state;
  std::vector<BranchEdge> required_edges;
  int newly_required = 0;
  std::vector<BranchEdge> forbidden_edges;
  int newly_forbidden = 0;
};

struct BranchEdgeDisplay {
  int req_fw = 0;
  int req_bw = 0;
  int forbid_fw = 0;
  int forbid_bw = 0;
};

int main() {
    const std::string gtfs_path = "../data/RG_20260108_all";

    std::cout << "Loading GTFS data from: " << gtfs_path << std::endl;
    GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);

    gtfs_day = GtfsNormalizeStops(gtfs_day);
    StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(
      gtfs_day,
      GetStepsOptions{
        .max_walking_distance_meters=1000.0,
        .walking_speed_ms=1.0,
      }
    );

    std::unordered_set<StopId> bart_stops =
        GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

    std::cout << "Initializing solution state...\n";
    ProblemState initial_state = InitializeProblemState(steps_from_gtfs, bart_stops);

    std::string run_dir = GetTimestampDir();
    std::filesystem::create_directory(run_dir);
    std::cout << "Output directory: " << run_dir << std::endl;

    std::vector<Path> best_feasible_paths;

    auto cmp = [](const std::unique_ptr<SearchNode>& a, const std::unique_ptr<SearchNode>& b) -> bool {
      return a->lb > b->lb;
    };
    std::vector<std::unique_ptr<SearchNode>> q;
    int next_node_id = 0;

    q.push_back(std::make_unique<SearchNode>(next_node_id, 0, initial_state));
    next_node_id += 1;

    while (!q.empty()) {
      std::pop_heap(q.begin(), q.end(), cmp);
      std::unique_ptr<SearchNode> node = std::move(q.back());
      q.pop_back();

      std::cout << "Solving " << node->id
        << ": " << node->required_edges.size() << " required, " << node->forbidden_edges.size() << " forbidden"
        << " (" << q.size() + 1 << " active nodes)\n";

      std::unordered_map<BranchEdge, BranchEdgeDisplay> disp;
      for (const BranchEdge& e : node->required_edges) {
        StopId a = e.a, b = e.b;
        if (a.v < b.v) {
          disp[BranchEdge{a, b}].req_fw += 1;
        } else {
          disp[BranchEdge{b, a}].req_bw += 1;
        }
      }
      for (const BranchEdge& e : node->forbidden_edges) {
        StopId a = e.a, b = e.b;
        if (a.v < b.v) {
          disp[BranchEdge{a, b}].forbid_fw += 1;
        } else {
          disp[BranchEdge{b, a}].forbid_bw += 1;
        }
      }
      std::cout << "  ";
      int disp_i = 0;
      for (const auto& [e, d] : disp) {
        if (disp_i > 0 && disp_i % 3 == 0) {
          std::cout << "\n  ";
        }
        disp_i += 1;
        if (d.req_fw == 1 && d.req_bw == 0 && d.forbid_fw == 0 && d.forbid_bw == 0) {
          std::cout << node->state.StopName(e.a) << "-->" << node->state.StopName(e.b);
        } else if (d.req_fw == 1 && d.req_bw == 0 && d.forbid_fw == 0 && d.forbid_bw == 1) {
          std::cout << node->state.StopName(e.a) << "!->" << node->state.StopName(e.b);
        } else if (d.req_fw == 0 && d.req_bw == 1 && d.forbid_fw == 0 && d.forbid_bw == 0) {
          std::cout << node->state.StopName(e.a) << "<--" << node->state.StopName(e.b);
        } else if (d.req_fw == 0 && d.req_bw == 1 && d.forbid_fw == 1 && d.forbid_bw == 0) {
          std::cout << node->state.StopName(e.a) << "<-!" << node->state.StopName(e.b);
        } else if (d.req_fw == 0 && d.req_bw == 0 && d.forbid_fw == 1 && d.forbid_bw == 1) {
          std::cout << node->state.StopName(e.a) << "!-!" << node->state.StopName(e.b);
        } else if (d.req_fw == 1 && d.req_bw == 1 && d.forbid_fw == 0 && d.forbid_bw == 0) {
          std::cout << node->state.StopName(e.a) << "<->" << node->state.StopName(e.b);
        } else {
          std::cout << node->state.StopName(e.a) << "???" << node->state.StopName(e.b) << "(" << d.req_fw << " " << d.req_bw << " " << d.forbid_fw << " " << d.forbid_bw << ")";
        }
        std::cout << "   ";
      }
      std::cout << "\n";
      if (node->newly_required + node->newly_forbidden > 0) {
        for (int i = node->required_edges.size() - node->newly_required; i < node->required_edges.size(); ++i) {
          const auto& e = node->required_edges[i];
          std::cout << " [latest require] " << node->state.StopName(e.a) << "->" << node->state.StopName(e.b) << "\n";
        }
        for (int i = node->forbidden_edges.size() - node->newly_forbidden; i < node->forbidden_edges.size(); ++i) {
          const auto& e = node->forbidden_edges[i];
          std::cout << " [latest forbid] " << node->state.StopName(e.a) << "->" << node->state.StopName(e.b) << "\n";
        }
      }


      std::optional<TspTourResult> result = DoSolve(
        steps_from_gtfs.mapping,
        node->state,
        run_dir + "/node" + std::to_string(node->id),
        best_feasible_paths.size() > 0 ? std::make_optional(best_feasible_paths[0].DurationSeconds() + 1) : std::nullopt
      );
      if (!result.has_value()) {
        std::cout << "Solved " << node->id << ": Infeasible\n\n";
        continue;
      }

      std::cout << "Solved " << node->id << ": "
        << TimeSinceServiceStart{node->lb}.ToString() << " -> "
        << TimeSinceServiceStart{result->optimal_value}.ToString() << "\n";

      if (best_feasible_paths.size() > 0 && result->optimal_value >= best_feasible_paths[0].DurationSeconds()) {
        std::cout << "Pruned " << node->id << "\n\n";
        continue;
      }
      std::cout << "\n";

      assert(result->optimal_value >= node->lb);

      std::vector<Step> primitive_steps;
      for (const TarelEdge& e : result->tour_edges) {
        const auto& paths = node->state.completed.PathsBetween(e.origin.stop, e.destination.stop);
        assert(paths.size() > 0);
        Path best = paths[0];
        for (const Path& p : paths) {
          if (p.DurationSeconds() < best.DurationSeconds()) {
            best = p;
          }
        }
        for (const Step& s : best.steps) {
          primitive_steps.push_back(s);
        }
      }

      // // Check that the primitive steps visit all the stops.
      // std::unordered_set<StopId> unvisited_stops = initial_state.stops;
      // for (const Step& s : primitive_steps) {
      //   unvisited_stops.erase(s.origin.stop);
      //   unvisited_stops.erase(s.destination.stop);
      // }
      // if (unvisited_stops.size() > 0) {
      //   std::cout << "Unvisited stops!\n";
      //   for (StopId s : unvisited_stops) {
      //     std::cout << "  " << initial_state.StopName(s) << "\n";
      //   }
      //   std::cout << "Full route:\n";
      //   for (const TarelEdge& e : result->tour_edges) {
      //     std::cout << "  " << initial_state.StopName(e.origin.stop) << "\n";
      //     const auto& paths = completed.PathsBetween(e.origin.stop, e.destination.stop);
      //     assert(paths.size() > 0);
      //     Path best = paths[0];
      //     for (const Path& p : paths) {
      //       if (p.DurationSeconds() < best.DurationSeconds()) {
      //         best = p;
      //       }
      //     }
      //     for (int steps_i = 0; steps_i < best.steps.size(); ++steps_i) {
      //       const Step& s = best.steps[steps_i];
      //       std::cout << "    " << initial_state.StopName(s.origin.stop) << " -> " << initial_state.StopName(s.destination.stop) << "\n";
      //     }
      //     std::cout << "  " << initial_state.StopName(e.destination.stop) << "\n";
      //   }
      //   assert(false);
      // }

      std::vector<Path> feasible_paths = ComputeMinDurationFeasiblePaths(*result, node->state);
      if (
        feasible_paths.size() > 0 &&
        (best_feasible_paths.size() == 0 || feasible_paths[0].DurationSeconds() < best_feasible_paths[0].DurationSeconds())
      ) {
        std::cout << "Found new UB: " << TimeSinceServiceStart{feasible_paths[0].DurationSeconds()}.ToString() << "\n";
        best_feasible_paths = feasible_paths;
      }

      // // The route traced by `primitive_steps` except skipping any stops that
      // // have been elmininated during branching.
      // std::vector<StopId> reduced_primitive_route;
      // {
      //   StopId cur = node->state.boundary.start;
      //   reduced_primitive_route.push_back(node->state.boundary.start);
      //   for (const Step& s : primitive_steps) {
      //     assert(s.origin.stop == cur);
      //     if (node->state.stops.contains(s.destination.stop)) {
      //       reduced_primitive_route.push_back(s.destination.stop);
      //     }
      //     cur = s.destination.stop;
      //   }
      // }
      // assert(reduced_primitive_route[0] == node->state.boundary.start);
      // assert(reduced_primitive_route.back() == node->state.boundary.end);

      // if (reduced_primitive_route.size() <= 3) {
      //   std::cout << "Not branching " << node->id << ": reduced primitive route not long enough\n";
      //   continue;
      // }

      // // TODO: Figure out if we can branch on START and END edges.
      // int branch_index = (rand() % (reduced_primitive_route.size() - 3)) + 1;
      // BranchEdge branch_edge{reduced_primitive_route[branch_index], reduced_primitive_route[branch_index + 1]};
      // BranchEdge branch_edge_bw{branch_edge.b, branch_edge.a};

      // Branch on a random primitive step.
      if (primitive_steps.size() <= 2) {
        std::cout << "Not branching " << node->id << ": not enough primitive steps\n";
        continue;
      }
      Step branch_step = primitive_steps[((rand() + 1) % (primitive_steps.size() - 2)) + 1];
      BranchEdge branch_edge{branch_step.origin.stop, branch_step.destination.stop};
      BranchEdge branch_edge_bw{branch_edge.b, branch_edge.a};

      {
        std::unique_ptr<SearchNode> node_require_fw = std::make_unique<SearchNode>(
          next_node_id,
          result->optimal_value,
          BranchRequire(node->state, branch_edge.a, branch_edge.b),
          node->required_edges,
          0,
          node->forbidden_edges,
          0
        );
        next_node_id += 1;
        node_require_fw->required_edges.push_back(branch_edge);
        node_require_fw->newly_required = 1;
        q.push_back(std::move(node_require_fw));
        std::push_heap(q.begin(), q.end(), cmp);
      }

      {
        std::unique_ptr<SearchNode> node_forbid_fw_require_bw = std::make_unique<SearchNode>(
          next_node_id,
          result->optimal_value,
          BranchRequire(BranchForbid(node->state, branch_edge.a, branch_edge.b), branch_edge.b, branch_edge.a),
          node->required_edges,
          0,
          node->forbidden_edges,
          0
        );
        next_node_id += 1;
        node_forbid_fw_require_bw->forbidden_edges.push_back(branch_edge);
        node_forbid_fw_require_bw->newly_forbidden = 1;
        node_forbid_fw_require_bw->required_edges.push_back(branch_edge_bw);
        node_forbid_fw_require_bw->newly_required = 1;
        q.push_back(std::move(node_forbid_fw_require_bw));
        std::push_heap(q.begin(), q.end(), cmp);
      }

      {
        std::unique_ptr<SearchNode> node_forbid_fw_bw = std::make_unique<SearchNode>(
          next_node_id,
          result->optimal_value,
          BranchForbid(BranchForbid(node->state, branch_edge.a, branch_edge.b), branch_edge.b, branch_edge.a),
          node->required_edges,
          0,
          node->forbidden_edges,
          0
        );
        next_node_id += 1;
        node_forbid_fw_bw->forbidden_edges.push_back(branch_edge);
        node_forbid_fw_bw->forbidden_edges.push_back(branch_edge_bw);
        node_forbid_fw_bw->newly_forbidden = 2;
        q.push_back(std::move(node_forbid_fw_bw));
        std::push_heap(q.begin(), q.end(), cmp);
      }
    }

    return 0;

    ProblemState state = initial_state;

    std::unordered_map<Step, TripId> last_non_flex_tid;
    for (const Path& p : state.completed.AllPaths()) {
      TripId tid = TripId::NOOP;
      for (const Step& s : p.steps) {
        if (!s.is_flex) {
          tid = s.destination.trip;
        }
      }
      last_non_flex_tid[p.merged_step] = tid;
    }

    // BEGIN: Attempt at bound refinement.
    // {
    //   std::unordered_map<Step, std::string> step_to_tours;

    //   for (int tour_idx = 0; tour_idx < 100; ++tour_idx) {
    //     std::string tour_dir = run_dir + "/tour" + std::to_string(tour_idx);
    //     std::filesystem::create_directory(tour_dir);
    //     std::string tarel_summary_dir = tour_dir + "/tarel_summary";
    //     std::filesystem::create_directory(tarel_summary_dir);
    //     std::ofstream log(tour_dir + "/log");

    //     std::unordered_map<std::string, StepPartitionId> partition_to_id;
    //     std::unordered_map<StepPartitionId, std::string> id_to_partition;
    //     auto GetId = [&](const std::string& partition) -> StepPartitionId {
    //       auto [it, inserted] = partition_to_id.try_emplace(partition, StepPartitionId{static_cast<int>(partition_to_id.size())});
    //       if (inserted) {
    //         id_to_partition[it->second] = it->first;
    //       }
    //       return it->second;
    //     };

    //     std::vector<TarelEdge> raw_tarel_edges = MakeTarelEdges(
    //       completed,
    //       std::function<StepPartitionId(Step)>([&](const Step& s) -> StepPartitionId {
    //         return GetId((s.is_flex ? "flex" : state.dest_trip_id_to_partition[s.destination.trip]) + step_to_tours[s]);
    //       })
    //     );
    //     std::vector<TarelEdge> tarel_edges = MergeEquivalentTarelStates(raw_tarel_edges);

    //     WriteTarelSummary(state, tarel_summary_dir, tarel_edges, id_to_partition);

    //     log << "Active partitions:\n";
    //     std::vector<std::string> active_partitions;
    //     for (const auto& [_, x] : id_to_partition) {
    //       active_partitions.push_back(x);
    //     }
    //     std::ranges::sort(active_partitions);
    //     for (const std::string& x : active_partitions) {
    //       log << "  " << x << "\n";
    //     }
    //     log << std::flush;

    //     TspGraphData graph = MakeTspGraphEdges(tarel_edges, state.boundary);
    //     std::ofstream tsp_log(tour_dir + "/tsp_log");
    //     std::optional<TspTourResult> tour_result = SolveTspAndExtractTour(tarel_edges, graph, state.boundary, std::nullopt, &tsp_log);
    //     if (!tour_result.has_value()) {
    //       log << "No valid TSP tour exists (uses forbidden edges)\n";
    //       std::cout << "Tour " << tour_idx << ": No valid tour\n";
    //       continue;
    //     }
    //     std::cout << "Tour " << tour_idx << " LB: " << TimeSinceServiceStart{tour_result->optimal_value}.ToString() << "\n";
    //     std::vector<Path> feasible_paths = ComputeMinDurationFeasiblePaths(*tour_result, state, completed);
    //     if (feasible_paths.size() > 0) {
    //       log << feasible_paths.size() << " feasible paths with start times:";
    //       for (const Path& p : feasible_paths) {
    //         log << " " << p.merged_step.origin.time.ToString();
    //       }
    //       log << "\n";
    //       log << "Duration: " << TimeSinceServiceStart{feasible_paths[0].DurationSeconds()}.ToString() << "\n";
    //       PrintTarelTourResults(log, *tour_result, state, feasible_paths[0], id_to_partition, completed);
    //     } else {
    //       log << "No feasible path?!\n";
    //     }
    //     log << std::flush;

    //     // Ok so we are going to print out "runs" of achievable Tarel weights starting from any point along the tour.
    //     // What does this even mean?
    //     for (int run_start_idx = 0; run_start_idx < tour_result->tour_edges.size() - 1; ++run_start_idx) {
    //       StopId run_start_stop = tour_result->tour_edges[run_start_idx].origin.stop;
    //       std::vector<Step> feasible_paths = {ZeroEdge(run_start_stop, run_start_stop)};
    //       for (int edge_idx = run_start_idx; edge_idx < tour_result->tour_edges.size(); ++edge_idx) {
    //         const TarelEdge edge = tour_result->tour_edges[edge_idx];
    //         feasible_paths = PairwiseMergedSteps(feasible_paths, edge.steps);
    //         if (feasible_paths.size() == 0) {
    //           log << "Run ended: " << state.StopName(run_start_stop) << " -> " << state.StopName(edge.destination.stop) << "\n";
    //           break;
    //         }
    //       }
    //       if (feasible_paths.size() > 0) {
    //         log << "Run completed: " << state.StopName(run_start_stop) << " -> END\n";
    //         break;
    //       }
    //     }
    //     log << std::flush;

    //     std::map<std::pair<StopId, TripId>, int> tour_dest_and_tids;
    //     for (const TarelEdge& e : tour_result->tour_edges) {
    //       for (const Step& s : e.steps) {
    //         if (last_non_flex_tid[s] != TripId::NOOP) {
    //           tour_dest_and_tids[std::make_pair(e.destination.stop, last_non_flex_tid[s])] = 1;
    //         }
    //       }
    //     }

    //     // std::unordered_set<TripId> tour_tids_feas;
    //     // for (const Path& p : feasible_paths) {
    //     //   for (const Step& s : p.steps) {
    //     //     tour_tids_feas.insert(last_non_flex_tid[s]);
    //     //   }
    //     // }

    //     const std::string suffix = "-R" + std::to_string(tour_idx);
    //     for (const Path& p : completed.AllPaths()) {
    //       if (
    //         tour_dest_and_tids[std::make_pair(p.merged_step.destination.stop, last_non_flex_tid[p.merged_step])] > 0 &&
    //         !step_to_tours[p.merged_step].ends_with(suffix)
    //       ) {
    //         step_to_tours[p.merged_step] += suffix;
    //       }
    //     }

    //     // const std::string suffix_feas = "-F" + std::to_string(tour_idx);
    //     // for (const Path& p : completed.AllPaths()) {
    //     //   if (
    //     //     last_non_flex_tid[p.merged_step] != TripId::NOOP &&
    //     //     tour_tids_feas.contains(last_non_flex_tid[p.merged_step]) &&
    //     //     !step_to_tours[p.merged_step].ends_with(suffix_feas)
    //     //   ) {
    //     //     step_to_tours[p.merged_step] += suffix_feas;
    //     //   }
    //     // }

    //     // std::unordered_set<Step> tour_steps;
    //     // for (const TarelEdge& e : tour_result.tour_edges) {
    //     //   for (const Step& s : e.steps) {
    //     //     tour_steps.insert(s);
    //     //   }
    //     // }

    //     // std::unordered_set<TripId> last_scheduled_trip_ids_in_tour_steps;
    //     // for (const Path& p : completed.AllPaths()) {
    //     //   TripId tid{-1};
    //     //   if (tour_steps.contains(p.merged_step)) {
    //     //     for (const Step& s : p.steps) {
    //     //       if (!s.is_flex) {
    //     //         tid = s.destination.trip;
    //     //       }
    //     //     }
    //     //   }
    //     //   if (tid.v != -1) {
    //     //     last_scheduled_trip_ids_in_tour_steps.insert(tid);
    //     //   }
    //     // }

    //     // const std::string suffix = "-T" + std::to_string(tour_idx);
    //     // for (const Path& p : completed.AllPaths()) {
    //     //   TripId tid{-1};
    //     //   for (const Step& s : p.steps) {
    //     //     if (!s.is_flex) {
    //     //       tid = s.destination.trip;
    //     //     }
    //     //   }
    //     //   if (tid.v != -1) {
    //     //     if (last_scheduled_trip_ids_in_tour_steps.contains(tid) && !step_to_tours[p.merged_step].ends_with(suffix)) {
    //     //       step_to_tours[p.merged_step] += suffix;
    //     //     }
    //     //   }
    //     // }

    //     // int num_steps_partitioned = 0;
    //     // const std::string suffix = "-T" + std::to_string(tour_idx);
    //     // for (const TarelEdge& e : tour_result.tour_edges) {
    //     //   for (const Step& s : e.steps) {
    //     //     if (!step_to_tours[s].ends_with(suffix)) {
    //     //       step_to_tours[s] += suffix;
    //     //       num_steps_partitioned += 1;
    //     //     }
    //     //   }
    //     // }
    //     // log << "Num steps partitioned: " << num_steps_partitioned << "\n";
    //     // log << std::flush;
    //   }
    // }

    return 0;

    // BEGIN: "Line"-based partition.
    // {
    //   std::unordered_map<std::string, StepPartitionId> partition_to_id;
    //   std::unordered_map<StepPartitionId, std::string> id_to_partition;
    //   for (const auto& [_, v] : state.dest_trip_id_to_partition) {
    //     auto [it, inserted] = partition_to_id.try_emplace(v, StepPartitionId{static_cast<int>(partition_to_id.size())});
    //     if (inserted) {
    //       id_to_partition[it->second] = it->first;
    //     }
    //   }

    //   std::vector<TarelEdge> raw_tarel_edges = MakeTarelEdges(
    //     completed,
    //     std::function<StepPartitionId(Step)>([&](const Step& s) -> StepPartitionId {
    //       if (s.is_flex) {
    //         return partition_to_id.at("flex");
    //       }
    //       return partition_to_id.at(state.dest_trip_id_to_partition[s.destination.trip]);
    //     })
    //   );
    //   std::vector<TarelEdge> tarel_edges = MergeEquivalentTarelStates(raw_tarel_edges);

    //   TspGraphData graph = MakeTspGraphEdges(tarel_edges, state.boundary);
    //   std::optional<TspTourResult> tour_result = SolveTspAndExtractTour(tarel_edges, graph, state.boundary);
    //   if (!tour_result.has_value()) {
    //     std::cout << "No valid TSP tour exists (uses forbidden edges)\n";
    //   } else {
    //     std::vector<Path> feasible_paths = ComputeMinDurationFeasiblePaths(*tour_result, state, completed);
    //     if (feasible_paths.size() > 0) {
    //       std::cout << feasible_paths.size() << " feasible paths with start times:";
    //       for (const Path& p : feasible_paths) {
    //         std::cout << " " << p.merged_step.origin.time.ToString();
    //       }
    //       std::cout << "\n";
    //       std::cout << "Duration: " << TimeSinceServiceStart{feasible_paths[0].DurationSeconds()}.ToString() << "\n";
    //       PrintTarelTourResults(std::cout, *tour_result, state, feasible_paths[0], id_to_partition, completed);
    //     } else {
    //       std::cout << "No feasible path?!\n";
    //     }
    //   }
    // }

    // TODO: Think about whether it's possible for there to be a situation where
    // merging multiple times makes progress each time.
    // ... it seems like merging states could make it be so that some states who
    // previously had distinct dest states could now have the same dest states.
    // But:
    // - Maybe there's a reason why anything that ends up with same dest states must have started with same dest states anyways.
    // - Or not quite, but where there has to be a very unlikely coincidence of unrelated weights for dest states to get merged in such a way.
    // auto tarel_es_3 = MergeEquivalentTarelStates(tarel_es_2, state, tarel_result.state_descriptions);

    return 0;
}
