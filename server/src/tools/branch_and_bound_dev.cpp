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
#include <queue>
#include <random>
#include <sstream>
#include <string>
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

std::optional<TspTourResult> DoSolve(const SolutionState& state, const std::string& tour_dir) {
  std::filesystem::create_directory(tour_dir);
  std::ofstream log(tour_dir + "/log");

  StepPathsAdjacencyList completed =
    ReduceToMinimalSystemPaths(state.adj, state.stops, /*keep_through_other_destination=*/true);

  // Add END -> START edge.
  // It's safe to push a new group without checking for an existing group with
  // `start` dest because no edges (other than this one we're adding right
  // now) go into `start`.
  assert(completed.adjacent[state.boundary.end].size() == 0);
  completed.adjacent[state.boundary.end].push_back({ZeroPath(state.boundary.end, state.boundary.start)});

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
    completed,
    std::function<StepPartitionId(Step)>([&](const Step& s) -> StepPartitionId {
      return GetId((s.is_flex ? "flex" : state.dest_trip_id_to_partition.at(s.destination_trip)));
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
  std::optional<TspTourResult> tour_result = SolveTspAndExtractTour(tarel_edges, graph, state.boundary, &tsp_log);
  if (!tour_result.has_value()) {
    log << "No valid TSP tour exists (uses forbidden edges)\n";
    return std::nullopt;
  }
  std::vector<Path> feasible_paths = ComputeMinDurationFeasiblePaths(*tour_result, state, completed);
  if (feasible_paths.size() > 0) {
    log << feasible_paths.size() << " feasible paths with start times:";
    for (const Path& p : feasible_paths) {
      log << " " << p.merged_step.origin_time.ToString();
    }
    log << "\n";
    log << "Duration: " << TimeSinceServiceStart{feasible_paths[0].DurationSeconds()}.ToString() << "\n";
    PrintTarelTourResults(log, *tour_result, state, feasible_paths[0], id_to_partition, completed);
  } else {
    log << "No feasible path?!\n";
  }
  log << std::flush;

  return tour_result;
}

SolutionState BranchRequire(const SolutionState& state, StopId branch_a, StopId branch_b) {
  // Make it so that the only incoming edges to branch_b are those from branch_a.
  std::vector<Step> branched_steps = state.adj.AllSteps();
  std::erase_if(branched_steps, [&](const Step& s) -> bool {
    return s.destination_stop == branch_b && s.origin_stop != branch_a;
  });
  SolutionState branched_state = state;
  branched_state.adj = MakeAdjacencyList(branched_steps);
  branched_state.stops.erase(branch_a);
  if (branch_a == state.boundary.start) {
    branched_state.boundary.start = branch_b;
  }
  return branched_state;
}

SolutionState BranchForbid(const SolutionState& state, StopId branch_a, StopId branch_b) {
  std::vector<Step> branched_steps = state.adj.AllSteps();
  std::erase_if(branched_steps, [&](const Step& s) -> bool {
    return s.destination_stop == branch_b && s.origin_stop == branch_a;
  });
  SolutionState branched_state = state;
  branched_state.adj = MakeAdjacencyList(branched_steps);
  return branched_state;
}

struct SearchNode {
  int id;
  int lb;
  SolutionState state;
  std::unordered_set<BranchEdge> required_edges;
  std::unordered_set<BranchEdge> forbidden_edges;
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
    SolutionState initial_state = InitializeSolutionState(steps_from_gtfs, bart_stops);

    std::string run_dir = GetTimestampDir();
    std::filesystem::create_directory(run_dir);
    std::cout << "Output directory: " << run_dir << std::endl;

    int searched = 0;
    // auto cmp = [](const std::pair<TspTourResult, SolutionState>& a,
    //               const std::pair<TspTourResult, SolutionState>& b) {
    //   return a.first.optimal_value > b.first.optimal_value;  // min-heap by optimal_value
    // };
    // std::priority_queue<std::pair<TspTourResult, SolutionState>,
    //                     std::vector<std::pair<TspTourResult, SolutionState>>,
    //                     decltype(cmp)> q(cmp);

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

      std::cout << "Searching " << node->id << ": " << TimeSinceServiceStart{node->lb}.ToString() << "\n";
      std::optional<TspTourResult> result = DoSolve(node->state, run_dir + "/node" + std::to_string(node->id));
      if (!result.has_value()) {
        std::cout << node->id << ": infeasible\n";
        continue;
      }

      std::vector<Step> primitive_steps;
      StepPathsAdjacencyList completed =
        ReduceToMinimalSystemPaths(node->state.adj, node->state.stops, /*keep_through_other_destination=*/true);
      for (const TarelEdge& e : result->tour_edges) {
        const auto& paths = completed.PathsBetween(e.origin.stop, e.destination.stop);
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

      const Step& branch_step = primitive_steps[rand() % primitive_steps.size()];
      BranchEdge branch_edge{branch_step.origin_stop, branch_step.destination_stop};
      BranchEdge branch_edge_bw{branch_edge.b, branch_edge.a};

      {
        std::unique_ptr<SearchNode> node_require_fw = std::make_unique<SearchNode>(
          next_node_id,
          result->optimal_value,
          BranchRequire(node->state, branch_edge.a, branch_edge.b),
          node->required_edges,
          node->forbidden_edges
        );
        next_node_id += 1;
        node_require_fw->required_edges.insert(branch_edge);
        q.push_back(std::move(node_require_fw));
        std::push_heap(q.begin(), q.end(), cmp);
      }

      {
        std::unique_ptr<SearchNode> node_forbid_fw_require_bw = std::make_unique<SearchNode>(
          next_node_id,
          result->optimal_value,
          BranchRequire(BranchForbid(node->state, branch_edge.a, branch_edge.b), branch_edge.b, branch_edge.a),
          node->required_edges,
          node->forbidden_edges
        );
        next_node_id += 1;
        node_forbid_fw_require_bw->forbidden_edges.insert(branch_edge);
        node_forbid_fw_require_bw->required_edges.insert(branch_edge_bw);
        q.push_back(std::move(node_forbid_fw_require_bw));
        std::push_heap(q.begin(), q.end(), cmp);
      }

      {
        std::unique_ptr<SearchNode> node_forbid_fw_bw = std::make_unique<SearchNode>(
          next_node_id,
          result->optimal_value,
          BranchForbid(BranchForbid(node->state, branch_edge.a, branch_edge.b), branch_edge.b, branch_edge.a),
          node->required_edges,
          node->forbidden_edges
        );
        next_node_id += 1;
        node_forbid_fw_bw->forbidden_edges.insert(branch_edge);
        node_forbid_fw_bw->forbidden_edges.insert(branch_edge_bw);
        q.push_back(std::move(node_forbid_fw_bw));
        std::push_heap(q.begin(), q.end(), cmp);
      }
    }

    return 0;

    SolutionState state = initial_state;
    StepPathsAdjacencyList completed =
      ReduceToMinimalSystemPaths(state.adj, state.stops, /*keep_through_other_destination=*/true);

    // Add END -> START edge.
    // It's safe to push a new group without checking for an existing group with
    // `start` dest because no edges (other than this one we're adding right
    // now) go into `start`.
    assert(completed.adjacent[state.boundary.end].size() == 0);
    completed.adjacent[state.boundary.end].push_back({ZeroPath(state.boundary.end, state.boundary.start)});

    std::unordered_map<Step, TripId> last_non_flex_tid;
    for (const Path& p : completed.AllPaths()) {
      TripId tid = TripId::NOOP;
      for (const Step& s : p.steps) {
        if (!s.is_flex) {
          tid = s.destination_trip;
        }
      }
      last_non_flex_tid[p.merged_step] = tid;
    }

    // BEGIN: Attempt at bound refinement.
    {
      std::unordered_map<Step, std::string> step_to_tours;

      for (int tour_idx = 0; tour_idx < 100; ++tour_idx) {
        std::string tour_dir = run_dir + "/tour" + std::to_string(tour_idx);
        std::filesystem::create_directory(tour_dir);
        std::string tarel_summary_dir = tour_dir + "/tarel_summary";
        std::filesystem::create_directory(tarel_summary_dir);
        std::ofstream log(tour_dir + "/log");

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
          completed,
          std::function<StepPartitionId(Step)>([&](const Step& s) -> StepPartitionId {
            return GetId((s.is_flex ? "flex" : state.dest_trip_id_to_partition[s.destination_trip]) + step_to_tours[s]);
          })
        );
        std::vector<TarelEdge> tarel_edges = MergeEquivalentTarelStates(raw_tarel_edges);

        WriteTarelSummary(state, tarel_summary_dir, tarel_edges, id_to_partition);

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
        std::optional<TspTourResult> tour_result = SolveTspAndExtractTour(tarel_edges, graph, state.boundary, &tsp_log);
        if (!tour_result.has_value()) {
          log << "No valid TSP tour exists (uses forbidden edges)\n";
          std::cout << "Tour " << tour_idx << ": No valid tour\n";
          continue;
        }
        std::cout << "Tour " << tour_idx << " LB: " << TimeSinceServiceStart{tour_result->optimal_value}.ToString() << "\n";
        std::vector<Path> feasible_paths = ComputeMinDurationFeasiblePaths(*tour_result, state, completed);
        if (feasible_paths.size() > 0) {
          log << feasible_paths.size() << " feasible paths with start times:";
          for (const Path& p : feasible_paths) {
            log << " " << p.merged_step.origin_time.ToString();
          }
          log << "\n";
          log << "Duration: " << TimeSinceServiceStart{feasible_paths[0].DurationSeconds()}.ToString() << "\n";
          PrintTarelTourResults(log, *tour_result, state, feasible_paths[0], id_to_partition, completed);
        } else {
          log << "No feasible path?!\n";
        }
        log << std::flush;

        // Ok so we are going to print out "runs" of achievable Tarel weights starting from any point along the tour.
        // What does this even mean?
        for (int run_start_idx = 0; run_start_idx < tour_result->tour_edges.size() - 1; ++run_start_idx) {
          StopId run_start_stop = tour_result->tour_edges[run_start_idx].origin.stop;
          std::vector<Step> feasible_paths = {ZeroEdge(run_start_stop, run_start_stop)};
          for (int edge_idx = run_start_idx; edge_idx < tour_result->tour_edges.size(); ++edge_idx) {
            const TarelEdge edge = tour_result->tour_edges[edge_idx];
            feasible_paths = PairwiseMergedSteps(feasible_paths, edge.steps);
            if (feasible_paths.size() == 0) {
              log << "Run ended: " << state.StopName(run_start_stop) << " -> " << state.StopName(edge.destination.stop) << "\n";
              break;
            }
          }
          if (feasible_paths.size() > 0) {
            log << "Run completed: " << state.StopName(run_start_stop) << " -> END\n";
            break;
          }
        }
        log << std::flush;

        std::map<std::pair<StopId, TripId>, int> tour_dest_and_tids;
        for (const TarelEdge& e : tour_result->tour_edges) {
          for (const Step& s : e.steps) {
            if (last_non_flex_tid[s] != TripId::NOOP) {
              tour_dest_and_tids[std::make_pair(e.destination.stop, last_non_flex_tid[s])] = 1;
            }
          }
        }

        // std::unordered_set<TripId> tour_tids_feas;
        // for (const Path& p : feasible_paths) {
        //   for (const Step& s : p.steps) {
        //     tour_tids_feas.insert(last_non_flex_tid[s]);
        //   }
        // }

        const std::string suffix = "-R" + std::to_string(tour_idx);
        for (const Path& p : completed.AllPaths()) {
          if (
            tour_dest_and_tids[std::make_pair(p.merged_step.destination_stop, last_non_flex_tid[p.merged_step])] > 0 &&
            !step_to_tours[p.merged_step].ends_with(suffix)
          ) {
            step_to_tours[p.merged_step] += suffix;
          }
        }

        // const std::string suffix_feas = "-F" + std::to_string(tour_idx);
        // for (const Path& p : completed.AllPaths()) {
        //   if (
        //     last_non_flex_tid[p.merged_step] != TripId::NOOP &&
        //     tour_tids_feas.contains(last_non_flex_tid[p.merged_step]) &&
        //     !step_to_tours[p.merged_step].ends_with(suffix_feas)
        //   ) {
        //     step_to_tours[p.merged_step] += suffix_feas;
        //   }
        // }

        // std::unordered_set<Step> tour_steps;
        // for (const TarelEdge& e : tour_result.tour_edges) {
        //   for (const Step& s : e.steps) {
        //     tour_steps.insert(s);
        //   }
        // }

        // std::unordered_set<TripId> last_scheduled_trip_ids_in_tour_steps;
        // for (const Path& p : completed.AllPaths()) {
        //   TripId tid{-1};
        //   if (tour_steps.contains(p.merged_step)) {
        //     for (const Step& s : p.steps) {
        //       if (!s.is_flex) {
        //         tid = s.destination_trip;
        //       }
        //     }
        //   }
        //   if (tid.v != -1) {
        //     last_scheduled_trip_ids_in_tour_steps.insert(tid);
        //   }
        // }

        // const std::string suffix = "-T" + std::to_string(tour_idx);
        // for (const Path& p : completed.AllPaths()) {
        //   TripId tid{-1};
        //   for (const Step& s : p.steps) {
        //     if (!s.is_flex) {
        //       tid = s.destination_trip;
        //     }
        //   }
        //   if (tid.v != -1) {
        //     if (last_scheduled_trip_ids_in_tour_steps.contains(tid) && !step_to_tours[p.merged_step].ends_with(suffix)) {
        //       step_to_tours[p.merged_step] += suffix;
        //     }
        //   }
        // }

        // int num_steps_partitioned = 0;
        // const std::string suffix = "-T" + std::to_string(tour_idx);
        // for (const TarelEdge& e : tour_result.tour_edges) {
        //   for (const Step& s : e.steps) {
        //     if (!step_to_tours[s].ends_with(suffix)) {
        //       step_to_tours[s] += suffix;
        //       num_steps_partitioned += 1;
        //     }
        //   }
        // }
        // log << "Num steps partitioned: " << num_steps_partitioned << "\n";
        // log << std::flush;
      }
    }

    return 0;

    // BEGIN: "Line"-based partition.
    {
      std::unordered_map<std::string, StepPartitionId> partition_to_id;
      std::unordered_map<StepPartitionId, std::string> id_to_partition;
      for (const auto& [_, v] : state.dest_trip_id_to_partition) {
        auto [it, inserted] = partition_to_id.try_emplace(v, StepPartitionId{static_cast<int>(partition_to_id.size())});
        if (inserted) {
          id_to_partition[it->second] = it->first;
        }
      }

      std::vector<TarelEdge> raw_tarel_edges = MakeTarelEdges(
        completed,
        std::function<StepPartitionId(Step)>([&](const Step& s) -> StepPartitionId {
          if (s.is_flex) {
            return partition_to_id.at("flex");
          }
          return partition_to_id.at(state.dest_trip_id_to_partition[s.destination_trip]);
        })
      );
      std::vector<TarelEdge> tarel_edges = MergeEquivalentTarelStates(raw_tarel_edges);

      TspGraphData graph = MakeTspGraphEdges(tarel_edges, state.boundary);
      std::optional<TspTourResult> tour_result = SolveTspAndExtractTour(tarel_edges, graph, state.boundary);
      if (!tour_result.has_value()) {
        std::cout << "No valid TSP tour exists (uses forbidden edges)\n";
      } else {
        std::vector<Path> feasible_paths = ComputeMinDurationFeasiblePaths(*tour_result, state, completed);
        if (feasible_paths.size() > 0) {
          std::cout << feasible_paths.size() << " feasible paths with start times:";
          for (const Path& p : feasible_paths) {
            std::cout << " " << p.merged_step.origin_time.ToString();
          }
          std::cout << "\n";
          std::cout << "Duration: " << TimeSinceServiceStart{feasible_paths[0].DurationSeconds()}.ToString() << "\n";
          PrintTarelTourResults(std::cout, *tour_result, state, feasible_paths[0], id_to_partition, completed);
        } else {
          std::cout << "No feasible path?!\n";
        }
      }
    }

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
