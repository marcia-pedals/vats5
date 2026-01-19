#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_set>

#include "solver/data.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

std::string GetTimestampDir() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::localtime(&time_t_now);
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
}

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
    SolutionState state = InitializeSolutionState(steps_from_gtfs, bart_stops);

    // Dummy code for typechecking.
    StepPathsAdjacencyList completed =
      ReduceToMinimalSystemPaths(state.adj, state.stops, /*keep_through_other_destination=*/true);

    // Add END -> START edge.
    // It's safe to push a new group without checking for an existing group with
    // `start` dest because no edges (other than this one we're adding right
    // now) go into `start`.
    assert(completed.adjacent[state.boundary.end].size() == 0);
    completed.adjacent[state.boundary.end].push_back({ZeroPath(state.boundary.end, state.boundary.start)});

    std::string run_dir = GetTimestampDir();
    std::filesystem::create_directory(run_dir);
    std::cout << "Output directory: " << run_dir << std::endl;

    {
      std::unordered_map<Step, std::string> step_to_tours;

      for (int tour_idx = 0; tour_idx < 100; ++tour_idx) {
        std::string tour_dir = run_dir + "/tour" + std::to_string(tour_idx);
        std::filesystem::create_directory(tour_dir);
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
        TspTourResult tour_result = SolveTspAndExtractTour(tarel_edges, graph, state.boundary, &tsp_log);
        std::cout << "Tour " << tour_idx << " LB: " << TimeSinceServiceStart{tour_result.optimal_value}.ToString() << "\n";
        std::vector<Path> feasible_paths = ComputeMinDurationFeasiblePaths(tour_result, state, completed);
        if (feasible_paths.size() > 0) {
          log << feasible_paths.size() << " feasible paths with start times:";
          for (const Path& p : feasible_paths) {
            log << " " << p.merged_step.origin_time.ToString();
          }
          log << "\n";
          log << "Duration: " << TimeSinceServiceStart{feasible_paths[0].DurationSeconds()}.ToString() << "\n";
          PrintTarelTourResults(log, tour_result, state, feasible_paths[0], id_to_partition);
        } else {
          log << "No feasible path?!\n";
        }
        log << std::flush;

        int num_steps_partitioned = 0;
        const std::string suffix = "-T" + std::to_string(tour_idx);
        for (const TarelEdge& e : tour_result.tour_edges) {
          for (const Step& s : e.steps) {
            if (!step_to_tours[s].ends_with(suffix)) {
              step_to_tours[s] += suffix;
              num_steps_partitioned += 1;
            }
          }
        }
        log << "Num steps partitioned: " << num_steps_partitioned << "\n";
        log << std::flush;
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
      TspTourResult tour_result = SolveTspAndExtractTour(tarel_edges, graph, state.boundary);
      std::vector<Path> feasible_paths = ComputeMinDurationFeasiblePaths(tour_result, state, completed);
      if (feasible_paths.size() > 0) {
        std::cout << feasible_paths.size() << " feasible paths with start times:";
        for (const Path& p : feasible_paths) {
          std::cout << " " << p.merged_step.origin_time.ToString();
        }
        std::cout << "\n";
        std::cout << "Duration: " << TimeSinceServiceStart{feasible_paths[0].DurationSeconds()}.ToString() << "\n";
        PrintTarelTourResults(std::cout, tour_result, state, feasible_paths[0], id_to_partition);
      } else {
        std::cout << "No feasible path?!\n";
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
