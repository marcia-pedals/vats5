#include <crow/http_parser_merged.h>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_set>

#include "solver/branch_and_bound.h"
#include "solver/data.h"
#include "solver/graph_util.h"
#include "solver/steps_adjacency_list.h"
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

ProblemState EliminateEdges(const ProblemState& base_state, int threshold) {
    // Build mapping from PlainEdge to min duration.
    std::unordered_map<PlainEdge, int> edge_min_duration;
    for (const Step& step : base_state.minimal.AllSteps()) {
        PlainEdge edge = step.Plain();
        auto it = edge_min_duration.find(edge);
        if (it == edge_min_duration.end() || step.DurationSeconds() < it->second) {
            edge_min_duration[edge] = step.DurationSeconds();
        }
    }

    // Sort by duration descending.
    std::vector<std::pair<PlainEdge, int>> sorted_edges(
        edge_min_duration.begin(), edge_min_duration.end());
    std::sort(sorted_edges.begin(), sorted_edges.end(),
              [](const auto& a, const auto& b) {
                  return a.second > b.second;
              });

    std::erase_if(sorted_edges, [](const std::pair<PlainEdge, int>& e) {
      return e.second < 40 * 60;
    });

    // Process all edges in parallel with lower bound computation.
    int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 4;

    ProblemState current_state = base_state;
    std::mutex state_mutex;

    std::atomic<int> next_index{0};
    std::mutex output_mutex;
    int total = sorted_edges.size();

    auto worker = [&]() {
        while (true) {
            int i = next_index.fetch_add(1);
            if (i >= total) break;

            const auto& [edge, duration] = sorted_edges[i];

            ProblemState local_state;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                local_state = current_state;
            }

            BranchEdge branch_edge{edge.a, edge.b};
            std::vector<ProblemConstraint> constraints = {branch_edge.Require()};
            ProblemState constrained = ApplyConstraints(local_state, constraints);
            auto lb_result = ComputeTarelLowerBound(constrained, threshold);

            std::string edge_name =
                base_state.StopName(edge.a) + " -> " + base_state.StopName(edge.b);
            std::string dur_str = TimeSinceServiceStart{duration}.ToString();

            bool eliminated = !lb_result.has_value() || lb_result->optimal_value >= threshold;

            size_t extreme_count = 0;
            if (eliminated) {
                std::lock_guard<std::mutex> lock(state_mutex);
                std::vector<ProblemConstraint> forbid = {branch_edge.Forbid()};
                current_state = ApplyConstraints(current_state, forbid);
                auto extreme_stops = ComputeExtremeStops(
                    current_state.completed, current_state.required_stops,
                    current_state.boundary.start);
                extreme_count = extreme_stops.size();
            }

            std::lock_guard<std::mutex> lock(output_mutex);
            std::cout << "  " << edge_name << " (" << dur_str << "): ";
            if (eliminated) {
                std::cout << "eliminated (extreme stops: " << extreme_count << ")\n";
            } else {
                std::cout << TimeSinceServiceStart{lb_result->optimal_value}.ToString() << "\n";
            }
        }
    };

    std::cout << "Processing " << total << " edges with " << num_threads
              << " threads (threshold " << TimeSinceServiceStart{threshold}.ToString() << "):\n";
    std::vector<std::thread> threads;
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back(worker);
    }
    for (auto& t : threads) {
        t.join();
    }

    return current_state;
}

void EliminateIntermediateStops(ProblemState& state, int threshold) {
    // Collect all non-required, non-boundary stops.
    std::vector<StopId> intermediate_stops;
    for (int i = 0; i < state.minimal.NumStops(); ++i) {
        StopId stop{i};
        if (!state.required_stops.count(stop)) {
            intermediate_stops.push_back(stop);
        }
    }

    int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 4;

    std::atomic<int> next_index{0};
    std::mutex output_mutex;
    std::mutex state_mutex;
    int total = intermediate_stops.size();

    // For each stop, compute lb when requiring it.
    // If lb >= threshold, the stop is never on an optimal tour.
    std::vector<bool> eliminated(total, false);

    auto worker = [&]() {
        while (true) {
            int i = next_index.fetch_add(1);
            if (i >= total) break;

            StopId stop = intermediate_stops[i];

            ProblemState local_state;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                local_state = state;
            }

            // Require this stop and compute lower bound.
            auto new_required = local_state.required_stops;
            new_required.insert(stop);
            ProblemState with_stop = local_state.WithRequiredStops(new_required);
            auto lb_result = ComputeTarelLowerBound(with_stop, threshold);

            bool is_eliminated = !lb_result.has_value() || lb_result->optimal_value >= threshold;

            if (is_eliminated) {
                std::lock_guard<std::mutex> lock(state_mutex);
                // Forbid all edges incident to this stop.
                std::vector<ProblemConstraint> forbids;
                for (const Step& step : state.minimal.AllSteps()) {
                    if (step.origin.stop == stop || step.destination.stop == stop) {
                        forbids.push_back(BranchEdge{step.origin.stop, step.destination.stop}.Forbid());
                    }
                }
                if (!forbids.empty()) {
                    state = ApplyConstraints(state, forbids);
                }
                eliminated[i] = true;
            }

            std::lock_guard<std::mutex> lock(output_mutex);
            std::cout << "  " << local_state.StopName(stop) << ": ";
            if (is_eliminated) {
                std::cout << "eliminated\n";
            } else {
                std::cout << TimeSinceServiceStart{lb_result->optimal_value}.ToString() << "\n";
            }
        }
    };

    std::cout << "Processing " << total << " intermediate stops with " << num_threads
              << " threads (threshold " << TimeSinceServiceStart{threshold}.ToString() << "):\n";
    std::vector<std::thread> threads;
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back(worker);
    }
    for (auto& t : threads) {
        t.join();
    }

    int num_eliminated = 0;
    for (bool e : eliminated) {
        if (e) num_eliminated++;
    }
    std::cout << "Eliminated " << num_eliminated << " / " << total << " intermediate stops.\n";
}

int main() {
    const std::string gtfs_path = "../data/RG_20260108_all";
    // const std::string gtfs_path = "../data/RG_20250718_BA";

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
    ProblemState state = InitializeProblemState(steps_from_gtfs, bart_stops, /*optimize_edges=*/true);

    state = EliminateEdges(state, 5 * 3600 + 24 * 60);
    // EliminateIntermediateStops(state, 5 * 3600 + 24 * 60);

    auto extreme_stops = ComputeExtremeStops(
      state.completed, state.required_stops, state.boundary.start
    );
    std::cout << "Final extreme stop count: " << extreme_stops.size() << "\n";
    state = MakeProblemState(
      MakeAdjacencyList(ReduceToMinimalSystemPaths(state.minimal, extreme_stops).AllMergedSteps()),
      state.boundary,
      extreme_stops,
      state.stop_names,
      state.step_partition_names,
      state.original_edges
    );
    // state = state.WithRequiredStops(extreme_stops);

    std::string run_dir = GetTimestampDir();
    std::filesystem::create_directory(run_dir);
    std::cout << "Output directory: " << run_dir << std::endl;

    BranchAndBoundSolve(state, &std::cout, run_dir);

    return 0;
}
