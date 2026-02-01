#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "solver/branch_and_bound.h"
#include "solver/data.h"
#include "solver/tarel_graph.h"

using namespace vats5;

// Extract all intermediate stops from a path (excluding origin and destination)
std::unordered_set<StopId> GetIntermediateStops(const Path& path, StopId a, StopId b) {
    std::unordered_set<StopId> stops;
    for (const Step& step : path.steps) {
        if (step.origin.stop != a && step.origin.stop != b) {
            stops.insert(step.origin.stop);
        }
        if (step.destination.stop != a && step.destination.stop != b) {
            stops.insert(step.destination.stop);
        }
    }
    return stops;
}

// Find stops that appear in ALL paths
std::unordered_set<StopId> StopsOnAllPaths(std::span<const Path> paths, StopId a, StopId b) {
    if (paths.empty()) {
        return {};
    }

    // Start with stops from the first path
    std::unordered_set<StopId> result = GetIntermediateStops(paths[0], a, b);

    // Intersect with stops from all other paths
    for (size_t i = 1; i < paths.size(); ++i) {
        std::unordered_set<StopId> path_stops = GetIntermediateStops(paths[i], a, b);
        std::unordered_set<StopId> intersection;
        for (StopId stop : result) {
            if (path_stops.count(stop)) {
                intersection.insert(stop);
            }
        }
        result = std::move(intersection);
    }
    return result;
}

// Compute extreme stops: required stops that are not "inner" stops.
// Inner stops are stops that appear on all paths between some pair of required stops.
std::vector<StopId> ComputeExtremeStops(const ProblemState& state) {
  std::vector<StopId> required_stops_vec(
      state.required_stops.begin(),
      state.required_stops.end());

  // Collect all inner stops
  std::unordered_set<StopId> inner_stops;
  for (size_t i = 0; i < required_stops_vec.size(); ++i) {
    for (size_t j = i + 1; j < required_stops_vec.size(); ++j) {
      StopId a = required_stops_vec[i];
      StopId b = required_stops_vec[j];

      auto paths_ab = state.completed.PathsBetween(a, b);
      auto paths_ba = state.completed.PathsBetween(b, a);

      // Get stops on all a->b paths
      std::unordered_set<StopId> stops_ab = StopsOnAllPaths(paths_ab, a, b);
      // Get stops on all b->a paths
      std::unordered_set<StopId> stops_ba = StopsOnAllPaths(paths_ba, a, b);

      // Add stops that are on all paths in both directions
      for (StopId stop : stops_ab) {
        if (stops_ba.count(stop)) {
          inner_stops.insert(stop);
        }
      }
    }
  }

  // Collect extreme stops (required stops that are not inner stops, excluding START and END)
  std::vector<StopId> extreme_stops;
  for (StopId stop : state.required_stops) {
    if (!inner_stops.count(stop) &&
        stop != state.boundary.start &&
        stop != state.boundary.end) {
      extreme_stops.push_back(stop);
    }
  }
  return extreme_stops;
}

void PrintDegrees(const ProblemState& state) {
  std::unordered_map<StopId, std::unordered_set<StopId>> adjacent;
  for (const Step& step : state.minimal.AllSteps()) {
    if (step.origin.stop == state.boundary.start || step.destination.stop == state.boundary.end) {
      continue;
    }
    adjacent[step.origin.stop].insert(step.destination.stop);
    adjacent[step.destination.stop].insert(step.origin.stop);
  }
  std::vector<StopId> degree1;
  std::vector<StopId> degree2;
  for (const auto& [stop, stop_adj] : adjacent) {
    if (stop_adj.size() == 1) {
      degree1.push_back(stop);
    }
    if (stop_adj.size() == 2) {
      degree2.push_back(stop);
    }
  }

  std::cout << "Degree 1 stops (" << degree1.size() << "):";
  for (StopId stop : degree1) {
    std::cout << " " << state.StopName(stop);
  }
  std::cout << "\n";

  std::cout << "Degree 2 stops (" << degree2.size() << "):";
  for (StopId stop : degree2) {
    std::cout << " " << state.StopName(stop);
  }
  std::cout << "\n";
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
    ProblemState state = InitializeProblemState(steps_from_gtfs, bart_stops, true);

    std::cout << "Number of required stops: " << state.required_stops.size() << std::endl;

    int best_ub = 5 * 3600 + 44 * 60;
    std::optional<TspTourResult> initial_result = ComputeTarelLowerBound(state, best_ub);
    assert(initial_result.has_value());
    std::cout << "Initial LB: " << TimeSinceServiceStart{initial_result->optimal_value}.ToString() << "\n";
    PrintDegrees(state);

    std::unordered_map<BranchEdge, int> edge_min_duration;
    for (const Step& step : state.minimal.AllSteps()) {
      if (step.origin.stop == state.boundary.start || step.destination.stop == state.boundary.end) {
        continue;
      }

      BranchEdge edge{step.origin.stop, step.destination.stop};
      auto it = edge_min_duration.find(edge);
      int duration = step.DurationSeconds();
      if (it == edge_min_duration.end() || duration < it->second) {
        edge_min_duration[edge] = duration;
      }
    }
    std::vector<std::pair<BranchEdge, int>> all_edges;
    all_edges.reserve(edge_min_duration.size());
    for (const auto& [edge, duration] : edge_min_duration) {
      all_edges.emplace_back(edge, duration);
    }
    std::ranges::sort(all_edges, [](const auto& a, const auto& b) {
      return a.second > b.second;
    });

    std::atomic<size_t> next_index{0};
    std::mutex output_mutex;
    unsigned int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 1;

    std::mutex base_state_mutex;
    ProblemState base_state = state;

    auto worker = [&]() {
      while (true) {
        size_t i = next_index.fetch_add(1);
        if (i >= all_edges.size()) break;

        const auto& p = all_edges[i];
        const BranchEdge& edge = p.first;
        const int& min_duration = p.second;

        ProblemState base_copy;
        {
          std::lock_guard<std::mutex> lock(base_state_mutex);
          base_copy = base_state;
        }

        ProblemState constrained_state = ApplyConstraints(base_copy, {edge.Require()});
        std::optional<TspTourResult> result = ComputeTarelLowerBound(constrained_state, best_ub);

        if (!result.has_value()) {
          std::lock_guard<std::mutex> lock(base_state_mutex);
          base_state = ApplyConstraints(base_state, {edge.Forbid()});
          base_copy = base_state;
        }

        std::vector<StopId> extreme_stops = ComputeExtremeStops(base_copy);

        std::lock_guard<std::mutex> lock(output_mutex);
        std::cout
          << state.StopName(edge.a) << " -> " << state.StopName(edge.b) << " ["
          << TimeSinceServiceStart{min_duration}.ToString() << "]: ";
        if (result.has_value()) {
          std::cout << "not eliminated - lb = " << TimeSinceServiceStart{result->optimal_value}.ToString() << "\n";
        } else {
          std::cout << "eliminated!\n";
          std::cout << "extreme: (" << extreme_stops.size() << ")";
          for (StopId e : extreme_stops) {
            std::cout << " " << base_copy.StopName(e);
          }
          std::cout << "\n";
          PrintDegrees(base_copy);
        }
      }
    };

    std::vector<std::thread> threads;
    threads.reserve(num_threads);
    for (unsigned int t = 0; t < num_threads; ++t) {
      threads.emplace_back(worker);
    }
    for (auto& thread : threads) {
      thread.join();
    }

    return 0;
}
