#include <algorithm>
#include <iostream>
#include <limits>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/branch_and_bound.h"
#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
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

 std::set<std::vector<StopId>> GetDistinctStopSequences(std::span<const Path> paths) {
  std::set<std::vector<StopId>> different_paths;
  for (const Path& path : paths) {
    std::vector<StopId> stops;
    if (path.steps.size() > 0) {
      stops.push_back(path.steps[0].origin.stop);
    }
    for (const Step& step : path.steps) {
      stops.push_back(step.destination.stop);
    }
    different_paths.insert(stops);
  }
  return different_paths;
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

// Compute "rescue" map: for each edge, which extreme stops would be bypassed if that edge is taken.
std::unordered_map<BranchEdge, std::unordered_set<StopId>> ComputeRescueMap(
    const ProblemState& state,
    const std::vector<StopId>& extreme_stops
) {
  std::vector<StopId> required_stops_vec(
      state.required_stops.begin(),
      state.required_stops.end()
  );

  std::unordered_map<BranchEdge, std::unordered_set<StopId>> rescue;

  for (StopId extreme_stop : extreme_stops) {
    for (size_t i = 0; i < required_stops_vec.size(); ++i) {
      for (size_t j = i + 1; j < required_stops_vec.size(); ++j) {
        StopId a = required_stops_vec[i];
        StopId b = required_stops_vec[j];

        auto sequences_ab = GetDistinctStopSequences(state.completed.PathsBetween(a, b));
        auto sequences_ba = GetDistinctStopSequences(state.completed.PathsBetween(b, a));

        std::set<std::vector<StopId>> sequences;
        for (const std::vector<StopId>& seq : sequences_ab) {
          sequences.insert(seq);
        }
        for (const std::vector<StopId>& seq : sequences_ba) {
          std::vector<StopId> reversed = seq;
          std::reverse(reversed.begin(), reversed.end());
          sequences.insert(reversed);
        }

        std::set<std::vector<StopId>> sequences_with;
        std::set<std::vector<StopId>> sequences_without;
        for (const std::vector<StopId>& seq : sequences) {
          bool has = false;
          for (StopId stop : seq) {
            if (stop == extreme_stop) {
              has = true;
              break;
            }
          }
          if (has) {
            sequences_with.insert(seq);
          } else {
            sequences_without.insert(seq);
          }
        }

        if (sequences_with.size() > 0 && sequences_without.size() == 1) {
          std::vector<StopId> seq = *(sequences_without.begin());
          for (size_t k = 0; k < seq.size() - 1; ++k) {
            StopId ea = seq[k];
            StopId eb = seq[k + 1];
            if (ea.v > eb.v) {
              std::swap(ea, eb);
            }
            rescue[BranchEdge{ea, eb}].insert(extreme_stop);
          }
        }
      }
    }
  }

  return rescue;
}

std::unordered_set<BranchEdge> EdgesNotOnAllPaths(std::span<const Path> paths) {
  if (paths.size() == 0) {
    return {};
  }

  std::unordered_set<BranchEdge> edges_on_all_paths;
  for (const Step& step : paths[0].steps) {
    edges_on_all_paths.insert(BranchEdge{step.origin.stop, step.destination.stop});
  }
  for (const Path& path : paths) {
    std::unordered_set<BranchEdge> edges_on_this_path;
    for (const Step& step : path.steps) {
      edges_on_this_path.insert(BranchEdge{step.origin.stop, step.destination.stop});
    }
    std::unordered_set<BranchEdge> intersection;
    for (const BranchEdge& edge : edges_on_all_paths) {
      if (edges_on_this_path.contains(edge)) {
        intersection.insert(edge);
      }
    }
    edges_on_all_paths = std::move(intersection);
  }

  std::unordered_set<BranchEdge> result;
  for (const Path& path : paths) {
    for (const Step& step : path.steps) {
      BranchEdge edge{step.origin.stop, step.destination.stop};
      if (!edges_on_all_paths.contains(edge)) {
        result.insert(edge);
      }
    }
  }

  return result;
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
    ProblemState initial_state = InitializeProblemState(steps_from_gtfs, bart_stops, true);

    std::cout << "Number of required stops: " << initial_state.required_stops.size() << std::endl;

    std::vector<StopId> extreme_stops = ComputeExtremeStops(initial_state);
    // auto rescue = ComputeRescueMap(initial_state, extreme_stops);

    // std::vector<BranchEdge> rescue_ordered;
    // for (const auto& [edge, _] : rescue) {
    //   rescue_ordered.push_back(edge);
    // }
    // std::ranges::sort(rescue_ordered, [&](const BranchEdge& a, const BranchEdge& b) {
    //   return rescue[a].size() > rescue[b].size();
    // });
    // for (const BranchEdge& edge : rescue_ordered) {
    //   std::cout << initial_state.StopName(edge.a) << " <-> " << initial_state.StopName(edge.b) << " (" << rescue[edge].size() << "): ";
    //   for (StopId stop : rescue[edge]) {
    //     std::cout << initial_state.StopName(stop) << " ";
    //   }
    //   std::cout << "\n";
    // }

    std::cout << "\nExtreme stops:\n";
    for (StopId stop : extreme_stops) {
        std::cout << "  " << initial_state.StopName(stop) << "\n";
    }
    std::cout << "\nTotal: " << extreme_stops.size() << " extreme stops\n";

    std::unordered_map<BranchEdge, int> edge_min_duration;
    for (const Step& step : initial_state.minimal.AllSteps()) {
      StopId a = step.origin.stop;
      StopId b = step.destination.stop;
      if (a.v > b.v) {
        std::swap(a, b);
      }
      BranchEdge edge{a, b};
      int duration = step.DurationSeconds();
      auto it = edge_min_duration.find(edge);
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
      return a.second > b.second;  // descending by duration
    });

    std::cout << "Edge count: " << all_edges.size() << "\n";

    size_t baseline_extreme_count = extreme_stops.size();
    BranchEdge found_edge;
    bool found = false;
    int i = 0;
    for (const auto& [edge, duration] : all_edges) {
      BranchEdge edge_rv{edge.b, edge.a};
      ProblemState state = ApplyConstraints(initial_state, {edge.Forbid(), edge_rv.Forbid()});
      std::vector<StopId> extreme_forbid = ComputeExtremeStops(state);
      if (extreme_forbid.size() < baseline_extreme_count) {
        found_edge = edge;
        found = true;
        std::cout << "Found at index " << i << ": " << initial_state.StopName(edge.a)
          << " <-> " << initial_state.StopName(edge.b)
          << " (duration " << duration << ", extreme count " << extreme_forbid.size() << ")\n";
        break;
      }

      if (i % 10 == 0) {
        std::cout << i << " / " << all_edges.size() << "\n";
      }
      i += 1;
    }

    if (found) {
      std::cout << "First edge that decreases extreme count: "
        << initial_state.StopName(found_edge.a) << " <-> " << initial_state.StopName(found_edge.b) << "\n";
    } else {
      std::cout << "No edge found that decreases extreme count\n";
    }

    // BranchEdge bedge = rescue_ordered[0];
    // BranchEdge bedge_rev{bedge.b, bedge.a};

    // ProblemState branch_require = ApplyConstraints(initial_state, {rescue_ordered[0].Require()});
    // std::vector<StopId> extreme_require = ComputeExtremeStops(branch_require);
    // std::cout << "\nExtreme require:\n";
    // for (StopId stop : extreme_require) {
    //     std::cout << "  " << branch_require.StopName(stop) << "\n";
    // }
    // std::cout << "\nTotal: " << extreme_require.size() << " extreme stops\n";

    // ProblemState branch_forbid = ApplyConstraints(initial_state, {bedge.Forbid(), bedge_rev.Forbid()});
    // std::vector<StopId> extreme_forbid = ComputeExtremeStops(branch_forbid);
    // std::cout << "\nExtreme forbid:\n";
    // for (StopId stop : extreme_forbid) {
    //     std::cout << "  " << branch_forbid.StopName(stop) << "\n";
    // }
    // std::cout << "\nTotal: " << extreme_forbid.size() << " extreme stops\n";


    // Sort for std::next_permutation
    std::sort(extreme_stops.begin(), extreme_stops.end(), [](StopId a, StopId b) {
        return a.v < b.v;
    });

    int best_duration = std::numeric_limits<int>::max();
    std::vector<StopId> best_permutation;
    int permutation_count = 0;

    std::cout << "\nSearching all permutations...\n";
    do {
        ++permutation_count;

        // Build route: START -> extreme_stops[0] -> ... -> extreme_stops[n-1] -> END
        std::vector<StopId> route;
        route.push_back(initial_state.boundary.start);
        for (StopId stop : extreme_stops) {
            route.push_back(stop);
        }
        route.push_back(initial_state.boundary.end);

        // Compute paths along this route
        std::vector<Step> feasible_steps;
        feasible_steps.push_back(Step::PrimitiveFlex(
            initial_state.boundary.start, initial_state.boundary.start, 0, TripId::NOOP));

        bool valid = true;
        for (size_t i = 0; i + 1 < route.size(); ++i) {
            StopId from = route[i];
            StopId to = route[i + 1];
            auto paths = initial_state.completed.PathsBetween(from, to);
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

        if (best_step_it->DurationSeconds() < best_duration) {
            best_duration = best_step_it->DurationSeconds();
            best_permutation = extreme_stops;
        }
    } while (std::next_permutation(extreme_stops.begin(), extreme_stops.end(),
        [](StopId a, StopId b) { return a.v < b.v; }));

    std::cout << "Checked " << permutation_count << " permutations\n";
    std::cout << "\nBest duration: " << TimeSinceServiceStart{best_duration}.ToString() << "\n";
    std::cout << "Best permutation:\n";
    std::cout << "  " << initial_state.StopName(initial_state.boundary.start) << " (START)\n";
    for (StopId stop : best_permutation) {
        std::cout << "  " << initial_state.StopName(stop) << "\n";
    }
    std::cout << "  " << initial_state.StopName(initial_state.boundary.end) << " (END)\n";

    return 0;
}
