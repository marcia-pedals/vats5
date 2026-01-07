#include "shortest_path.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <mutex>
#include <queue>
#include <thread>

#include "solver/step_merge.h"

namespace vats5 {

StepsAdjacencyList MakeAdjacencyList(const std::vector<Step>& steps) {
  StepsAdjacencyList adjacency_list;

  // Group steps by origin_stop and destination_stop
  std::unordered_map<StopId, std::unordered_map<StopId, std::vector<Step>>>
      groups;

  int max_stop_id = 0;
  for (const Step& step : steps) {
    groups[step.origin_stop][step.destination_stop].push_back(step);
    max_stop_id = std::max(max_stop_id, step.origin_stop.v);
    max_stop_id = std::max(max_stop_id, step.destination_stop.v);
  }

  // Process each group: sort and make minimal
  for (auto& [origin_stop, destination_map] : groups) {
    std::vector<StepGroup> sorted_minimal_groups;

    for (auto& [destination_stop, step_vec] : destination_map) {
      SortSteps(step_vec);
      MakeMinimalCover(step_vec);
      if (!step_vec.empty()) {
        StepGroup sg;

        // Extract flex step if present (it's always first after sorting)
        size_t fixed_start = 0;
        if (step_vec[0].is_flex) {
          sg.flex_step = step_vec[0];
          fixed_start = 1;
        }

        // Copy fixed-schedule steps
        sg.steps.reserve(step_vec.size() - fixed_start);
        sg.departure_times_div10.reserve(step_vec.size() - fixed_start);
        for (size_t i = fixed_start; i < step_vec.size(); ++i) {
          sg.steps.push_back(step_vec[i]);
          sg.departure_times_div10.push_back(
              static_cast<int16_t>(step_vec[i].origin_time.seconds / 10)
          );
        }

        sorted_minimal_groups.push_back(std::move(sg));
      }
    }

    if (!sorted_minimal_groups.empty()) {
      adjacency_list.adjacent[origin_stop] = std::move(sorted_minimal_groups);
    }
  }

  adjacency_list.stop_id_ub = max_stop_id + 1;
  return adjacency_list;
}

// Compact entry for the priority queue - only stores what's needed for
// ordering. The full PathState is stored separately in best_arrival map.
struct FrontierEntry {
  StopId destination_stop;
  TimeSinceServiceStart arrival_time;
};

struct FrontierEntryComparator {
  bool operator()(const FrontierEntry& a, const FrontierEntry& b) const {
    // Highest priority is to arrive earliest.
    if (a.arrival_time.seconds != b.arrival_time.seconds) {
      return a.arrival_time.seconds > b.arrival_time.seconds;
    }

    // Break ties arbitrarily but consistently.
    return a.destination_stop.v > b.destination_stop.v;
  }
};

// Return pointer to the first fixed-schedule step departing >= `t`, or nullptr.
// Precondition: `step_group.steps` contains only fixed-schedule steps (no
// flex).
const Step* FindDepartureAtOrAfter(
    const StepGroup& step_group, TimeSinceServiceStart t
) {
  const auto& steps = step_group.steps;
  const auto& departure_times_div10 = step_group.departure_times_div10;

  if (steps.empty()) {
    return nullptr;
  }

  // Binary search on the cache-friendly departure_times_div10 array.
  // We search for t.seconds / 10, which may undershoot due to rounding.
  int16_t target_div10 = static_cast<int16_t>(t.seconds / 10);
  auto lower_bound_it = std::lower_bound(
      departure_times_div10.begin(), departure_times_div10.end(), target_div10
  );

  if (lower_bound_it == departure_times_div10.end()) {
    return nullptr;
  }

  // Linear scan forward to fix rounding errors.
  // The binary search may have landed on a step that departs before t
  // due to integer division truncation.
  size_t index = lower_bound_it - departure_times_div10.begin();
  while (index < steps.size() && steps[index].origin_time.seconds < t.seconds) {
    ++index;
  }

  if (index >= steps.size()) {
    return nullptr;
  }

  return &steps[index];
}

// Backtrack through the search results to reconstruct the full path.
// Returns the steps in order from origin to destination.
std::vector<Step> BacktrackPath(
    const std::vector<PathState>& search_result, StopId dest
) {
  std::vector<Step> path;
  PathState state = search_result[dest.v];
  while (state.step.origin_trip != TripId::NOOP) {
    path.push_back(state.step);
    state = search_result[state.step.origin_stop.v];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

// Compute the merged step for a path, with proper origin time calculation.
// The origin time adjustment handles flex paths that transition to fixed trips.
Step ComputeMergedStep(const std::vector<Step>& path) {
  if (path.empty()) {
    return Step{};
  }

  const Step& first = path.front();
  const Step& last = path.back();

  // Calculate origin time with flex adjustment
  TimeSinceServiceStart origin_time = first.origin_time;
  bool is_flex = first.is_flex;

  if (is_flex && path.size() > 1) {
    // If the path starts with flex and transitions to a fixed trip,
    // we can delay departure. Find where flex ends.
    TimeSinceServiceStart flex_arrival = first.destination_time;
    for (size_t i = 1; i < path.size(); ++i) {
      if (!path[i].is_flex) {
        // Found transition from flex to fixed
        // We can wait: (fixed_departure - flex_arrival) extra time
        origin_time.seconds +=
            path[i].origin_time.seconds - flex_arrival.seconds;
        is_flex = false;  // Path becomes non-flex when we connect to fixed
        break;
      }
      flex_arrival = path[i].destination_time;
    }
  }

  return Step{
      first.origin_stop,
      last.destination_stop,
      origin_time,
      last.destination_time,
      first.origin_trip,
      last.destination_trip,
      is_flex
  };
}

// Sentinel PathState representing an unvisited stop.
const PathState kUnvisitedPathState{Step{
    StopId{-1},
    StopId{-1},
    TimeSinceServiceStart{std::numeric_limits<int>::max()},
    TimeSinceServiceStart{std::numeric_limits<int>::max()},
    TripId{-1},
    TripId{-1},
    false
}};

std::vector<PathState> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list,
    TimeSinceServiceStart origin_time,
    StopId origin_stop,
    const std::unordered_set<StopId>& destinations
) {
  std::unordered_set<StopId> reached_destinations;
  std::vector<bool> finalized(adjacency_list.stop_id_ub, false);

  // Compact priority queue storing only destination_stop and arrival_time.
  std::priority_queue<
      FrontierEntry,
      std::vector<FrontierEntry>,
      FrontierEntryComparator>
      frontier;

  // Maps stop index to the best PathState we've found for reaching it.
  // Unvisited stops have kUnvisitedPathState.
  std::vector<PathState> best_arrival(
      adjacency_list.stop_id_ub, kUnvisitedPathState
  );

  // The state at the origin stop.
  const Step initial_step{
      origin_stop,
      origin_stop,
      origin_time,
      origin_time,
      TripId::NOOP,
      TripId::NOOP,
      false  // is_flex
  };
  frontier.push(FrontierEntry{origin_stop, origin_time});
  best_arrival[origin_stop.v] = PathState{initial_step};

  while (!frontier.empty()) {
    const FrontierEntry current_entry = frontier.top();
    const StopId current_stop = current_entry.destination_stop;
    const TimeSinceServiceStart current_time = current_entry.arrival_time;
    frontier.pop();

    if (finalized[current_stop.v]) {
      continue;
    }
    finalized[current_stop.v] = true;

    if (destinations.contains(current_stop)) {
      reached_destinations.insert(current_stop);
      if (reached_destinations.size() == destinations.size()) {
        return best_arrival;
      }
    }

    auto adj_it = adjacency_list.adjacent.find(current_stop);
    if (adj_it == adjacency_list.adjacent.end()) {
      continue;
    }

    for (const StepGroup& step_group : adj_it->second) {
      // Handle flex trip if present
      if (step_group.flex_step.has_value()) {
        const Step& flex_step = *step_group.flex_step;
        const StopId next_stop = flex_step.destination_stop;
        if (!finalized[next_stop.v]) {
          // Calculate arrival time based on current time + duration
          TimeSinceServiceStart arrival_time{
              current_time.seconds + flex_step.FlexDurationSeconds()
          };

          // Only add if this is a better arrival time
          if (arrival_time.seconds <
              best_arrival[next_stop.v].step.destination_time.seconds) {
            Step flex_step_at_now = flex_step;
            flex_step_at_now.origin_time = current_time;
            flex_step_at_now.destination_time = arrival_time;

            best_arrival[next_stop.v] = PathState{flex_step_at_now};
            frontier.push(FrontierEntry{next_stop, arrival_time});
          }
        }
      }

      // Regular fixed-time trip handling
      {
        const Step* next_step_ptr =
            FindDepartureAtOrAfter(step_group, current_time);
        if (next_step_ptr == nullptr) {
          continue;
        }

        const Step& next_step = *next_step_ptr;
        const StopId next_stop = next_step.destination_stop;
        if (finalized[next_stop.v]) {
          continue;
        }

        // Only add if this is a better arrival time
        if (next_step.destination_time.seconds <
            best_arrival[next_stop.v].step.destination_time.seconds) {
          best_arrival[next_stop.v] = PathState{next_step};
          frontier.push(FrontierEntry{next_stop, next_step.destination_time});
        }
      }
    }
  }

  return best_arrival;
}

std::unordered_map<StopId, std::vector<Path>> FindMinimalPathSet(
    const StepsAdjacencyList& adjacency_list,
    StopId origin,
    const std::unordered_set<StopId>& destinations
) {
  const TimeSinceServiceStart origin_time_ub{36 * 3600};

  const TimeSinceServiceStart big_time{origin_time_ub.seconds * 10};

  std::unordered_map<StopId, std::vector<Step>> result;
  std::unordered_map<StopId, TimeSinceServiceStart> current_origin_time;

  // A mapping from a Step in the result to its full Path.
  std::unordered_map<Step, Path> full_paths;

  // Whether the full path represented by step goes through a different element
  // of `destinations` before reaching its ultimate destination.
  std::unordered_map<Step, bool> through_other_destination;

  while (true) {
    // Find the smallest `current_origin_time` and query from there. Include all
    // destinations with that `current_origin_time` in the query.
    std::unordered_set<StopId> destinations_to_query;
    TimeSinceServiceStart query_time = big_time;
    for (const auto& dest : destinations) {
      if (current_origin_time[dest] < query_time) {
        query_time = current_origin_time[dest];
        destinations_to_query.clear();
      }
      if (current_origin_time[dest] == query_time) {
        destinations_to_query.insert(dest);
      }
    }

    if (query_time >= origin_time_ub) {
      break;
    }

    // Do query.
    const std::vector<PathState> search_result = FindShortestPathsAtTime(
        adjacency_list, query_time, origin, destinations_to_query
    );

    // Push all results and update current origin times.
    for (const auto& dest : destinations_to_query) {
      const PathState& path_state = search_result[dest.v];
      if (path_state.step.destination_time.seconds ==
          std::numeric_limits<int>::max()) {
        // There are no more steps for this destination.
        current_origin_time[dest] = big_time;
        continue;
      }

      // Backtrack to get the full path
      std::vector<Step> path_steps = BacktrackPath(search_result, dest);
      if (path_steps.empty()) {
        current_origin_time[dest] = big_time;
        continue;
      }

      // Construct merged step by computing origin_time from the path
      Step merged_step = ComputeMergedStep(path_steps);
      result[dest].push_back(merged_step);

      Path& full_path = full_paths[merged_step];
      full_path.merged_step = merged_step;
      full_path.steps = path_steps;

      // Check if path goes through another destination
      for (const Step& step : path_steps) {
        if (step.destination_stop != dest &&
            destinations.find(step.destination_stop) != destinations.end()) {
          through_other_destination[merged_step] = true;
          break;
        }
      }

      if (merged_step.is_flex) {
        // Pure flex step: we don't know when there is a departure that will be
        // faster than flex, so increment query time by 1 second.
        // TODO: This is very slow (approx responsible for 2x-ing the e2e
        // reduction of BART), and there might be some binary-search-like
        // strategy for identifying the earliest next departure that's faster
        // than flex.
        current_origin_time[dest] = query_time;
        current_origin_time[dest].seconds += 1;
      } else {
        // Non-flex step: This is the best step up until
        // merged_step.origin_time, so advance current origin time to 1 past
        // that.
        current_origin_time[dest] = merged_step.origin_time;
        current_origin_time[dest].seconds += 1;
      }
    }
  }

  std::unordered_map<StopId, std::vector<Path>> result_with_paths;

  for (const StopId dest : destinations) {
    auto& dest_result = result[dest];
    SortSteps(dest_result);
    MakeMinimalCover(dest_result);
    std::erase_if(dest_result, [&](const Step& s) {
      return through_other_destination[s];
    });

    // Remove any paths whose origin is after the ub.
    if (dest_result.size() > 0 && !dest_result.back().is_flex &&
        dest_result.back().origin_time >= origin_time_ub) {
      dest_result.pop_back();
    }

    auto& dest_paths = result_with_paths[dest];
    for (const Step whole_step : dest_result) {
      Path path = full_paths[whole_step];
      // Normalize flex steps: shift origin_time back to 00:00:00
      if (path.merged_step.is_flex) {
        int32_t offset = path.merged_step.origin_time.seconds;
        path.merged_step.origin_time.seconds = 0;
        path.merged_step.destination_time.seconds -= offset;
        for (Step& step : path.steps) {
          step.origin_time.seconds -= offset;
          step.destination_time.seconds -= offset;
        }
      }
      dest_paths.push_back(path);
    }
  }

  return result_with_paths;
}

PathsAdjacencyList ReduceToMinimalSystemPaths(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& system_stops
) {
  // Convert to vector for indexed access
  std::vector<StopId> origins(system_stops.begin(), system_stops.end());
  const size_t num_origins = origins.size();

  // Per-origin results stored in a vector (one per origin)
  std::vector<std::vector<std::vector<Path>>> per_origin_results(num_origins);

  // Work queue: atomic index for next origin to process
  std::atomic<size_t> next_origin{0};

  // Progress tracking
  std::atomic<size_t> completed{0};
  std::mutex progress_mutex;

  // Determine number of threads
  const unsigned int num_threads =
      std::max(1u, std::thread::hardware_concurrency());

  // Worker function that grabs unclaimed origins
  auto worker = [&]() {
    while (true) {
      // Atomically claim the next origin
      size_t i = next_origin.fetch_add(1);
      if (i >= num_origins) {
        break;
      }

      const StopId origin = origins[i];

      std::unordered_set<StopId> destinations = system_stops;
      destinations.erase(origin);

      std::unordered_map<StopId, std::vector<Path>> paths =
          FindMinimalPathSet(adjacency_list, origin, destinations);

      // Collect results for this origin
      std::vector<std::vector<Path>> origin_result;
      origin_result.reserve(destinations.size());
      for (const StopId dest : destinations) {
        origin_result.push_back(std::move(paths[dest]));
      }
      per_origin_results[i] = std::move(origin_result);

      // Update progress
      size_t done = ++completed;
      {
        std::lock_guard<std::mutex> lock(progress_mutex);
        std::cout << done << " / " << num_origins << "\n";
      }
    }
  };

  // Launch threads
  std::vector<std::thread> threads;
  threads.reserve(num_threads);
  for (unsigned int t = 0; t < num_threads; ++t) {
    threads.emplace_back(worker);
  }

  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }

  // Combine results
  PathsAdjacencyList result;
  result.adjacent.reserve(num_origins);
  for (size_t i = 0; i < num_origins; ++i) {
    result.adjacent[origins[i]] = std::move(per_origin_results[i]);
  }

  return result;
}

PathsAdjacencyList SplitPathsAt(
    const PathsAdjacencyList& paths,
    const std::unordered_set<StopId> intermediate_stops
) {
  std::unordered_map<StopId, std::unordered_map<StopId, std::vector<Path>>>
      result;
  result.reserve(paths.adjacent.size() + intermediate_stops.size());

  for (const auto& [origin_stop, path_groups] : paths.adjacent) {
    for (const auto& path_group : path_groups) {
      for (const Path& path : path_group) {
        std::vector<Step> accumulated_steps;
        std::optional<Step> accumulated_merged_step;
        for (const Step& step : path.steps) {
          if (accumulated_merged_step.has_value()) {
            accumulated_merged_step =
                MergedStep(*accumulated_merged_step, step);
          } else {
            accumulated_merged_step = step;
          }
          accumulated_steps.push_back(step);
          if (intermediate_stops.contains(
                  accumulated_merged_step->destination_stop
              )) {
            result[accumulated_merged_step->origin_stop]
                  [accumulated_merged_step->destination_stop]
                      .push_back(
                          Path{*accumulated_merged_step, accumulated_steps}
                      );
            accumulated_merged_step.reset();
            accumulated_steps.clear();
          }
        }
        if (accumulated_merged_step.has_value()) {
          result[accumulated_merged_step->origin_stop]
                [accumulated_merged_step->destination_stop]
                    .push_back(Path{*accumulated_merged_step, accumulated_steps}
                    );
          accumulated_merged_step.reset();
          accumulated_steps.clear();
        }
      }
    }
  }

  PathsAdjacencyList final_result;
  for (const auto& [origin_stop, dest_map] : result) {
    for (const auto& [dest_stop, paths] : dest_map) {
      std::vector<Path> deduped_paths;

      std::unordered_set<Step> seen_merged_steps;
      for (const Path& path : paths) {
        if (seen_merged_steps.insert(path.merged_step).second) {
          deduped_paths.push_back(path);
        }
      }

      std::sort(
          deduped_paths.begin(),
          deduped_paths.end(),
          [](const Path& a, const Path& b) {
            return a.merged_step.origin_time.seconds <
                   b.merged_step.origin_time.seconds;
          }
      );

      final_result.adjacent[origin_stop].push_back(std::move(deduped_paths));
    }
  }
  return final_result;
}

std::vector<Step> SnapToStops(
    const DataGtfsMapping& mapping,
    const std::unordered_set<StopId>& stops,
    double threshold_meters,
    const std::vector<Step>& path
) {
  // Helper to find the nearest stop from `stops` if within threshold, or return
  // the original stop if none are close enough.
  auto snap_stop = [&](StopId stop) -> StopId {
    // If stop is already in the set, no snapping needed.
    if (stops.count(stop) > 0) {
      return stop;
    }

    const StopPosition& pos = mapping.stop_positions[stop.v];
    StopId nearest = stop;
    double nearest_distance = threshold_meters;

    for (StopId candidate : stops) {
      const StopPosition& candidate_pos = mapping.stop_positions[candidate.v];
      double dx = pos.x_meters - candidate_pos.x_meters;
      double dy = pos.y_meters - candidate_pos.y_meters;
      double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = candidate;
      }
    }

    return nearest;
  };

  std::vector<Step> result;
  for (size_t i = 0; i < path.size(); ++i) {
    Step snapped = path[i];

    // Don't snap the origin of the first step or destination of the last step.
    if (i > 0) {
      snapped.origin_stop = snap_stop(path[i].origin_stop);
    }
    if (i < path.size() - 1) {
      snapped.destination_stop = snap_stop(path[i].destination_stop);
    }

    // Drop steps that become self-loops after snapping.
    if (snapped.origin_stop != snapped.destination_stop) {
      result.push_back(snapped);
    }
  }

  return result;
}

PathsAdjacencyList AdjacencyListSnapToStops(
    const DataGtfsMapping& mapping,
    double threshold_meters,
    const PathsAdjacencyList& paths
) {
  // Collect all StopIds that are keys in the adjacency list.
  std::unordered_set<StopId> stops;
  for (const auto& [stop_id, _] : paths.adjacent) {
    stops.insert(stop_id);
  }

  PathsAdjacencyList result;
  for (const auto& [origin_stop, path_groups] : paths.adjacent) {
    std::vector<std::vector<Path>> snapped_groups;
    for (const auto& path_group : path_groups) {
      std::vector<Path> snapped_paths;
      for (const Path& path : path_group) {
        std::vector<Step> snapped_steps =
            SnapToStops(mapping, stops, threshold_meters, path.steps);
        if (!snapped_steps.empty()) {
          snapped_paths.emplace_back(
              path.merged_step, std::move(snapped_steps)
          );
        }
      }
      if (!snapped_paths.empty()) {
        snapped_groups.push_back(std::move(snapped_paths));
      }
    }
    if (!snapped_groups.empty()) {
      result.adjacent[origin_stop] = std::move(snapped_groups);
    }
  }

  return result;
}

StepsAdjacencyList AdjacentPathsToStepsList(const PathsAdjacencyList& paths) {
  StepsAdjacencyList result;
  int max_stop_id = 0;
  for (const auto& [origin_stop, path_groups] : paths.adjacent) {
    max_stop_id = std::max(max_stop_id, origin_stop.v);
    std::vector<StepGroup> step_groups;
    for (const auto& path_group : path_groups) {
      StepGroup sg;
      for (const Path& path : path_group) {
        max_stop_id =
            std::max(max_stop_id, path.merged_step.destination_stop.v);

        if (path.merged_step.is_flex) {
          sg.flex_step = path.merged_step;
        } else {
          sg.steps.push_back(path.merged_step);
          sg.departure_times_div10.push_back(
              static_cast<int16_t>(path.merged_step.origin_time.seconds / 10)
          );
        }
      }
      if (sg.flex_step.has_value() || !sg.steps.empty()) {
        step_groups.push_back(std::move(sg));
      }
    }
    if (!step_groups.empty()) {
      result.adjacent[origin_stop] = std::move(step_groups);
    }
  }
  result.stop_id_ub = max_stop_id + 1;
  return result;
}

}  // namespace vats5