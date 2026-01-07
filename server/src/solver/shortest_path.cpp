#include "shortest_path.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <mutex>
#include <queue>
#include <thread>

#include "solver/step_merge.h"

namespace vats5 {

// Helper to convert Step to AdjacencyListStep (drops origin_stop and
// destination_stop).
inline AdjacencyListStep ToAdjacencyListStep(const Step& step) {
  return AdjacencyListStep{
      step.origin_time,
      step.destination_time,
      step.origin_trip,
      step.destination_trip
  };
}

// Helper to convert AdjacencyListStep back to Step.
inline Step FromAdjacencyListStep(
    const AdjacencyListStep& als,
    StopId origin_stop,
    StopId destination_stop,
    bool is_flex
) {
  return Step{
      origin_stop,
      destination_stop,
      als.origin_time,
      als.destination_time,
      als.origin_trip,
      als.destination_trip,
      is_flex
  };
}

// Helper to convert a flex Step to AdjacencyListFlexStep.
inline AdjacencyListFlexStep ToAdjacencyListFlexStep(const Step& step) {
  return AdjacencyListFlexStep{
      step.destination_time.seconds - step.origin_time.seconds,
      step.origin_trip,
      step.destination_trip
  };
}

// Temporary step group with vectors for building the adjacency list.
struct TempStepGroup {
  StopId destination_stop;
  AdjacencyListFlexStep
      flex_step;  // origin_trip == TripId::NOOP means no flex step
  std::vector<AdjacencyListStep> steps;
  std::vector<int16_t> departure_times_div10;

  bool has_flex_step() const { return flex_step.origin_trip != TripId::NOOP; }
};

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

  // Build step groups per stop in a temporary structure
  std::vector<std::vector<TempStepGroup>> temp_adjacent(max_stop_id + 1);

  // Process each group: sort and make minimal
  for (auto& [origin_stop, destination_map] : groups) {
    std::vector<TempStepGroup> sorted_minimal_groups;

    for (auto& [destination_stop, step_vec] : destination_map) {
      SortSteps(step_vec);
      MakeMinimalCover(step_vec);
      if (!step_vec.empty()) {
        TempStepGroup sg;
        sg.destination_stop = destination_stop;
        // Initialize flex_step with NOOP sentinel (no flex step by default)
        sg.flex_step.origin_trip = TripId::NOOP;

        // Extract flex step if present (it's always first after sorting)
        size_t fixed_start = 0;
        if (step_vec[0].is_flex) {
          sg.flex_step = ToAdjacencyListFlexStep(step_vec[0]);
          fixed_start = 1;
        }

        // Copy fixed-schedule steps
        sg.steps.reserve(step_vec.size() - fixed_start);
        sg.departure_times_div10.reserve(step_vec.size() - fixed_start);
        for (size_t i = fixed_start; i < step_vec.size(); ++i) {
          sg.steps.push_back(ToAdjacencyListStep(step_vec[i]));
          sg.departure_times_div10.push_back(
              static_cast<int16_t>(step_vec[i].origin_time.seconds / 10)
          );
        }

        sorted_minimal_groups.push_back(std::move(sg));
      }
    }

    if (!sorted_minimal_groups.empty()) {
      temp_adjacent[origin_stop.v] = std::move(sorted_minimal_groups);
    }
  }

  // Convert to CSR format
  size_t num_stops = temp_adjacent.size();
  adjacency_list.offsets.resize(num_stops + 1);
  adjacency_list.offsets[0] = 0;

  // First pass: compute offsets for step groups and count total steps
  size_t total_steps = 0;
  for (size_t i = 0; i < num_stops; ++i) {
    adjacency_list.offsets[i + 1] =
        adjacency_list.offsets[i] +
        static_cast<uint32_t>(temp_adjacent[i].size());
    for (const auto& tsg : temp_adjacent[i]) {
      total_steps += tsg.steps.size();
    }
  }

  // Reserve space
  adjacency_list.step_groups.reserve(adjacency_list.offsets[num_stops]);
  adjacency_list.all_steps.reserve(total_steps);
  adjacency_list.all_departure_times_div10.reserve(total_steps);

  // Second pass: flatten into step_groups and all_steps
  for (size_t i = 0; i < num_stops; ++i) {
    for (auto& tsg : temp_adjacent[i]) {
      StepGroup sg;
      sg.destination_stop = tsg.destination_stop;
      sg.flex_step = std::move(tsg.flex_step);
      sg.steps_begin = static_cast<uint32_t>(adjacency_list.all_steps.size());

      // Append steps to flat arrays
      for (size_t j = 0; j < tsg.steps.size(); ++j) {
        adjacency_list.all_steps.push_back(std::move(tsg.steps[j]));
        adjacency_list.all_departure_times_div10.push_back(
            tsg.departure_times_div10[j]
        );
      }

      adjacency_list.step_groups.push_back(std::move(sg));
    }
  }

  // Collect statistics
  std::vector<size_t> step_groups_per_stop;
  std::vector<size_t> flex_steps_per_group;
  std::vector<size_t> non_flex_steps_per_group;

  for (size_t i = 0; i < num_stops; ++i) {
    auto step_groups = adjacency_list.step_groups_for_stop(i);
    if (!step_groups.empty()) {
      step_groups_per_stop.push_back(step_groups.size());
      for (const StepGroup& sg : step_groups) {
        flex_steps_per_group.push_back(sg.has_flex_step() ? 1 : 0);
        non_flex_steps_per_group.push_back(
            adjacency_list.steps_end_for_group(sg) - sg.steps_begin
        );
      }
    }
  }

  // Helper to compute percentile (p10, p50, p90)
  auto percentile = [](std::vector<size_t>& v, double p) -> double {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t idx = static_cast<size_t>(p * (v.size() - 1));
    return static_cast<double>(v[idx]);
  };

  // Helper to compute average
  auto average = [](const std::vector<size_t>& v) -> double {
    if (v.empty()) return 0.0;
    size_t sum = 0;
    for (size_t x : v) sum += x;
    return static_cast<double>(sum) / v.size();
  };

  std::cout << "MakeAdjacencyList stats:\n";
  std::cout << "  Stops: " << step_groups_per_stop.size() << "\n";
  std::cout << "  StepGroups/stop - avg: " << average(step_groups_per_stop)
            << ", p10: " << percentile(step_groups_per_stop, 0.1)
            << ", p50: " << percentile(step_groups_per_stop, 0.5)
            << ", p90: " << percentile(step_groups_per_stop, 0.9) << "\n";
  std::cout << "  Flex steps/StepGroup - avg: " << average(flex_steps_per_group)
            << ", p10: " << percentile(flex_steps_per_group, 0.1)
            << ", p50: " << percentile(flex_steps_per_group, 0.5)
            << ", p90: " << percentile(flex_steps_per_group, 0.9) << "\n";
  std::cout << "  Non-flex steps/StepGroup - avg: "
            << average(non_flex_steps_per_group)
            << ", p10: " << percentile(non_flex_steps_per_group, 0.1)
            << ", p50: " << percentile(non_flex_steps_per_group, 0.5)
            << ", p90: " << percentile(non_flex_steps_per_group, 0.9) << "\n";

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
const AdjacencyListStep* FindDepartureAtOrAfter(
    const StepsAdjacencyList& adjacency_list,
    const StepGroup& step_group,
    TimeSinceServiceStart t
) {
  auto steps = adjacency_list.steps_for_group(step_group);
  auto departure_times_div10 =
      adjacency_list.departure_times_for_group(step_group);

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
  const size_t num_stops = adjacency_list.num_stops();
  std::vector<bool> finalized(num_stops, false);

  // Compact priority queue storing only destination_stop and arrival_time.
  std::priority_queue<
      FrontierEntry,
      std::vector<FrontierEntry>,
      FrontierEntryComparator>
      frontier;

  // Maps stop index to the best PathState we've found for reaching it.
  // Unvisited stops have kUnvisitedPathState.
  std::vector<PathState> best_arrival(num_stops, kUnvisitedPathState);

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

    auto step_groups = adjacency_list.step_groups_for_stop(current_stop.v);
    if (step_groups.empty()) {
      continue;
    }

    for (const StepGroup& step_group : step_groups) {
      const StopId next_stop = step_group.destination_stop;

      // Handle flex trip if present
      if (step_group.has_flex_step()) {
        const AdjacencyListFlexStep& flex_step = step_group.flex_step;
        if (!finalized[next_stop.v]) {
          // Calculate arrival time based on current time + duration
          TimeSinceServiceStart arrival_time{
              current_time.seconds + flex_step.duration_seconds
          };

          // Only add if this is a better arrival time
          if (arrival_time.seconds <
              best_arrival[next_stop.v].step.destination_time.seconds) {
            Step flex_step_at_now{
                current_stop,  // origin_stop is current_stop
                next_stop,
                current_time,
                arrival_time,
                flex_step.origin_trip,
                flex_step.destination_trip,
                true  // is_flex
            };

            best_arrival[next_stop.v] = PathState{flex_step_at_now};
            frontier.push(FrontierEntry{next_stop, arrival_time});
          }
        }
      }

      // Regular fixed-time trip handling
      {
        const AdjacencyListStep* next_als_ptr =
            FindDepartureAtOrAfter(adjacency_list, step_group, current_time);
        if (next_als_ptr == nullptr) {
          continue;
        }

        if (finalized[next_stop.v]) {
          continue;
        }

        // Only add if this is a better arrival time
        if (next_als_ptr->destination_time.seconds <
            best_arrival[next_stop.v].step.destination_time.seconds) {
          Step next_step = FromAdjacencyListStep(
              *next_als_ptr, current_stop, next_stop, false
          );
          best_arrival[next_stop.v] = PathState{next_step};
          frontier.push(FrontierEntry{next_stop, next_als_ptr->destination_time}
          );
        }
      }
    }
  }

  return best_arrival;
}

std::unordered_map<StopId, std::vector<Path>> FindMinimalPathSet(
    const StepsAdjacencyList& adjacency_list,
    StopId origin,
    const std::unordered_set<StopId>& destinations,
    TimeSinceServiceStart origin_time_lb,
    TimeSinceServiceStart origin_time_ub
) {
  const TimeSinceServiceStart big_time{origin_time_ub.seconds * 10};

  std::unordered_map<StopId, std::vector<Step>> result;
  std::unordered_map<StopId, TimeSinceServiceStart> current_origin_time;
  for (const StopId dest : destinations) {
    current_origin_time[dest] = origin_time_lb;
  }

  // A mapping from a Step in the result to its full Path.
  std::unordered_map<Step, Path> full_paths;

  // Whether the full path represented by step goes through a different element
  // of `destinations` before reaching its ultimate destination.
  std::unordered_map<Step, bool> through_other_destination;

  // Sorted vector of all departure times from origin. This is optional because
  // usually we don't need it. We compute it on demand.
  std::optional<std::vector<TimeSinceServiceStart>> origin_departure_times;

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
        // Pure flex path (started flex and stayed flex throughout):
        // Flex step: The origin time is exactly the query time, but we know
        // that this is still gonna be the best step up until the next departure
        // from origin, so we can advance the current origin time to that.

        if (!origin_departure_times.has_value()) {
          // Need to compute origin departure times.
          origin_departure_times.emplace(std::vector<TimeSinceServiceStart>());
          auto origin_step_groups =
              adjacency_list.step_groups_for_stop(origin.v);
          for (const StepGroup& step_group_from_origin : origin_step_groups) {
            // All steps in step_group are fixed-schedule (no flex)
            auto steps = adjacency_list.steps_for_group(step_group_from_origin);
            for (const AdjacencyListStep& step_from_origin : steps) {
              origin_departure_times->push_back(step_from_origin.origin_time);
            }
          }

          // TODO: Since we're merging sorted lists, we could do that faster
          // than sorting at the end.
          std::sort(
              origin_departure_times->begin(), origin_departure_times->end()
          );
        }

        TimeSinceServiceStart want_departure_at_or_after = query_time;
        want_departure_at_or_after.seconds += 1;
        const auto next_departure_it = std::lower_bound(
            origin_departure_times->begin(),
            origin_departure_times->end(),
            want_departure_at_or_after
        );
        if (next_departure_it != origin_departure_times->end()) {
          current_origin_time[dest] = *next_departure_it;
        } else {
          current_origin_time[dest] = big_time;
        }
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

  // Time range parameters
  constexpr int32_t range_size_seconds = 6 * 3600;   // 6 hours
  constexpr int32_t total_time_seconds = 36 * 3600;  // 36 hours total
  constexpr size_t num_ranges = (total_time_seconds + range_size_seconds - 1) /
                                range_size_seconds;  // 6 ranges

  // Work items: (origin_index, range_index) pairs
  const size_t num_work_items = num_origins * num_ranges;

  // Per-work-item results: [origin_index][range_index] -> map of dest -> paths
  std::vector<std::vector<std::unordered_map<StopId, std::vector<Path>>>>
      per_work_item_results(
          num_origins,
          std::vector<std::unordered_map<StopId, std::vector<Path>>>(num_ranges)
      );

  // Work queue: atomic index for next work item to process
  std::atomic<size_t> next_work_item{0};

  // Progress tracking
  std::atomic<size_t> completed{0};
  std::mutex progress_mutex;

  // Determine number of threads
  const unsigned int num_threads =
      std::max(1u, std::thread::hardware_concurrency());

  // Thread finish time tracking
  std::vector<std::chrono::steady_clock::time_point> thread_finish_times(
      num_threads
  );
  std::mutex finish_mutex;

  // Worker function that grabs unclaimed work items
  auto worker = [&](unsigned int thread_id) {
    while (true) {
      // Atomically claim the next work item
      size_t work_idx = next_work_item.fetch_add(1);
      if (work_idx >= num_work_items) {
        break;
      }

      // Decode work item into origin and range indices
      size_t origin_idx = work_idx / num_ranges;
      size_t range_idx = work_idx % num_ranges;

      const StopId origin = origins[origin_idx];

      std::unordered_set<StopId> destinations = system_stops;
      destinations.erase(origin);

      // Compute time bounds for this range
      TimeSinceServiceStart time_lb{
          static_cast<int32_t>(range_idx * range_size_seconds)
      };
      TimeSinceServiceStart time_ub{
          static_cast<int32_t>((range_idx + 1) * range_size_seconds)
      };
      if (time_ub.seconds > total_time_seconds) {
        time_ub.seconds = total_time_seconds;
      }

      std::unordered_map<StopId, std::vector<Path>> paths = FindMinimalPathSet(
          adjacency_list, origin, destinations, time_lb, time_ub
      );

      per_work_item_results[origin_idx][range_idx] = std::move(paths);

      // Update progress
      size_t done = ++completed;
      {
        std::lock_guard<std::mutex> lock(progress_mutex);
        std::cout << done << " / " << num_work_items << "\n";
      }
    }
    // Record when this thread finished
    {
      std::lock_guard<std::mutex> lock(finish_mutex);
      thread_finish_times[thread_id] = std::chrono::steady_clock::now();
    }
  };

  // Launch threads
  std::vector<std::thread> threads;
  threads.reserve(num_threads);
  for (unsigned int t = 0; t < num_threads; ++t) {
    threads.emplace_back(worker, t);
  }

  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }

  // Compute and report thread wait time
  std::sort(thread_finish_times.begin(), thread_finish_times.end());
  auto first_finish = thread_finish_times.front();
  auto second_last_finish = thread_finish_times[thread_finish_times.size() - 2];
  auto last_finish = thread_finish_times.back();
  auto wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                     last_finish - first_finish
  )
                     .count();
  auto last_task_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          last_finish - second_last_finish
  )
                          .count();
  std::cout << "Thread wait time (first to last finish): " << wait_ms
            << " ms\n";
  std::cout << "Thread wait time (2nd last to last finish): " << last_task_ms
            << " ms\n";

  // Combine results: merge all ranges for each origin
  PathsAdjacencyList result;
  result.adjacent.reserve(num_origins);

  for (size_t origin_idx = 0; origin_idx < num_origins; ++origin_idx) {
    const StopId origin = origins[origin_idx];
    std::unordered_set<StopId> destinations = system_stops;
    destinations.erase(origin);

    std::vector<std::vector<Path>> origin_result;
    origin_result.reserve(destinations.size());

    for (const StopId dest : destinations) {
      std::vector<Path> combined_paths;
      for (size_t range_idx = 0; range_idx < num_ranges; ++range_idx) {
        auto& range_paths = per_work_item_results[origin_idx][range_idx][dest];
        combined_paths.insert(
            combined_paths.end(),
            std::make_move_iterator(range_paths.begin()),
            std::make_move_iterator(range_paths.end())
        );
      }

      // Move flex steps to the front and keep only one
      auto flex_end = std::stable_partition(
          combined_paths.begin(),
          combined_paths.end(),
          [](const Path& p) { return p.merged_step.is_flex; }
      );
      if (flex_end - combined_paths.begin() > 1) {
        combined_paths.erase(combined_paths.begin() + 1, flex_end);
      }

      origin_result.push_back(std::move(combined_paths));
    }

    result.adjacent[origin] = std::move(origin_result);
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

  // First pass: find max_stop_id
  for (const auto& [origin_stop, path_groups] : paths.adjacent) {
    max_stop_id = std::max(max_stop_id, origin_stop.v);
    for (const auto& path_group : path_groups) {
      for (const Path& path : path_group) {
        max_stop_id =
            std::max(max_stop_id, path.merged_step.destination_stop.v);
      }
    }
  }

  // Build temporary vector of TempStepGroups
  std::vector<std::vector<TempStepGroup>> temp_adjacent(max_stop_id + 1);

  // Second pass: populate step groups
  for (const auto& [origin_stop, path_groups] : paths.adjacent) {
    std::vector<TempStepGroup> step_groups;
    for (const auto& path_group : path_groups) {
      TempStepGroup tsg;
      // Initialize flex_step with NOOP sentinel (no flex step by default)
      tsg.flex_step.origin_trip = TripId::NOOP;
      // All paths in a path_group have the same destination
      if (!path_group.empty()) {
        tsg.destination_stop = path_group[0].merged_step.destination_stop;
      }
      for (const Path& path : path_group) {
        if (path.merged_step.is_flex) {
          tsg.flex_step = ToAdjacencyListFlexStep(path.merged_step);
        } else {
          tsg.steps.push_back(ToAdjacencyListStep(path.merged_step));
          tsg.departure_times_div10.push_back(
              static_cast<int16_t>(path.merged_step.origin_time.seconds / 10)
          );
        }
      }
      if (tsg.has_flex_step() || !tsg.steps.empty()) {
        step_groups.push_back(std::move(tsg));
      }
    }
    if (!step_groups.empty()) {
      temp_adjacent[origin_stop.v] = std::move(step_groups);
    }
  }

  // Convert to CSR format
  size_t num_stops = temp_adjacent.size();
  result.offsets.resize(num_stops + 1);
  result.offsets[0] = 0;

  // First pass: compute offsets for step groups and count total steps
  size_t total_steps = 0;
  for (size_t i = 0; i < num_stops; ++i) {
    result.offsets[i + 1] =
        result.offsets[i] + static_cast<uint32_t>(temp_adjacent[i].size());
    for (const auto& tsg : temp_adjacent[i]) {
      total_steps += tsg.steps.size();
    }
  }

  // Reserve space
  result.step_groups.reserve(result.offsets[num_stops]);
  result.all_steps.reserve(total_steps);
  result.all_departure_times_div10.reserve(total_steps);

  // Second pass: flatten into step_groups and all_steps
  for (size_t i = 0; i < num_stops; ++i) {
    for (auto& tsg : temp_adjacent[i]) {
      StepGroup sg;
      sg.destination_stop = tsg.destination_stop;
      sg.flex_step = std::move(tsg.flex_step);
      sg.steps_begin = static_cast<uint32_t>(result.all_steps.size());

      // Append steps to flat arrays
      for (size_t j = 0; j < tsg.steps.size(); ++j) {
        result.all_steps.push_back(std::move(tsg.steps[j]));
        result.all_departure_times_div10.push_back(tsg.departure_times_div10[j]
        );
      }

      result.step_groups.push_back(std::move(sg));
    }
  }

  return result;
}

}  // namespace vats5