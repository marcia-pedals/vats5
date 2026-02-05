#include "steps_shortest_path.h"

#include <algorithm>
#include <cassert>
#include <mutex>
#include <thread>

#include "solver/relaxed_shortest_path.h"
#include "solver/step_merge.h"

namespace vats5 {

struct FrontierEntry {
  StopId destination_stop;
  TimeSinceServiceStart arrival_time;

  // Number of destinations visited on the path so far (not including this
  // stop).
  int16_t destinations_visited;

  // Whether the full path up to now is entirely flex.
  bool is_flex;

  // Lower bound for time required to reach any remaining destination.
  int lb_to_dest = 0;
};

struct FrontierEntryComparator {
  bool operator()(const FrontierEntry& a, const FrontierEntry& b) const {
    int f_a = a.arrival_time.seconds + a.lb_to_dest;
    int f_b = b.arrival_time.seconds + b.lb_to_dest;
    if (f_a != f_b) {
      return f_a > f_b;
    }

    // Break ties by arrival time (prefer earlier actual arrivals).
    if (a.arrival_time.seconds != b.arrival_time.seconds) {
      return a.arrival_time.seconds > b.arrival_time.seconds;
    }

    // Tiebreak: prefer paths that have gone through more destinations.
    if (a.destinations_visited != b.destinations_visited) {
      return a.destinations_visited < b.destinations_visited;
    }

    // Break ties arbitrarily but consistently.
    return a.destination_stop.v > b.destination_stop.v;
  }
};

// Return index of the first fixed-schedule step departing >= `t`, or
// steps.size() if none found.
// Precondition: `steps` contains only fixed-schedule steps (no flex).
size_t FindDepartureAtOrAfter(
    std::span<const AdjacencyListStep> steps,
    std::span<const int16_t> departure_times_div10,
    TimeSinceServiceStart t
) {
  if (steps.empty()) {
    return steps.size();
  }

  // Binary search on the cache-friendly departure_times_div10 array.
  // We search for t.seconds / 10, which may undershoot due to rounding.
  int16_t target_div10 = static_cast<int16_t>(t.seconds / 10);
  auto lower_bound_it = std::lower_bound(
      departure_times_div10.begin(), departure_times_div10.end(), target_div10
  );

  if (lower_bound_it == departure_times_div10.end()) {
    return steps.size();
  }

  // Linear scan forward to fix rounding errors.
  // The binary search may have landed on a step that departs before t
  // due to integer division truncation.
  size_t idx = lower_bound_it - departure_times_div10.begin();
  while (idx < steps.size() && steps[idx].origin_time.seconds < t.seconds) {
    ++idx;
  }

  return idx;
}

// Backtrack through the search results to reconstruct the full path.
// Returns the steps in order from origin to destination.
std::vector<Step> BacktrackPath(
    const std::vector<Step>& search_result, StopId dest
) {
  std::vector<Step> path;
  Step state = search_result[dest.v];
  while (state.origin.trip != TripId::NOOP) {
    path.push_back(state);
    state = search_result[state.origin.stop.v];
  }
  std::reverse(path.begin(), path.end());

  // Advance flex steps to the latest possible moment.
  // TODO: Consider what to do if the whole path is flex:
  // - normalize to start the whole thing at 0?
  // - or maybe it is already normalized such?
  for (int i = static_cast<int>(path.size()) - 2; i >= 0; --i) {
    if (path[i].is_flex) {
      int flex_duration = path[i].DurationSeconds();
      path[i].destination.time = path[i + 1].origin.time;
      path[i].origin.time.seconds = path[i].destination.time.seconds - flex_duration;
    }
  }

  return path;
}

// Sentinel Step representing an unvisited stop.
const Step kUnvisitedStep = Step::PrimitiveScheduled(
    StopId{-1}, StopId{-1},
    TimeSinceServiceStart{std::numeric_limits<int>::max()},
    TimeSinceServiceStart{std::numeric_limits<int>::max()},
    TripId{-1}
);

const std::vector<int>* HeuristicCache::GetOrCompute(
    const std::unordered_set<StopId>& destinations
) {
  if (relaxed_distances == nullptr ||
      relaxed_distances->distance_to.empty()) {
    return nullptr;
  }

  // Create sorted key from destinations
  std::vector<StopId> cache_key(destinations.begin(), destinations.end());
  std::sort(cache_key.begin(), cache_key.end());

  auto cache_it = cache.find(cache_key);
  if (cache_it != cache.end()) {
    return &cache_it->second;
  }

  // Compute by taking min of distance_to_single_destination for each dest
  // Get num_stops from the first destination's vector
  const int num_stops = static_cast<int>(
      relaxed_distances->distance_to.begin()->second.size()
  );
  std::vector<int> computed(num_stops, std::numeric_limits<int>::max());

  for (const StopId dest : destinations) {
    auto it = relaxed_distances->distance_to.find(dest);
    if (it != relaxed_distances->distance_to.end()) {
      const std::vector<int>& single_dists = it->second;
      for (int i = 0; i < num_stops && i < static_cast<int>(single_dists.size());
           ++i) {
        computed[i] = std::min(computed[i], single_dists[i]);
      }
    }
  }

  auto [inserted_it, _] =
      cache.emplace(std::move(cache_key), std::move(computed));
  return &inserted_it->second;
}

std::vector<Step> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list,
    TimeSinceServiceStart origin_time,
    StopId origin_stop,
    const std::unordered_set<StopId>& destinations,
    int* smallest_next_departure_gap_from_flex,
    HeuristicCache* heuristic_cache,
    const std::vector<StopId>& block_paths_through
) {
  if (smallest_next_departure_gap_from_flex != nullptr) {
    *smallest_next_departure_gap_from_flex = std::numeric_limits<int>::max();
  }

  // Get initial heuristic distances
  const std::vector<int>* heuristic_distances =
      heuristic_cache != nullptr ? heuristic_cache->GetOrCompute(destinations)
                                 : nullptr;

  // Helper to get heuristic value for a stop (0 if no heuristic provided)
  auto GetHeuristic = [&](StopId stop) -> int {
    if (heuristic_distances == nullptr || stop.v < 0 ||
        stop.v >= static_cast<int>(heuristic_distances->size())) {
      return 0;
    }
    int h = (*heuristic_distances)[stop.v];
    // Treat unreachable stops (max int) as 0 heuristic to avoid overflow
    return (h == std::numeric_limits<int>::max()) ? 0 : h;
  };

  std::unordered_set<StopId> remaining_destinations = destinations;
  std::vector<bool> finalized(adjacency_list.NumStops(), false);

  // Compact priority queue storing only destination_stop and arrival_time.
  FrontierEntryComparator frontier_cmp;
  std::vector<FrontierEntry> frontier;

  // Maps stop index to the best Step we've found for reaching it.
  // Unvisited stops have kUnvisitedStep.
  std::vector<Step> best_arrival(adjacency_list.NumStops(), kUnvisitedStep);

  // Maps stop index to the number of destinations visited on the best path to
  // it. -1 means unvisited.
  std::vector<int16_t> best_destinations_visited(adjacency_list.NumStops(), -1);

  // The state at the origin stop.
  const Step initial_step = Step::PrimitiveScheduled(
      origin_stop, origin_stop, origin_time, origin_time, TripId::NOOP
  );
frontier.push_back(FrontierEntry{
      origin_stop, origin_time, /*destinations_visited=*/0,
      /*is_flex=*/true, GetHeuristic(origin_stop)
  });
  best_arrival[origin_stop.v] = initial_step;
  best_destinations_visited[origin_stop.v] = 0;

  // Thresholds at which to recompute heuristic for tighter bounds
  auto ShouldRecomputeHeuristic = [](size_t remaining) {
    return remaining == 5 || remaining == 2 || remaining == 1;
  };

  while (!frontier.empty()) {
    std::pop_heap(frontier.begin(), frontier.end(), frontier_cmp);
    const FrontierEntry current_entry = frontier.back();
    const StopId current_stop = current_entry.destination_stop;
    const TimeSinceServiceStart current_time = current_entry.arrival_time;
    frontier.pop_back();

    if (finalized[current_stop.v]) {
      continue;
    }
    finalized[current_stop.v] = true;

    if (remaining_destinations.erase(current_stop) > 0) {
      if (remaining_destinations.empty()) {
        return best_arrival;
      }

      // Check if we should recompute heuristic with fewer destinations
      if (heuristic_cache != nullptr &&
          ShouldRecomputeHeuristic(remaining_destinations.size())) {
        heuristic_distances = heuristic_cache->GetOrCompute(remaining_destinations);

        // Recompute heuristic for all entries in the frontier and reheapify
        for (FrontierEntry& entry : frontier) {
          entry.lb_to_dest = GetHeuristic(entry.destination_stop);
        }
        std::make_heap(frontier.begin(), frontier.end(), frontier_cmp);
      }
    }

    // Calculate destinations visited for paths continuing from this stop.
    const int16_t next_destinations_visited =
        current_entry.destinations_visited +
        (destinations.contains(current_stop) ? 1 : 0);

    bool blocked = false;
    for (StopId block_stop : block_paths_through) {
      if (block_stop != origin_stop && block_stop == current_stop) {
        blocked = true;
        break;
      }
    }
    if (blocked) {
      continue;
    }

    std::span<const StepGroup> step_groups =
        adjacency_list.GetGroups(current_stop);

    for (const StepGroup& step_group : step_groups) {
      const StopId next_stop = step_group.destination_stop;

      // Handle flex trip if present
      if (step_group.flex_step.has_value()) {
        const AdjacencyListStep& flex_step = *step_group.flex_step;
        if (!finalized[next_stop.v]) {
          // Calculate arrival time based on current time + duration
          TimeSinceServiceStart arrival_time{
              current_time.seconds + flex_step.FlexDurationSeconds()
          };

          // Only add if this is a better path (earlier arrival, or same
          // arrival with more destinations visited).
          const bool is_better =
              arrival_time.seconds <
                  best_arrival[next_stop.v].destination.time.seconds ||
              (arrival_time.seconds ==
                   best_arrival[next_stop.v].destination.time.seconds &&
               next_destinations_visited >
                   best_destinations_visited[next_stop.v]);
          if (is_better) {
            Step flex_step_at_now{
                StepEndpoint{current_stop, true, StepPartitionId::NONE, current_time, flex_step.origin_trip},
                StepEndpoint{next_stop, true, StepPartitionId::NONE, arrival_time, flex_step.destination_trip},
                true  // is_flex
            };

            best_arrival[next_stop.v] = flex_step_at_now;
            best_destinations_visited[next_stop.v] = next_destinations_visited;
            frontier.push_back(FrontierEntry{
                next_stop,
                arrival_time,
                next_destinations_visited,
                current_entry.is_flex,
                GetHeuristic(next_stop)
            });
            std::push_heap(frontier.begin(), frontier.end(), frontier_cmp);
          }
        }
      }

      // Regular fixed-time trip handling
      {
        std::span<const AdjacencyListStep> group_steps =
            adjacency_list.GetSteps(step_group);
        std::span<const int16_t> group_departure_times =
            adjacency_list.GetDepartureTimes(step_group);

        size_t next_step_idx = FindDepartureAtOrAfter(
            group_steps, group_departure_times, current_time
        );
        if (next_step_idx >= group_steps.size()) {
          continue;
        }

        if (current_entry.is_flex &&
            smallest_next_departure_gap_from_flex != nullptr) {
          int this_gap = std::numeric_limits<int>::max();
          size_t next_dep_gap_idx = next_step_idx;
          while (next_dep_gap_idx < group_steps.size() &&
                 group_steps[next_dep_gap_idx].origin_time <= current_time) {
            next_dep_gap_idx++;
          }
          if (next_dep_gap_idx < group_steps.size() &&
              group_steps[next_dep_gap_idx].origin_time > current_time) {
            this_gap = group_steps[next_dep_gap_idx].origin_time.seconds -
                       current_time.seconds;
          }
          if (this_gap < *smallest_next_departure_gap_from_flex) {
            *smallest_next_departure_gap_from_flex = this_gap;
          }
        }

        const AdjacencyListStep& adj_step = group_steps[next_step_idx];
        if (finalized[next_stop.v]) {
          continue;
        }

        // Only add if this is a better path (earlier arrival, or same
        // arrival with more destinations visited).
        const bool is_better =
            adj_step.destination_time.seconds <
                best_arrival[next_stop.v].destination.time.seconds ||
            (adj_step.destination_time.seconds ==
                 best_arrival[next_stop.v].destination.time.seconds &&
             next_destinations_visited >
                 best_destinations_visited[next_stop.v]);
        if (is_better) {
          Step next_step = adj_step.ToStep(current_stop, next_stop, false);
          best_arrival[next_stop.v] = next_step;
          best_destinations_visited[next_stop.v] = next_destinations_visited;
          frontier.push_back(FrontierEntry{
              next_stop,
              next_step.destination.time,
              next_destinations_visited,
              /*is_flex=*/false,
              GetHeuristic(next_stop)
          });
          std::push_heap(frontier.begin(), frontier.end(), frontier_cmp);
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
    TimeSinceServiceStart origin_time_ub,
    const RelaxedDistances* relaxed_distances,
    bool keep_through_other_destination,
    const std::vector<StopId>& block_paths_through
) {
  HeuristicCache heuristic_cache(relaxed_distances);

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
    int smallest_next_departure_gap_from_flex = std::numeric_limits<int>::max();
    const std::vector<Step> search_result = FindShortestPathsAtTime(
        adjacency_list,
        query_time,
        origin,
        destinations_to_query,
        &smallest_next_departure_gap_from_flex,
        &heuristic_cache,
        block_paths_through
    );

    // Push all results and update current origin times.
    for (const auto& dest : destinations_to_query) {
      const Step& path_state = search_result[dest.v];
      if (path_state.destination.time.seconds ==
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
      Step merged_step = ConsecutiveMergedSteps(path_steps);
      result[dest].push_back(merged_step);

      Path& full_path = full_paths[merged_step];
      full_path.merged_step = merged_step;
      full_path.steps = path_steps;

      // Check if path goes through another destination
      for (const Step& step : path_steps) {
        if (step.destination.stop != dest &&
            destinations.find(step.destination.stop) != destinations.end()) {
          through_other_destination[merged_step] = true;
          break;
        }
      }

      if (merged_step.is_flex) {
        // Pure flex step: Step forwards by
        // `smallest_next_departure_gap_from_flex` (see
        // `FindShortestPathsAtTime` doc comment).
        if (smallest_next_departure_gap_from_flex ==
            std::numeric_limits<int>::max()) {
          current_origin_time[dest] = big_time;
        } else {
          current_origin_time[dest] = query_time;
          current_origin_time[dest].seconds +=
              smallest_next_departure_gap_from_flex;
        }
      } else {
        // Non-flex step: This is the best step up until
        // merged_step.origin_time, so advance current origin time to 1 past
        // that.
        current_origin_time[dest] = merged_step.origin.time;
        current_origin_time[dest].seconds += 1;
      }
    }
  }

  std::unordered_map<StopId, std::vector<Path>> result_with_paths;

  for (const StopId dest : destinations) {
    auto& dest_result = result[dest];
    SortSteps(dest_result);
    MakeMinimalCover(dest_result);
    if (!keep_through_other_destination) {
      std::erase_if(dest_result, [&](const Step& s) {
        return through_other_destination[s];
      });
    }

    // Remove any paths whose origin is after the ub.
    if (dest_result.size() > 0 && !dest_result.back().is_flex &&
        dest_result.back().origin.time >= origin_time_ub) {
      dest_result.pop_back();
    }

    auto& dest_paths = result_with_paths[dest];
    for (const Step whole_step : dest_result) {
      Path path = full_paths[whole_step];
      // Normalize flex steps: shift origin_time back to 00:00:00
      if (path.merged_step.is_flex) {
        int32_t offset = path.merged_step.origin.time.seconds;
        path.merged_step.origin.time.seconds = 0;
        path.merged_step.destination.time.seconds -= offset;
        for (Step& step : path.steps) {
          step.origin.time.seconds -= offset;
          step.destination.time.seconds -= offset;
        }
      }
      dest_paths.push_back(path);
    }
  }

  return result_with_paths;
}

StepPathsAdjacencyList ReduceToMinimalSystemPaths(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& system_stops,
    bool keep_through_other_destination
) {
  // Convert to vector for indexed access
  std::vector<StopId> origins(system_stops.begin(), system_stops.end());
  const size_t num_origins = origins.size();

  RelaxedDistances relaxed_distances =
      ComputeRelaxedDistances(adjacency_list, system_stops);

  // Split into 6-hour chunks: 36 hours / 6 hours = 6 chunks per origin
  constexpr int kChunkSeconds = 6 * 3600;
  constexpr int kTotalSeconds = 36 * 3600;
  constexpr int kNumChunks = kTotalSeconds / kChunkSeconds;  // 6 chunks
  const size_t num_work_items = num_origins * kNumChunks;

  // Per-work-item results: map from destination to paths
  std::vector<std::unordered_map<StopId, std::vector<Path>>> per_item_results(
      num_work_items
  );

  // Work queue and progress tracking
  std::mutex mutex;
  size_t next_item = 0;
  size_t completed = 0;

  // Determine number of threads
  const unsigned int num_threads =
      std::max(1u, std::thread::hardware_concurrency());

  // std::cout << "Using " << num_threads << " threads for " << num_work_items
  //           << " work items\n";

  // Worker function that grabs unclaimed work items
  auto worker = [&]() {
    while (true) {
      // Claim the next work item
      size_t i;
      {
        std::lock_guard<std::mutex> lock(mutex);
        i = next_item;
        next_item += 1;
      }
      if (i >= num_work_items) {
        break;
      }

      const size_t origin_index = i / kNumChunks;
      const int chunk_index = i % kNumChunks;
      const StopId origin = origins[origin_index];

      std::unordered_set<StopId> destinations = system_stops;
      destinations.erase(origin);

      TimeSinceServiceStart lb{chunk_index * kChunkSeconds};
      TimeSinceServiceStart ub{(chunk_index + 1) * kChunkSeconds};

      per_item_results[i] = FindMinimalPathSet(
          adjacency_list, origin, destinations, lb, ub, &relaxed_distances,
          keep_through_other_destination
      );

      // Update progress (print only every 10 items)
      {
        std::lock_guard<std::mutex> lock(mutex);
        completed += 1;
        // if (completed % 10 == 0 || completed == num_work_items) {
        //   std::cout << completed << " / " << num_work_items << "\n";
        // }
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

  // Combine results per origin
  StepPathsAdjacencyList result;
  result.adjacent.reserve(num_origins);

  for (size_t origin_idx = 0; origin_idx < num_origins; ++origin_idx) {
    const StopId origin = origins[origin_idx];

    // Collect all paths from all chunks for this origin, grouped by destination
    std::unordered_map<StopId, std::vector<Path>> dest_to_paths;

    for (int c = 0; c < kNumChunks; ++c) {
      size_t item_idx = origin_idx * kNumChunks + c;
      for (auto& [dest, paths] : per_item_results[item_idx]) {
        for (Path& path : paths) {
          dest_to_paths[dest].push_back(std::move(path));
        }
      }
    }

    // Make the results sorted and minimal. (It might not be sorted because each
    // chunk could emit a flex step, and it might not be minimal because of
    // boundary effects).
    //
    // TODO: Aaaaa lotsa unnecessary allocations. If we made SortSteps and
    // MakeMinimalCover able to operate directly on std::vector<Path> we could
    // avoid all of these.
    std::vector<std::vector<Path>> origin_result;
    origin_result.reserve(dest_to_paths.size());
    for (const auto& [dest, paths] : dest_to_paths) {
      std::unordered_map<Step, Path> merged_step_to_path;
      std::vector<Step> steps;
      for (const auto& path : paths) {
        merged_step_to_path[path.merged_step] = path;
        steps.push_back(path.merged_step);
      }
      SortSteps(steps);
      MakeMinimalCover(steps);
      std::vector<Path> updated_paths;
      updated_paths.reserve(steps.size());
      for (const Step& s : steps) {
        updated_paths.push_back(merged_step_to_path.at(s));
      }
      origin_result.push_back(updated_paths);
    }

    result.adjacent[origin] = std::move(origin_result);
  }

  return result;
}

}  // namespace vats5
