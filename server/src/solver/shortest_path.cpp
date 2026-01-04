#include "shortest_path.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <queue>

#include "solver/step_merge.h"

namespace vats5 {

StepsAdjacencyList MakeAdjacencyList(const std::vector<Step>& steps) {
  StepsAdjacencyList adjacency_list;

  // Group steps by origin_stop and destination_stop
  std::unordered_map<StopId, std::unordered_map<StopId, std::vector<Step>>>
      groups;

  for (const Step& step : steps) {
    groups[step.origin_stop][step.destination_stop].push_back(step);
  }

  // Process each group: sort and make minimal
  for (auto& [origin_stop, destination_map] : groups) {
    std::vector<std::vector<Step>> sorted_minimal_groups;

    for (auto& [destination_stop, step_group] : destination_map) {
      SortSteps(step_group);
      MakeMinimalCover(step_group);
      if (!step_group.empty()) {
        sorted_minimal_groups.push_back(std::move(step_group));
      }
    }

    if (!sorted_minimal_groups.empty()) {
      adjacency_list.adjacent[origin_stop] = std::move(sorted_minimal_groups);
    }
  }

  return adjacency_list;
}

struct PathStateComparator {
  bool operator()(const PathState& a, const PathState& b) const {
    // Highest priority is to arrive earliest.
    if (a.step.destination_time.seconds != b.step.destination_time.seconds) {
      return a.step.destination_time.seconds > b.step.destination_time.seconds;
    }

    // Break ties arbitrarily but consistently.
    return a.step.destination_stop.v > b.step.destination_stop.v;
  }
};

// Return the first (non-flex) step departing >= `t`, if any.
// Precondition: `steps` is sorted and minimal.
std::optional<Step> FindDepartureAtOrAfter(
    const std::vector<Step>& steps, TimeSinceServiceStart t
) {
  bool has_flex_trip = !steps.empty() && steps[0].is_flex;

  // Skip flex trips in the search - they're at the beginning if present
  auto search_begin = has_flex_trip ? steps.begin() + 1 : steps.begin();

  auto lower_bound_it = std::lower_bound(
      search_begin,
      steps.end(),
      t,
      [](const Step& step, TimeSinceServiceStart target_time) {
        assert(!step.is_flex);
        return step.origin_time.seconds < target_time.seconds;
      }
  );
  if (lower_bound_it == steps.end()) {
    return std::nullopt;
  }
  return *lower_bound_it;
}

// Backtrack through the search results to reconstruct the full path.
// Returns the steps in order from origin to destination.
std::vector<Step> BacktrackPath(
    const std::unordered_map<StopId, PathState>& search_result, StopId dest
) {
  std::vector<Step> path;
  PathState state = search_result.at(dest);
  while (state.step.origin_trip != TripId::NOOP) {
    path.push_back(state.step);
    state = search_result.at(state.step.origin_stop);
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

std::unordered_map<StopId, PathState> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list,
    TimeSinceServiceStart origin_time,
    StopId origin_stop,
    const std::unordered_set<StopId>& destinations
) {
  std::unordered_map<StopId, PathState> result;
  std::unordered_set<StopId> reached_destinations;

  std::priority_queue<PathState, std::vector<PathState>, PathStateComparator>
      frontier;

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
  frontier.push(PathState{initial_step});

  while (!frontier.empty()) {
    const PathState current_state = frontier.top();
    const StopId current_stop = current_state.step.destination_stop;
    const TimeSinceServiceStart current_time =
        current_state.step.destination_time;
    frontier.pop();

    if (result.find(current_stop) != result.end()) {
      continue;
    }
    result[current_stop] = current_state;

    if (destinations.find(current_stop) != destinations.end()) {
      reached_destinations.insert(current_stop);
      if (reached_destinations.size() == destinations.size()) {
        return result;
      }
    }

    auto adj_it = adjacency_list.adjacent.find(current_stop);
    if (adj_it == adjacency_list.adjacent.end()) {
      continue;
    }

    // Determine the origin time to use for new paths.
    // If current step is NOOP (we're at the origin), new paths will start
    // fresh. Otherwise, we propagate the existing origin_time.
    const bool at_origin = current_state.step.origin_trip == TripId::NOOP;

    for (const std::vector<Step>& step_group : adj_it->second) {
      // Check if this group has a flex trip (first step is flex)
      bool has_flex_trip = !step_group.empty() && step_group[0].is_flex;

      if (has_flex_trip) {
        // For flex trips, we can take them at any time
        // The destination_time contains the duration
        const Step& flex_step =
            step_group[0];  // Should be only one step in flex group
        const StopId next_stop = flex_step.destination_stop;
        if (result.find(next_stop) == result.end()) {
          // Calculate arrival time based on current time + duration
          TimeSinceServiceStart arrival_time{
              current_time.seconds + flex_step.FlexDurationSeconds()
          };

          Step flex_step_at_now = flex_step;
          flex_step_at_now.origin_time = current_time;
          flex_step_at_now.destination_time = arrival_time;

          frontier.push(PathState{flex_step_at_now});
        }
      }

      // Regular fixed-time trip handling
      {
        const std::optional<Step> next_step_opt =
            FindDepartureAtOrAfter(step_group, current_time);
        if (!next_step_opt.has_value()) {
          continue;
        }

        const Step& next_step = *next_step_opt;
        const StopId next_stop = next_step.destination_stop;
        if (result.find(next_stop) != result.end()) {
          continue;
        }

        frontier.push(PathState{next_step});
      }
    }
  }

  return result;
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
    const std::unordered_map<StopId, PathState> search_result =
        FindShortestPathsAtTime(
            adjacency_list, query_time, origin, destinations_to_query
        );

    // Push all results and update current origin times.
    for (const auto& dest : destinations_to_query) {
      const auto r_it = search_result.find(dest);
      if (r_it == search_result.end()) {
        // There are no more steps for this destination.
        current_origin_time[dest] = big_time;
        continue;
      }

      // Backtrack to get the full path
      const PathState& path_state = r_it->second;
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
          if (adjacency_list.adjacent.contains(origin)) {
            for (const std::vector<Step>& steps_from_origin :
                 adjacency_list.adjacent.at(origin)) {
              for (const Step step_from_origin : steps_from_origin) {
                if (!step_from_origin.is_flex) {
                  origin_departure_times->push_back(step_from_origin.origin_time
                  );
                }
              }
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
  StepsAdjacencyList simplified_input = adjacency_list;

  PathsAdjacencyList result;
  result.adjacent.reserve(system_stops.size());

  for (const StopId origin : system_stops) {
    std::cout << result.adjacent.size() << " / " << system_stops.size() << "\n";
    std::unordered_set<StopId> destinations = system_stops;
    destinations.erase(origin);
    std::unordered_map<StopId, std::vector<Path>> paths =
        FindMinimalPathSet(simplified_input, origin, destinations);
    for (const StopId dest : destinations) {
      const std::vector<Path>& paths_to_dest = paths[dest];
      result.adjacent[origin].push_back(paths_to_dest);
    }

    // TODO: Think about this and decide whether it makes things faster and
    // whether it is correct. And also think about whether there are similar
    // ideas that make things even faster. Experimentally, it seems like it
    // speeds up the BART reduction from 230s to 190s, so nice but not amazing.
    // simplified_input.adjacent[origin] = result.adjacent[origin];
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
  for (const auto& [origin_stop, path_groups] : paths.adjacent) {
    std::vector<std::vector<Step>> step_groups;
    for (const auto& path_group : path_groups) {
      std::vector<Step> steps;
      for (const Path& path : path_group) {
        steps.push_back(path.merged_step);
      }
      if (!steps.empty()) {
        step_groups.push_back(std::move(steps));
      }
    }
    if (!step_groups.empty()) {
      result.adjacent[origin_stop] = std::move(step_groups);
    }
  }
  return result;
}

}  // namespace vats5