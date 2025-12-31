#include "shortest_path.h"

#include <algorithm>
#include <cassert>
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
  bool operator()(const PathState& a_state, const PathState& b_state) const {
    const Step& a = a_state.whole_step;
    const Step& b = b_state.whole_step;

    // Highest priority is to arrive earliest.
    if (a.destination_time.seconds != b.destination_time.seconds) {
      return a.destination_time.seconds > b.destination_time.seconds;
    }

    // Next priority is to depart latest.
    if (a.origin_time.seconds != b.origin_time.seconds) {
      return a.origin_time.seconds < b.origin_time.seconds;
    }

    // Break ties arbitrarily but consistently.
    return a.destination_stop.v > b.destination_stop.v;
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
  const PathState initial_state(initial_step, initial_step);
  frontier.emplace(initial_state);

  while (!frontier.empty()) {
    const PathState current_state = frontier.top();
    const StopId current_stop = current_state.whole_step.destination_stop;
    const TimeSinceServiceStart current_time =
        current_state.whole_step.destination_time;
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

          if (current_state.whole_step.origin_trip == TripId::NOOP) {
            // If our current step is just staying in place, start the path with
            // flex step
            Step flex_path_step{
                flex_step.origin_stop,
                flex_step.destination_stop,
                current_time,  // Start immediately
                arrival_time,  // Arrive after duration
                flex_step.origin_trip,
                flex_step.destination_trip,
                true  // is_flex
            };
            frontier.push(PathState{flex_path_step, flex_step_at_now});
          } else {
            // Combine current step with flex step
            Step combined_flex_step{
                current_state.whole_step.origin_stop,
                flex_step.destination_stop,
                current_state.whole_step.origin_time,
                arrival_time,  // Arrive after walking duration
                current_state.whole_step.origin_trip,
                flex_step.destination_trip,
                current_state.whole_step.is_flex  // is_flex
            };
            frontier.push(PathState{combined_flex_step, flex_step_at_now});
          }
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

        if (current_state.whole_step.origin_trip == TripId::NOOP) {
          // If our current step is just staying in place, then "start" the path
          // outwards with the whole next step.
          frontier.push(PathState{next_step, next_step});
        } else {
          // If we have a current step that involves actually moving around,
          // combine the "starting" part of the current step with the
          // "finishing" part of the next step.

          TimeSinceServiceStart new_step_origin_time =
              current_state.whole_step.origin_time;
          if (current_state.whole_step.is_flex) {
            // If the current step is flex, we may be able to wait longer before
            // starting and still catch this connection.
            new_step_origin_time.seconds +=
                next_step.origin_time.seconds -
                current_state.whole_step.destination_time.seconds;
          }

          frontier.push(PathState{
              Step{
                  current_state.whole_step.origin_stop,
                  next_step.destination_stop,
                  new_step_origin_time,
                  next_step.destination_time,
                  current_state.whole_step.origin_trip,
                  next_step.destination_trip,
                  false  // is_flex
              },
              next_step
          });
        }
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
    const std::unordered_map<StopId, PathState> steps = FindShortestPathsAtTime(
        adjacency_list, query_time, origin, destinations_to_query
    );

    // Push all results and update current origin times.
    for (const auto& dest : destinations_to_query) {
      const auto r_it = steps.find(dest);
      if (r_it == steps.end()) {
        // There are no more steps for this destination.
        current_origin_time[dest] = big_time;
        continue;
      }
      const PathState r = r_it->second;
      result[dest].push_back(r.whole_step);

      Path& full_path = full_paths[r.whole_step];
      full_path.merged_step = r.whole_step;
      PathState backtracking_state = r;
      while (backtracking_state.current_step.origin_trip != TripId::NOOP) {
        full_path.steps.push_back(backtracking_state.current_step);
        backtracking_state =
            steps.at(backtracking_state.current_step.origin_stop);
        if (destinations.find(backtracking_state.current_step.destination_stop
            ) != destinations.end()) {
          through_other_destination[r.whole_step] = true;
        }
      }
      std::reverse(full_path.steps.begin(), full_path.steps.end());

      if (r.whole_step.is_flex) {
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
        // Non-flex step: This is the best step up until `r.origin_time`, so
        // advance current origin time to 1 past that.
        current_origin_time[dest] = r.whole_step.origin_time;
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

StepsAdjacencyList ReduceToMinimalSystemSteps(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& system_stops
) {
  StepsAdjacencyList simplified_input = adjacency_list;

  StepsAdjacencyList result;
  result.adjacent.reserve(system_stops.size());

  for (const StopId origin : system_stops) {
    std::cout << result.adjacent.size() << " / " << system_stops.size() << "\n";
    std::unordered_set<StopId> destinations = system_stops;
    destinations.erase(origin);
    std::unordered_map<StopId, std::vector<Path>> paths =
        FindMinimalPathSet(simplified_input, origin, destinations);
    for (const StopId dest : destinations) {
      const std::vector<Path>& paths_to_dest = paths[dest];
      if (paths_to_dest.size() > 0) {
        std::vector<Step> steps_to_dest;
        steps_to_dest.reserve(paths_to_dest.size());
        for (const Path& path : paths_to_dest) {
          steps_to_dest.push_back(path.merged_step);
        }
        result.adjacent[origin].push_back(steps_to_dest);
      }
    }

    // TODO: Think about this and decide whether it makes things faster and
    // whether it is correct. And also think about whether there are similar
    // ideas that make things even faster. Experimentally, it seems like it
    // speeds up the BART reduction from 230s to 190s, so nice but not amazing.
    // simplified_input.adjacent[origin] = result.adjacent[origin];
  }

  return result;
}

}  // namespace vats5