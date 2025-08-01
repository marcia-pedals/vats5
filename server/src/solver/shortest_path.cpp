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

struct StepOriginTimeComparator {
  bool operator()(const Step& a, const Step& b) const {
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

std::unordered_map<StopId, Step> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list,
    TimeSinceServiceStart origin_time,
    StopId origin_stop,
    const std::unordered_set<StopId>& destinations
) {
  std::unordered_map<StopId, Step> result;
  std::unordered_set<StopId> visited;

  std::priority_queue<Step, std::vector<Step>, StepOriginTimeComparator>
      frontier;

  frontier.emplace(Step{
      origin_stop,
      origin_stop,
      origin_time,
      origin_time,
      TripId::NOOP,
      TripId::NOOP,
      false  // is_flex
  });

  while (!frontier.empty()) {
    const Step current_step = frontier.top();
    const StopId current_stop = current_step.destination_stop;
    const TimeSinceServiceStart current_time = current_step.destination_time;
    frontier.pop();

    if (visited.find(current_stop) != visited.end()) {
      continue;
    }
    visited.insert(current_stop);

    if (destinations.find(current_stop) != destinations.end()) {
      result[current_stop] = current_step;
      if (result.size() == destinations.size()) {
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
        if (visited.find(next_stop) == visited.end()) {
          // Calculate arrival time based on current time + duration
          TimeSinceServiceStart arrival_time{
              current_time.seconds + flex_step.FlexDurationSeconds()
          };

          if (current_step.origin_trip == TripId::NOOP) {
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
            frontier.push(flex_path_step);
          } else {
            // Combine current step with flex step
            Step combined_flex_step{
                current_step.origin_stop,
                flex_step.destination_stop,
                current_step.origin_time,
                arrival_time,  // Arrive after walking duration
                current_step.origin_trip,
                flex_step.destination_trip,
                current_step.is_flex  // is_flex
            };
            frontier.push(combined_flex_step);
          }
        }
      }

      // Regular fixed-time trip handling
      {
        // Skip flex trips in the search - they're at the beginning if present
        auto search_begin =
            has_flex_trip ? step_group.begin() + 1 : step_group.begin();

        auto lower_bound_it = std::lower_bound(
            search_begin,
            step_group.end(),
            current_time,
            [](const Step& step, TimeSinceServiceStart target_time) {
              assert(!step.is_flex);
              return step.origin_time.seconds < target_time.seconds;
            }
        );
        if (lower_bound_it == step_group.end()) {
          continue;
        }

        const Step& next_step = *lower_bound_it;
        const StopId next_stop = next_step.destination_stop;
        if (visited.find(next_stop) != visited.end()) {
          continue;
        }

        if (current_step.origin_trip == TripId::NOOP) {
          // If our current step is just staying in place, then "start" the path
          // outwards with the whole next step.
          frontier.push(next_step);
        } else {
          // If we have a current step that involves actually moving around,
          // combine the "starting" part of the current step with the
          // "finishing" part of the next step.
          frontier.push(Step{
              current_step.origin_stop,
              next_step.destination_stop,
              current_step.origin_time,
              next_step.destination_time,
              current_step.origin_trip,
              next_step.destination_trip,
              false  // is_flex
          });
        }
      }
    }
  }

  return result;
}

std::vector<Step> FindMinimalPathSet(
    const StepsAdjacencyList& adjacency_list, StopId origin, StopId destination
) {
  // Invariant: Sorted by origin_time ascending throughout.
  std::vector<Step> result;
  result.reserve(24);

  std::unordered_set<StopId> dests;
  dests.insert(destination);

  // First query with origin times evenly spaced within the range of origin
  // times.
  for (auto origin_time = TimeSinceServiceStart{0};
       origin_time.seconds <= 24 * 3600;
       origin_time.seconds += 3600) {
    auto r =
        FindShortestPathsAtTime(adjacency_list, origin_time, origin, dests);
    auto r_it = r.find(destination);
    if (r_it == r.end()) {
      continue;
    }

    // Insert the step while maintaining the sortedness invariant.
    const Step& new_step = r_it->second;
    if (result.empty() ||
        new_step.origin_time.seconds >= result.back().origin_time.seconds) {
      // Common case: new step belongs at the end
      result.push_back(new_step);
    } else {
      // Find correct insertion position
      auto insert_pos = std::upper_bound(
          result.begin(),
          result.end(),
          new_step,
          [](const Step& a, const Step& b) {
            return a.origin_time.seconds < b.origin_time.seconds;
          }
      );
      result.insert(insert_pos, new_step);
    }
  }

  // TODO: Finish implementing.

  return result;
}

// std::vector<std::vector<Step>> FindShortestPaths(
//     const StepsAdjacencyList& adjacency_list,
//     StopId origin_stop,
//     std::vector<StopId> destinations
// ) {
//   // TODO
//   return {};
// }

}  // namespace vats5