#include "shortest_path.h"

#include <algorithm>
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
      SortByOriginAndDestinationTime(step_group);
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

std::unordered_map<StopId, Step> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list, 
    TimeSinceServiceStart time, 
    StopId origin,
    std::unordered_set<StopId> destinations
) {
  std::unordered_map<StopId, Step> result;
  std::unordered_map<StopId, Step> best_step;
  
  using QueueEntry = std::pair<TimeSinceServiceStart, StopId>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> frontier;
  
  // TODO: There's a bit of weirdness about the origin because you can't represent a "do nothing
  // stay here" step. I wonder if it causes any problems. Try to find something more elegant.
  frontier.push({time, origin});
  
  while (!frontier.empty()) {
    auto [current_time, current_stop] = frontier.top();
    frontier.pop();

    auto current_step_it = best_step.find(current_stop);
    if (current_step_it != best_step.end() && current_time > current_step_it->second.destination_time) {
      continue;
    }

    // Check if current stop is a destination and we have a way to reach it
    if (destinations.find(current_stop) != destinations.end()) {
      if (current_step_it != best_step.end()) {
        result[current_stop] = current_step_it->second;
      } else if (current_stop == origin) {
        // Special case for origin being a destination - create a "stay here" step
        // TODO: Kinda hacky. Maybe just forbid this.
        result[current_stop] = Step{
          current_stop, current_stop, time, time, TripId{-1}, TripId{-1}
        };
      }
      if (result.size() == destinations.size()) {
        return result;
      }
    }
    
    auto adj_it = adjacency_list.adjacent.find(current_stop);
    if (adj_it == adjacency_list.adjacent.end()) {
      continue;
    }
    
    for (const std::vector<Step>& step_group : adj_it->second) {
      auto lower_bound_it = std::lower_bound(
        step_group.begin(), step_group.end(), current_time,
        [](const Step& step, TimeSinceServiceStart target_time) {
          return step.origin_time.seconds < target_time.seconds;
        });
      if (lower_bound_it == step_group.end()) {
        continue;
      }
 
      const Step& next_step = *lower_bound_it;
      auto existing_step_it = best_step.find(next_step.destination_stop);
      if (existing_step_it == best_step.end() || existing_step_it->second.destination_time > next_step.destination_time) {
        Step new_step;
        if (current_step_it == best_step.end()) {
          // Direct step from origin
          new_step = next_step;
        } else {
          // Chained step
          const Step& current_step = current_step_it->second;
          new_step = Step{
            current_step.origin_stop,
            next_step.destination_stop,
            current_step.origin_time,
            next_step.destination_time,
            current_step.origin_trip,
            next_step.destination_trip,
          };
        }
        best_step[next_step.destination_stop] = new_step;
        frontier.push({new_step.destination_time, next_step.destination_stop});
      }
    }
  }
  
  return result;
}

std::vector<std::vector<Step>> FindShortestPaths(
    const StepsAdjacencyList& adjacency_list, StopId origin,
    std::vector<StopId> destinations) {
  // TODO
  return {};
}

}  // namespace vats5