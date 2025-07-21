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

static std::vector<Step> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list, 
    TimeSinceServiceStart time, 
    StopId origin,
    std::vector<StopId> destinations
) {
  std::unordered_map<StopId, TimeSinceServiceStart> best_time;
  std::unordered_map<StopId, Step> best_step;
  
  using QueueEntry = std::pair<TimeSinceServiceStart, StopId>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> frontier;
  
  best_time[origin] = time;
  frontier.push({time, origin});
  
  while (!frontier.empty()) {
    auto [current_time, current_stop] = frontier.top();
    frontier.pop();
    
    if (best_time[current_stop].seconds < current_time.seconds) {
      continue;
    }
    
    auto adj_it = adjacency_list.adjacent.find(current_stop);
    if (adj_it == adjacency_list.adjacent.end()) continue;
    
    for (const std::vector<Step>& step_group : adj_it->second) {
      auto lower_bound_it = std::lower_bound(
        step_group.begin(), step_group.end(), current_time,
        [](const Step& step, TimeSinceServiceStart target_time) {
          return step.origin_time.seconds < target_time.seconds;
        });
      
      if (lower_bound_it != step_group.end()) {
        const Step& next_step = *lower_bound_it;
        StopId next_stop = next_step.destination_stop;
        TimeSinceServiceStart next_time = next_step.destination_time;
        
        auto existing_time_it = best_time.find(next_stop);
        if (existing_time_it == best_time.end() || 
            next_time.seconds < existing_time_it->second.seconds) {
          best_time[next_stop] = next_time;
          best_step[next_stop] = next_step;
          frontier.push({next_time, next_stop});
        }
      }
    }
  }
  
  std::vector<Step> result;
  for (StopId dest : destinations) {
    auto step_it = best_step.find(dest);
    if (step_it != best_step.end()) {
      result.push_back(step_it->second);
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