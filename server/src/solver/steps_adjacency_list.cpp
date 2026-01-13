#include "steps_adjacency_list.h"

#include <algorithm>
#include <unordered_map>

#include "solver/step_merge.h"

namespace vats5 {

// Temporary structure for building StepGroups before flattening
struct TempStepGroup {
  StopId destination_stop;
  std::optional<AdjacencyListStep> flex_step;
  std::vector<AdjacencyListStep> fixed_steps;
};

StepsAdjacencyList MakeAdjacencyList(const std::vector<Step>& steps) {
  // Group steps by origin_stop and destination_stop
  std::unordered_map<StopId, std::unordered_map<StopId, std::vector<Step>>>
      temp_groups;

  int max_stop_id = 0;
  for (const Step& step : steps) {
    temp_groups[step.origin_stop][step.destination_stop].push_back(step);
    max_stop_id = std::max(max_stop_id, step.origin_stop.v);
    max_stop_id = std::max(max_stop_id, step.destination_stop.v);
  }

  // Build per-origin TempStepGroups
  std::unordered_map<int, std::vector<TempStepGroup>> origin_to_temp_groups;

  for (auto& [origin_stop, destination_map] : temp_groups) {
    std::vector<TempStepGroup> sorted_minimal_groups;

    for (auto& [destination_stop, step_vec] : destination_map) {
      SortSteps(step_vec);
      MakeMinimalCover(step_vec);
      if (!step_vec.empty()) {
        TempStepGroup tsg;
        tsg.destination_stop = destination_stop;

        // Extract flex step if present (it's always first after sorting)
        size_t fixed_start = 0;
        if (step_vec[0].is_flex) {
          tsg.flex_step = AdjacencyListStep::FromStep(step_vec[0]);
          fixed_start = 1;
        }

        // Copy fixed-schedule steps
        tsg.fixed_steps.reserve(step_vec.size() - fixed_start);
        for (size_t i = fixed_start; i < step_vec.size(); ++i) {
          tsg.fixed_steps.push_back(AdjacencyListStep::FromStep(step_vec[i]));
        }

        sorted_minimal_groups.push_back(std::move(tsg));
      }
    }

    if (!sorted_minimal_groups.empty()) {
      // Sort by destination stop ID for consistent ordering
      std::sort(
          sorted_minimal_groups.begin(),
          sorted_minimal_groups.end(),
          [](const TempStepGroup& a, const TempStepGroup& b) {
            return a.destination_stop.v < b.destination_stop.v;
          }
      );
      origin_to_temp_groups[origin_stop.v] = std::move(sorted_minimal_groups);
    }
  }

  // Build CSR format for groups
  StepsAdjacencyList adjacency_list;
  adjacency_list.group_offsets.resize(max_stop_id + 1, 0);

  // First pass: count groups per origin
  for (const auto& [origin_v, groups] : origin_to_temp_groups) {
    adjacency_list.group_offsets[origin_v] = static_cast<int>(groups.size());
  }

  // Convert counts to offsets (prefix sum)
  int running_offset = 0;
  for (int i = 0; i <= max_stop_id; ++i) {
    int count = adjacency_list.group_offsets[i];
    adjacency_list.group_offsets[i] = running_offset;
    running_offset += count;
  }

  // Allocate groups vector
  adjacency_list.groups.resize(running_offset);

  // Second pass: place groups in flat vector and flatten steps.
  // Iterate by origin stop ID in sorted order for deterministic output.
  int steps_offset = 0;
  for (int origin_v = 0; origin_v <= max_stop_id; ++origin_v) {
    auto it = origin_to_temp_groups.find(origin_v);
    if (it == origin_to_temp_groups.end()) {
      continue;
    }
    std::vector<TempStepGroup>& temp_groups = it->second;
    int group_offset = adjacency_list.group_offsets[origin_v];
    for (size_t i = 0; i < temp_groups.size(); ++i) {
      TempStepGroup& tsg = temp_groups[i];
      StepGroup& sg = adjacency_list.groups[group_offset + i];

      sg.destination_stop = tsg.destination_stop;
      sg.flex_step = std::move(tsg.flex_step);
      sg.steps_start = steps_offset;
      sg.steps_end = steps_offset + static_cast<int>(tsg.fixed_steps.size());

      // Append fixed steps to flat vector
      for (const AdjacencyListStep& step : tsg.fixed_steps) {
        adjacency_list.steps.push_back(step);
        adjacency_list.departure_times_div10.push_back(
            static_cast<int16_t>(step.origin_time.seconds / 10)
        );
      }
      steps_offset = sg.steps_end;
    }
  }

  return adjacency_list;
}

StepsAdjacencyList AdjacentPathsToStepsList(const StepPathsAdjacencyList& paths) {
  // Extract all merged steps from paths
  std::vector<Step> all_steps;
  for (const auto& [origin_stop, path_groups] : paths.adjacent) {
    for (const auto& path_group : path_groups) {
      for (const Path& path : path_group) {
        all_steps.push_back(path.merged_step);
      }
    }
  }
  return MakeAdjacencyList(all_steps);
}

}  // namespace vats5
