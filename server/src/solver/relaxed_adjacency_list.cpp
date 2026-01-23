#include "relaxed_adjacency_list.h"

#include <algorithm>
#include <limits>
#include "solver/data.h"

namespace vats5 {

std::vector<WeightedEdge> MakeRelaxedEdges(const StepsAdjacencyList& steps_list
) {
  const int num_stops = steps_list.NumStops();
  std::vector<WeightedEdge> edges;

  for (int origin_v = 0; origin_v < num_stops; ++origin_v) {
    std::span<const StepGroup> groups = steps_list.GetGroups(StopId{origin_v});

    for (const StepGroup& group : groups) {
      int min_duration = std::numeric_limits<int>::max();

      // Check flex step
      if (group.flex_step.has_value()) {
        min_duration = group.flex_step->FlexDurationSeconds();
      }

      // Check fixed-schedule steps
      std::span<const AdjacencyListStep> fixed_steps =
          steps_list.GetSteps(group);
      for (const AdjacencyListStep& step : fixed_steps) {
        int duration = step.destination_time.seconds - step.origin_time.seconds;
        min_duration = std::min(min_duration, duration);
      }

      if (min_duration < std::numeric_limits<int>::max()) {
        edges.push_back(WeightedEdge{
            StopId{origin_v}, group.destination_stop, min_duration
        });
      }
    }
  }

  return edges;
}

std::vector<WeightedEdge> MakeRelaxedEdges(
    const StepPathsAdjacencyList& paths_list
) {
  std::vector<WeightedEdge> edges;

  for (const auto& [origin_stop, path_groups] : paths_list.adjacent) {
    for (const auto& path_group : path_groups) {
      if (path_group.empty()) {
        continue;
      }

      // All paths in a group have the same destination
      StopId destination = path_group[0].merged_step.destination.stop;

      // Find minimum duration across all paths in this group
      int min_duration = std::numeric_limits<int>::max();
      for (const Path& path : path_group) {
        min_duration = std::min(min_duration, path.DurationSeconds());
      }

      if (min_duration < std::numeric_limits<int>::max()) {
        edges.push_back(WeightedEdge{origin_stop, destination, min_duration});
      }
    }
  }

  return edges;
}

RelaxedAdjacencyList MakeRelaxedAdjacencyListFromEdges(
    const std::vector<WeightedEdge>& edges
) {
  if (edges.empty()) {
    return RelaxedAdjacencyList{};
  }

  // Find the number of stops (max stop ID + 1)
  int num_stops = 0;
  for (const WeightedEdge& edge : edges) {
    num_stops = std::max(num_stops, edge.origin.v + 1);
    num_stops = std::max(num_stops, edge.destination.v + 1);
  }

  // Group edges by origin
  std::vector<std::vector<RelaxedEdge>> per_origin_edges(num_stops);
  for (const WeightedEdge& edge : edges) {
    per_origin_edges[edge.origin.v].push_back(
        RelaxedEdge{edge.destination, edge.weight_seconds}
    );
  }

  // Build CSR format
  RelaxedAdjacencyList result;
  result.edge_offsets.resize(num_stops);

  int running_offset = 0;
  for (int i = 0; i < num_stops; ++i) {
    result.edge_offsets[i] = running_offset;
    running_offset += static_cast<int>(per_origin_edges[i].size());
  }

  result.edges.reserve(running_offset);
  for (int i = 0; i < num_stops; ++i) {
    for (const RelaxedEdge& edge : per_origin_edges[i]) {
      result.edges.push_back(edge);
    }
  }

  return result;
}

RelaxedAdjacencyList ReverseRelaxedAdjacencyList(
    const RelaxedAdjacencyList& adjacency_list
) {
  const int num_stops = adjacency_list.NumStops();

  // Collect reversed edges: for each edge (u -> v, w), create (v -> u, w)
  std::vector<std::vector<RelaxedEdge>> per_origin_edges(num_stops);

  for (int origin_v = 0; origin_v < num_stops; ++origin_v) {
    for (const RelaxedEdge& edge :
         adjacency_list.GetEdges(StopId{origin_v})) {
      // Reverse the edge: destination becomes origin
      per_origin_edges[edge.destination_stop.v].push_back(
          RelaxedEdge{StopId{origin_v}, edge.weight_seconds}
      );
    }
  }

  // Build CSR format
  RelaxedAdjacencyList result;
  result.edge_offsets.resize(num_stops);

  int running_offset = 0;
  for (int i = 0; i < num_stops; ++i) {
    result.edge_offsets[i] = running_offset;
    running_offset += static_cast<int>(per_origin_edges[i].size());
  }

  result.edges.reserve(running_offset);
  for (int i = 0; i < num_stops; ++i) {
    for (const RelaxedEdge& edge : per_origin_edges[i]) {
      result.edges.push_back(edge);
    }
  }

  return result;
}

}  // namespace vats5
