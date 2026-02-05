#include "relaxed_shortest_path.h"

#include <algorithm>
#include <functional>
#include <limits>

namespace vats5 {

std::vector<int> FindShortestRelaxedPaths(
    const RelaxedAdjacencyList& adjacency_list, StopId origin
) {
  const int num_stops = adjacency_list.NumStops();
  std::vector<int> distances(num_stops, std::numeric_limits<int>::max());
  std::vector<bool> finalized(num_stops, false);

  // Priority queue: (distance, stop_id)
  using Entry = std::pair<int, StopId>;
  auto frontier_cmp = std::greater<Entry>{};
  std::vector<Entry> frontier;

  distances[origin.v] = 0;
  frontier.push_back({0, origin});

  while (!frontier.empty()) {
    std::pop_heap(frontier.begin(), frontier.end(), frontier_cmp);
    auto [current_dist, current_stop] = frontier.back();
    frontier.pop_back();

    if (finalized[current_stop.v]) {
      continue;
    }
    finalized[current_stop.v] = true;

    for (const RelaxedEdge& edge : adjacency_list.GetEdges(current_stop)) {
      int new_dist = current_dist + edge.weight_seconds;
      if (new_dist < distances[edge.destination_stop.v]) {
        distances[edge.destination_stop.v] = new_dist;
        frontier.push_back({new_dist, edge.destination_stop});
        std::push_heap(frontier.begin(), frontier.end(), frontier_cmp);
      }
    }
  }

  return distances;
}

RelaxedDistances ComputeRelaxedDistances(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& destinations
) {
  std::vector<WeightedEdge> relaxed_edges = MakeRelaxedEdges(adjacency_list);
  RelaxedAdjacencyList relaxed =
      MakeRelaxedAdjacencyListFromEdges(relaxed_edges);
  RelaxedAdjacencyList reversed = ReverseRelaxedAdjacencyList(relaxed);

  RelaxedDistances result;

  // Compute distances from each destination separately on the reversed graph
  for (const StopId dest : destinations) {
    result.distance_to[dest] = FindShortestRelaxedPaths(reversed, dest);
  }

  return result;
}

}  // namespace vats5
