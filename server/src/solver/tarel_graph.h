#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

struct SolutionMetadata {
  // stop_names[stop_id.v] is the name of stop_id
  std::vector<std::string> stop_names;

  SolutionMetadata Remapped(const StopIdMapping& mapping);
};

struct SolutionBoundary {
  StopId start;
  StopId end;
};

struct SolutionState {
  std::unordered_set<StopId> stops;
  StepsAdjacencyList adj;
  SolutionBoundary boundary;
  SolutionMetadata metadata;

  std::unordered_map<TripId, std::string> dest_trip_id_to_partition;

  std::string StopName(StopId stop) const;
};

struct StepPartitionId {
  int v;
  bool operator==(const StepPartitionId&) const = default;
  auto operator<=>(const StepPartitionId&) const = default;
};

struct TarelState {
  StopId stop;
  StepPartitionId partition;

  bool operator==(const TarelState&) const = default;
  bool operator<(const TarelState& other) const;
};

}  // namespace vats5

template <>
struct std::hash<vats5::StepPartitionId> {
  std::size_t operator()(const vats5::StepPartitionId& v) const {
    return std::hash<int>{}(v.v);
  }
};

template <>
struct std::hash<vats5::TarelState> {
  std::size_t operator()(const vats5::TarelState& v) const {
    std::size_t seed = std::hash<vats5::StopId>{}(v.stop);
    seed ^= std::hash<vats5::StepPartitionId>{}(v.partition) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

template <>
struct std::hash<std::pair<vats5::StopId, bool>> {
  std::size_t operator()(const std::pair<vats5::StopId, bool>& v) const {
    std::size_t seed = std::hash<vats5::StopId>{}(v.first);
    seed ^= std::hash<bool>{}(v.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

template <typename T>
struct std::hash<std::pair<vats5::StopId, T>> {
  std::size_t operator()(const std::pair<vats5::StopId, T>& v) const {
    std::size_t seed = std::hash<vats5::StopId>{}(v.first);
    seed ^= std::hash<T>{}(v.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

namespace vats5 {

// An edge from `origin.stop` to `destination.stop` in the "tarel graph".
//
// "What's a tarel graph?", you might ask. It stands for Transfer-Aware
// RELaxation. It works like this:
//
// Each step is assigned a `StepPartitionId`. Then, the weight of a tarel edge
// is the min possible time between _arriving_ at `origin.stop` using a
// `origin.partition` step and _arriving_ at `destination.stop` using a
// `destination.partition` step.
//
// The idea is that if you partition steps by something like what transit line
// they come from, then the transfer time betweent two lines is reasonably
// consistent and so the min time to transfer and get to the next stop is a
// reasonably tight lower bound on the actual time.
//
// Also, as you will see later, it is reasonably straightforward to express the
// objective of minimizing weight on a tarel graph as a vanilla TSP.
struct TarelEdge {
  TarelState origin;
  TarelState destination;
  int weight;
  std::vector<TarelState> original_origins;
  std::vector<TarelState> original_destinations;
};

Step ZeroEdge(StopId a, StopId b);
Path ZeroPath(StopId a, StopId b);

SolutionState InitializeSolutionState(
  const StepsFromGtfs& steps_from_gtfs,
  const std::unordered_set<StopId> system_stops
);

std::vector<TarelEdge> MergeEquivalentTarelStates(
  const std::vector<TarelEdge>& edges
);

void SolveTarelTspInstance(
  const std::vector<TarelEdge>& edges,
  const SolutionState& state,
  const StepPathsAdjacencyList& completed,
  const std::unordered_map<StepPartitionId, std::string>& state_descriptions
);

std::vector<TarelEdge> MakeTarelEdges(
    const StepPathsAdjacencyList& adj,
    const std::function<StepPartitionId(Step)>& partition
);

}  // namespace vats5
