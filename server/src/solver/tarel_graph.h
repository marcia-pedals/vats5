#pragma once

#include <functional>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

struct ProblemBoundary {
  StopId start;
  StopId end;
};

struct ProblemState {
  // The graph of minimal steps, i.e. the steps from which all possible tours
  // can be made, with the property that deleting one step will make at least
  // one tour impossible.
  StepsAdjacencyList minimal;

  // The completion of `minimal`, i.e. every possible route between elements of
  // `stops` is a path. Also includes a zero-duration END->START edge to
  // complete the cycle for TSP formulation.
  StepPathsAdjacencyList completed;

  // Which stops in `minimal`/`completed` are the START and END.
  ProblemBoundary boundary;

  // All stops that are required to be visited, including START and END.
  std::unordered_set<StopId> required_stops;

  // Names of all the stops for display purposes.
  std::unordered_map<StopId, std::string> stop_names;

  std::string StopName(StopId stop) const {
    return stop_names.at(stop);
  }
};

ProblemState MakeProblemState(
  StepsAdjacencyList minimal,
  ProblemBoundary boundary,
  std::unordered_set<StopId> stops,
  std::unordered_map<StopId, std::string> stop_names
);

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

inline std::ostream& operator<<(std::ostream& os, const StepPartitionId& value) {
  return os << "Partition{" << value.v << "}";
}

inline std::ostream& operator<<(std::ostream& os, const TarelState& value) {
  return os << "TarelState{" << value.stop << ", " << value.partition << "}";
}

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

  // All steps to `destination` that achieve the minimum `weight`.
  std::vector<Step> steps;

  // All arrival times that achieve the minimum `weight`.
  std::vector<TimeSinceServiceStart> arrival_times;

  // All steps from origin to destination.
  std::vector<Step> all_steps;
};

Step ZeroEdge(StopId a, StopId b);
Path ZeroPath(StopId a, StopId b);

constexpr int kCycleEdgeWeight = -1000;

// Return type for MakeTspGraphEdges.
struct TspGraphData {
  std::vector<TarelState> state_by_id;
  std::unordered_map<TarelState, StopId> id_by_state;
  std::unordered_map<StopId, int> num_states_by_stop;
  std::vector<WeightedEdge> tsp_edges;
  int expected_num_cycle_edges;
};

// Return type for SolveTspAndExtractTour.
struct TspTourResult {
  std::vector<StopId> original_stop_tour;
  std::vector<TimeSinceServiceStart> cumulative_weights;
  std::vector<TarelEdge> tour_edges;
  int optimal_value;
};

// Adds the `boundary` START and END to `stops` and `stop_names`, and adds
// START->* and *->END zero-duration flex steps to `steps`.
void AddBoundary(
 std::vector<Step>& steps,
 std::unordered_set<StopId>& stops,
 std::unordered_map<StopId, std::string>& stop_names,
 ProblemBoundary bounday
);

ProblemState InitializeProblemState(
  const StepsFromGtfs& steps_from_gtfs,
  const std::unordered_set<StopId> system_stops
);

std::vector<TarelEdge> MergeEquivalentTarelStates(
  const std::vector<TarelEdge>& edges
);

TspGraphData MakeTspGraphEdges(
  const std::vector<TarelEdge>& edges,
  const ProblemBoundary& boundary
);

std::optional<TspTourResult> SolveTspAndExtractTour(
  const std::vector<TarelEdge>& edges,
  const TspGraphData& graph,
  const ProblemBoundary& boundary,
  std::optional<int> ub = std::nullopt,
  std::ostream* tsp_log = nullptr
);

std::vector<Path> ComputeMinDurationFeasiblePaths(
  const TspTourResult& tour_result,
  const ProblemState& state
);

void PrintTarelTourResults(
  std::ostream& out,
  const TspTourResult& tour_result,
  const ProblemState& state,
  const Path& feasible_path,
  const std::unordered_map<StepPartitionId, std::string>& state_descriptions
);

std::optional<TspTourResult> ComputeTarelLowerBound(
    const ProblemState& state,
    const std::function<StepPartitionId(Step)>& partition);

std::vector<TarelEdge> MakeTarelEdges(
    const StepPathsAdjacencyList& adj,
    const std::function<StepPartitionId(Step)>& partition
);

void WriteTarelSummary(
  const ProblemState& state,
  const std::string& dir,
  const std::vector<TarelEdge>& edges,
  const std::unordered_map<StepPartitionId, std::string>& state_descriptions
);

}  // namespace vats5
