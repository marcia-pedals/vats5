#pragma once

#include <crow/multipart.h>

#include <functional>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

struct ProblemStateStopInfo {
  GtfsStopId gtfs_stop_id;
  std::string stop_name;

  bool operator==(const ProblemStateStopInfo& other) const {
    return gtfs_stop_id == other.gtfs_stop_id && stop_name == other.stop_name;
  }
};
// Serialize as [gtfs_stop_id, stop_name] array for compactness
inline void to_json(nlohmann::json& j, const ProblemStateStopInfo& info) {
  j = nlohmann::json::array({info.gtfs_stop_id, info.stop_name});
}
inline void from_json(const nlohmann::json& j, ProblemStateStopInfo& info) {
  info.gtfs_stop_id = j[0].get<GtfsStopId>();
  info.stop_name = j[1].get<std::string>();
}

struct ProblemBoundary {
  StopId start;
  StopId end;
};

inline void to_json(nlohmann::json& j, const ProblemBoundary& b) {
  j = nlohmann::json{{"start", b.start}, {"end", b.end}};
}
inline void from_json(const nlohmann::json& j, ProblemBoundary& b) {
  b.start = j.at("start").get<StopId>();
  b.end = j.at("end").get<StopId>();
}

}  // namespace vats5

template <>
struct std::hash<vats5::StepPartitionId> {
  std::size_t operator()(const vats5::StepPartitionId& v) const {
    return std::hash<int>{}(v.v);
  }
};

namespace vats5 {

struct ProblemState {
  // The graph of minimal steps, i.e. the steps from which all possible tours
  // can be made, with the property that deleting one step will make at least
  // one tour impossible.
  StepsAdjacencyList minimal;

  // Which stops in `minimal` are the START and END.
  ProblemBoundary boundary;

  // All stops that are required to be visited, including START and END.
  std::unordered_set<StopId> required_stops;

  // Information about all stops (GTFS ID and name) for display and lookup.
  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos;

  // Names of step partitions for display purposes.
  std::unordered_map<StepPartitionId, std::string> step_partition_names;

  // If x is a stop representing traveling on an edge, original_edges[x] is that
  // edge. This can be recursive, e.g. if we combine a->b and then (a->b)->c,
  // then the original edge for (a->b)->c has endpoints (a->b) and c, and the
  // original edge for (a->b) has endpoints a and b.
  std::unordered_map<StopId, PlainEdge> original_edges;

  const std::string& StopName(StopId stop) const {
    return stop_infos.at(stop).stop_name;
  }

  StopId StopIdFromName(const std::string& stop_name) {
    auto it = std::find_if(
        stop_infos.begin(), stop_infos.end(), [&stop_name](const auto& pair) {
          return pair.second.stop_name == stop_name;
        }
    );
    assert(it != stop_infos.end());
    return it->first;
  }

  std::string PartitionName(StepPartitionId partition) const {
    auto it = step_partition_names.find(partition);
    if (it == step_partition_names.end()) {
      std::ostringstream ss;
      ss << "unnamed(" << partition.v << ")";
      return ss.str();
    }
    return it->second;
  }

  // Return a copy of this state with the required stops replaced by `stops`.
  ProblemState WithRequiredStops(const std::unordered_set<StopId>& stops) const;
};

// Recursively expands a combined stop into its original constituent stops.
// If the stop is not in original_edges, it is appended as-is.
void ExpandStop(
    StopId stop,
    const std::unordered_map<StopId, PlainEdge>& original_edges,
    std::vector<StopId>& out
);

ProblemState MakeProblemState(
    StepsAdjacencyList minimal,
    ProblemBoundary boundary,
    std::unordered_set<StopId> stops,
    std::unordered_map<StopId, ProblemStateStopInfo> stop_infos,
    std::unordered_map<StepPartitionId, std::string> step_partition_names,
    std::unordered_map<StopId, PlainEdge> original_edges
);

// Compute the "completed" graph for a ProblemState: the completion of
// `state.minimal` where every possible route between elements of
// `state.required_stops` is a path. Includes a zero-duration END->START edge
// to complete the cycle for TSP formulation.
StepPathsAdjacencyList CompletedGraph(const ProblemState& state);

inline void to_json(nlohmann::json& j, const ProblemState& s) {
  std::vector<int> required_stops_vec;
  for (StopId stop : s.required_stops) {
    required_stops_vec.push_back(stop.v);
  }
  std::vector<std::pair<int, ProblemStateStopInfo>> stop_infos_vec;
  for (const auto& [k, v] : s.stop_infos) {
    stop_infos_vec.emplace_back(k.v, v);
  }
  std::vector<std::pair<int, std::string>> step_partition_names_vec;
  for (const auto& [k, v] : s.step_partition_names) {
    step_partition_names_vec.emplace_back(k.v, v);
  }
  std::vector<std::pair<int, PlainEdge>> original_edges_vec;
  for (const auto& [k, v] : s.original_edges) {
    original_edges_vec.emplace_back(k.v, v);
  }
  j = nlohmann::json{
      {"minimal", s.minimal},
      {"boundary", s.boundary},
      {"required_stops", required_stops_vec},
      {"stop_infos", stop_infos_vec},
      {"step_partition_names", step_partition_names_vec},
      {"original_edges", original_edges_vec},
  };
}

inline void from_json(const nlohmann::json& j, ProblemState& s) {
  auto minimal = j.at("minimal").get<StepsAdjacencyList>();
  auto boundary = j.at("boundary").get<ProblemBoundary>();
  std::unordered_set<StopId> required_stops;
  for (int v : j.at("required_stops").get<std::vector<int>>()) {
    required_stops.insert(StopId{v});
  }
  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos;
  for (const auto& [k, v] :
       j.at("stop_infos")
           .get<std::vector<std::pair<int, ProblemStateStopInfo>>>()) {
    stop_infos[StopId{k}] = v;
  }
  std::unordered_map<StepPartitionId, std::string> step_partition_names;
  for (const auto& [k, v] :
       j.at("step_partition_names")
           .get<std::vector<std::pair<int, std::string>>>()) {
    step_partition_names[StepPartitionId{k}] = v;
  }
  std::unordered_map<StopId, PlainEdge> original_edges;
  for (const auto& [k, v] :
       j.at("original_edges").get<std::vector<std::pair<int, PlainEdge>>>()) {
    original_edges[StopId{k}] = v;
  }
  s = MakeProblemState(
      std::move(minimal),
      boundary,
      std::move(required_stops),
      std::move(stop_infos),
      std::move(step_partition_names),
      std::move(original_edges)
  );
}

void showValue(const ProblemState& state, std::ostream& os);

struct TarelState {
  StopId stop;
  StepPartitionId partition;

  bool operator==(const TarelState&) const = default;
  bool operator<(const TarelState& other) const;
};

inline std::ostream& operator<<(
    std::ostream& os, const StepPartitionId& value
) {
  return os << "Partition{" << value.v << "}";
}

inline std::ostream& operator<<(std::ostream& os, const TarelState& value) {
  return os << "TarelState{" << value.stop << ", " << value.partition << "}";
}

}  // namespace vats5

template <>
struct std::hash<vats5::TarelState> {
  std::size_t operator()(const vats5::TarelState& v) const {
    std::size_t seed = std::hash<vats5::StopId>{}(v.stop);
    seed ^= std::hash<vats5::StepPartitionId>{}(v.partition) + 0x9e3779b9 +
            (seed << 6) + (seed >> 2);
    return seed;
  }
};

template <>
struct std::hash<std::pair<vats5::StopId, bool>> {
  std::size_t operator()(const std::pair<vats5::StopId, bool>& v) const {
    std::size_t seed = std::hash<vats5::StopId>{}(v.first);
    seed ^=
        std::hash<bool>{}(v.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
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

// Arrival times to a TarelState.
struct ArrivalTimes {
  std::vector<TimeSinceServiceStart> times;
  bool has_flex = false;
};

// Intermediate data computed by ComputeTarelIntermediateData, used to build
// tarel edges.
struct TarelEdgeIntermediateData {
  // steps_from[x] is all steps from stop x.
  // Within each group, the steps are sorted and minimal.
  std::unordered_map<StopId, std::unordered_map<TarelState, std::vector<Step>>>
      steps_from;

  // arrival_times_to[(x, p)] is all partition-p arrival times to stop x.
  // After construction, times in each value are sorted ascending and unique.
  std::unordered_map<TarelState, ArrivalTimes> arrival_times_to;
};

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

// Computes intermediate data (steps_from and arrival_times_to) from steps.
//
// Precondition: When filtered order-preservingly to any (origin, destination),
// steps satisfy `CheckSortedAndMinimal`.
TarelEdgeIntermediateData ComputeTarelIntermediateData(
    const std::vector<Step>& steps
);

// Builds tarel edges from intermediate data computed by
// ComputeTarelIntermediateData.
std::vector<TarelEdge> BuildTarelEdgesFromIntermediateData(
    const TarelEdgeIntermediateData& data
);

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
    std::unordered_map<StopId, ProblemStateStopInfo>& stop_infos,
    ProblemBoundary bounday
);

struct InitializeProblemStateResult {
  ProblemState problem_state;

  // The full original-data paths corresponding to `problem_state.minimal`
  // steps.
  //
  // Be warned that the StopIds in here are stop ids from the original data, not
  // the "compacted" StopIds in `problem_state`, so you'll have to figure out
  // their correspondence using other data if you need that.
  // TODO: Maybe store the mapping in this struct or do something to make it
  // easier to do the right thing and harder to make a mistake.
  StepPathsAdjacencyList minimal_paths_sparse;
};

InitializeProblemStateResult InitializeProblemState(
    const StepsFromGtfs& steps_from_gtfs,
    const std::unordered_set<StopId> system_stops,
    bool optimize_edges = false
);

std::vector<TarelEdge> MergeEquivalentTarelStates(
    const std::vector<TarelEdge>& edges
);

TspGraphData MakeTspGraphEdges(
    const std::vector<TarelEdge>& edges, const ProblemBoundary& boundary
);

std::optional<TspTourResult> SolveTspAndExtractTour(
    const std::vector<TarelEdge>& edges,
    const TspGraphData& graph,
    const ProblemBoundary& boundary,
    std::optional<int> ub = std::nullopt,
    std::ostream* tsp_log = nullptr
);

std::optional<TspTourResult> ComputeTarelLowerBound(
    const ProblemState& state,
    std::optional<int> ub = std::nullopt,
    std::ostream* tsp_log = nullptr
);

std::vector<TarelEdge> MakeTarelEdges(const StepPathsAdjacencyList& adj);

void WriteTarelSummary(
    const ProblemState& state,
    const std::string& dir,
    const std::vector<TarelEdge>& edges,
    const std::unordered_map<StepPartitionId, std::string>& state_descriptions
);

}  // namespace vats5
