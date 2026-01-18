#include <algorithm>
#include <ios>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "solver/concorde.h"
#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"

using namespace vats5;

struct SolutionMetadata {
  // stop_names[stop_id.v] is the name of stop_id
  std::vector<std::string> stop_names;

  SolutionMetadata Remapped(const StopIdMapping& mapping) {
    SolutionMetadata result;
    result.stop_names.resize(mapping.new_to_original.size(), "");
    for (int i = 0; i < mapping.new_to_original.size(); ++i) {
      result.stop_names[i] = this->stop_names[mapping.new_to_original[i].v];
    }
    return result;
  }
};

SolutionMetadata InitializeSolutionMetadata(const DataGtfsMapping& mapping) {
  int max_stop_id = 0;
  for (const auto& [stop_id, _]: mapping.stop_id_to_stop_name) {
    if (stop_id.v > max_stop_id) {
      max_stop_id = stop_id.v;
    }
  }

  SolutionMetadata result;
  result.stop_names.resize(max_stop_id + 1, "");

  for (const auto& [stop_id, stop_name]: mapping.stop_id_to_stop_name) {
    result.stop_names[stop_id.v] = stop_name;
  }

  return result;
}

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

  std::string StopName(StopId stop) const {
    return metadata.stop_names[stop.v];
  }
};

Step ZeroEdge(StopId a, StopId b) {
  return Step{
    a,
    b,
    TimeSinceServiceStart{0},
    TimeSinceServiceStart{0},
    TripId{-2}, // TODO
    TripId{-2}, // TODO
    /*is_flex=*/true
  };
}

Path ZeroPath(StopId a, StopId b) {
  Step step = ZeroEdge(a, b);
  return Path{step, {step}};
}

SolutionState InitializeSolutionState(
  const StepsFromGtfs& steps_from_gtfs,
  const std::unordered_set<StopId> system_stops
) {
  // Compute minimal adj list.
  StepPathsAdjacencyList minimal_paths_sparse = ReduceToMinimalSystemPaths(MakeAdjacencyList(steps_from_gtfs.steps), system_stops);
  StepsAdjacencyList minimal_steps_sparse = MakeAdjacencyList(minimal_paths_sparse.AllMergedSteps());

  std::unordered_map<TripId, std::string> dest_trip_id_to_partition;
  for (const auto& [_, groups] : minimal_paths_sparse.adjacent) {
    for (const auto& group : groups) {
      for (const auto& path : group) {
        std::string& partition = dest_trip_id_to_partition[path.merged_step.destination_trip];
        partition = "flex";
        for (const auto& step : path.steps) {
          if (!step.is_flex) {
            partition = steps_from_gtfs.mapping.trip_id_to_route_desc.at(step.destination_trip);
          }
        }
      }
    }
  }

  // Compact minimal adj list and make compact solution metadata.
  CompactStopIdsResult minimal_compact = CompactStopIds(minimal_steps_sparse);
  SolutionMetadata solution_metadata = InitializeSolutionMetadata(steps_from_gtfs.mapping).Remapped(minimal_compact.mapping);

  int num_actual_stops = minimal_compact.list.NumStops();

  // Add the "START" and "END" vertices.
  StopId start_vertex = StopId{num_actual_stops};
  solution_metadata.stop_names.push_back("START");
  assert(solution_metadata.stop_names[start_vertex.v] == "START");
  StopId end_vertex = StopId{num_actual_stops + 1};
  solution_metadata.stop_names.push_back("END");
  assert(solution_metadata.stop_names[end_vertex.v] == "END");

  // ... with 0-duration flex steps START->* and *->END.
  std::vector<Step> steps = minimal_compact.list.AllSteps();
  for (StopId actual_stop = StopId{0}; actual_stop.v < num_actual_stops; actual_stop.v += 1) {
    steps.push_back(ZeroEdge(start_vertex, actual_stop));
    steps.push_back(ZeroEdge(actual_stop, end_vertex));
  }

  std::unordered_set<StopId> stops;
  for (StopId stop = StopId{0}; stop.v < end_vertex.v + 1; stop.v += 1) {
    stops.insert(stop);
  }

  return SolutionState{
    stops,
    MakeAdjacencyList(steps),
    SolutionBoundary{.start=start_vertex, .end=end_vertex},
    solution_metadata,
    dest_trip_id_to_partition,
  };
}

void ExtendFeasiblePaths(
    std::vector<Step>& feasible_paths,
    const StepPathsAdjacencyList& completed,
    StopId a,
    StopId b) {
  std::vector<Step> next_steps = completed.MergedStepsBetween(a, b);
  feasible_paths = PairwiseMergedSteps(std::move(feasible_paths), std::move(next_steps));
}

struct StepPartitionId {
  int v;
  bool operator==(const StepPartitionId&) const = default;
  auto operator<=>(const StepPartitionId&) const = default;
};

template <>
struct std::hash<StepPartitionId> {
  std::size_t operator()(const StepPartitionId& v) const {
    return std::hash<int>{}(v.v);
  }
};

template <>
struct std::hash<std::pair<StopId, bool>> {
  std::size_t operator()(const std::pair<StopId, bool>& v) const {
    std::size_t seed = std::hash<StopId>{}(v.first);
    seed ^= std::hash<bool>{}(v.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct TarelState {
  StopId stop;
  StepPartitionId partition;

  bool operator==(const TarelState&) const = default;
  bool operator<(const TarelState& other) const {
    if (stop.v != other.stop.v) return stop.v < other.stop.v;
    return partition.v < other.partition.v;
  }
};

template <>
struct std::hash<TarelState> {
  std::size_t operator()(const TarelState& v) const {
    std::size_t seed = std::hash<StopId>{}(v.stop);
    seed ^= std::hash<StepPartitionId>{}(v.partition) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

template <typename T>
struct std::hash<std::pair<StopId, T>> {
  std::size_t operator()(const std::pair<StopId, T>& v) const {
    std::size_t seed = std::hash<StopId>{}(v.first);
    seed ^= std::hash<T>{}(v.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
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

struct TarelEdgesResult {
  std::vector<TarelEdge> edges;
  std::unordered_map<StepPartitionId, std::string> state_descriptions;
};

template <typename T>
concept Hashable = requires(T t) {
    { std::hash<T>{}(t) } -> std::convertible_to<std::size_t>;
};

struct ArrivalTimesFlex {};

struct ArrivalTimesScheduled {
  std::vector<TimeSinceServiceStart> times;
};

template <Hashable PartitionKey>
TarelEdgesResult MakeTarelEdges(
    const StepPathsAdjacencyList& adj,
    const SolutionBoundary& boundary,
    std::function<PartitionKey(Step)> partition,
    std::function<std::string(PartitionKey)> describe_partition = nullptr) {
  // Maps (stop, partition_key) -> StepPartitionId, assigning contiguous ids per stop.
  // std::unordered_map<StopId, std::unordered_map<PartitionKey, StepPartitionId>> partition_id_map;
  std::unordered_map<std::pair<StopId, PartitionKey>, StepPartitionId> partition_id_map;

  // State descriptions populated when describe_partition is provided.
  std::unordered_map<StepPartitionId, std::string> state_descriptions;

  auto get_partition_id = [&partition_id_map, &state_descriptions, &describe_partition](StopId dest_stop, PartitionKey key) -> StepPartitionId {
    auto with_dest = std::make_pair(dest_stop, key);
    auto [it, inserted] = partition_id_map.try_emplace(with_dest, StepPartitionId{static_cast<int>(partition_id_map.size())});
    if (inserted) {
      state_descriptions[it->second] = describe_partition(key);
    }
    return it->second;

    // auto& stop_map = partition_id_map[key];
    // auto it = stop_map.find(key);
    // if (it != stop_map.end()) {
    //   return TarelState{stop, it->second};
    // }
    // StepPartitionId new_id{static_cast<int>(stop_map.size())};
    // stop_map[key] = new_id;
    // if (describe_partition) {
    //   state_descriptions[TarelState{stop, new_id}] = describe_partition(key);
    // }
    // return TarelState{stop, new_id};
  };

  // steps_from[x] is all steps from x.
  // Within each group, the steps are sorted and minimal.
  std::unordered_map<StopId, std::unordered_map<std::pair<StopId, PartitionKey>, std::vector<Step>>> steps_from;

  // arrival_times_to[(x, p)] is all partition-p arrival times to stop x.
  // Note that I've been careful to put `ArrivalTimesScheduled` as the first
  // alternative so that when you []-access a value that doesn't exist yet, it
  // starts as an empty `ArrivalTimesScheduled`.
  // After we have constructed this, times in each value are sorted ascending
  // and unique.
  std::unordered_map<std::pair<StopId, PartitionKey>, std::variant<ArrivalTimesScheduled, ArrivalTimesFlex>> arrival_times_to;

  bool saw_start_as_origin = false;
  bool saw_start_as_dest = false;

  for (const auto& [origin_stop, path_groups] : adj.adjacent) {
    for (const auto& path_group : path_groups) {
      for (const Path& path : path_group) {
        const Step& step = path.merged_step;
        assert(step.origin_stop == origin_stop);
        PartitionKey pk = partition(step);

        if (step.origin_stop == boundary.start) {
          saw_start_as_origin = true;
        }
        if (step.destination_stop == boundary.start) {
          saw_start_as_dest = true;
        }

        // Preserves sorted-and-minimal property because: The paths within
        // `path_group` are sorted and minimal, they all have the same
        // `step.destination_stop`, no other path groups fom this origin have
        // the same destination stop, and order-preserving partitions preserve
        // sortedness and minimality.
        steps_from[step.origin_stop][std::make_pair(step.destination_stop, pk)].push_back(step);

        auto& arrival_times = arrival_times_to[std::make_pair(step.destination_stop, pk)];
        if (step.is_flex || std::holds_alternative<ArrivalTimesFlex>(arrival_times)) {
          arrival_times = ArrivalTimesFlex();
        } else {
          std::get<ArrivalTimesScheduled>(arrival_times).times.push_back(step.destination_time);
        }
      }
    }
  }

  assert(saw_start_as_origin);
  assert(saw_start_as_dest);

  bool sf_has_start = false;

  for (const auto& [x, groups_from] : steps_from) {
    if (x == boundary.start) {
      sf_has_start = true;
    }
    for (const auto& [_, group_from] : groups_from) {
      assert(CheckSortedAndMinimal(group_from));
    }
  }

  assert(sf_has_start);

  for (auto& [_, times_to] : arrival_times_to) {
    if (std::holds_alternative<ArrivalTimesScheduled>(times_to)) {
      std::vector<TimeSinceServiceStart>& times = std::get<ArrivalTimesScheduled>(times_to).times;
      std::ranges::sort(times);
      auto [first, last] = std::ranges::unique(times);
      times.erase(first, last);
    }
  }

  std::vector<TarelEdge> edges;
  for (const auto& [origin_vertex, arrival_times_to_origin] : arrival_times_to) {
    for (const auto& [dest_vertex, steps] : steps_from[origin_vertex.first]) {
      int weight = std::numeric_limits<int>::max();
      if (std::holds_alternative<ArrivalTimesFlex>(arrival_times_to_origin)) {
        // If the arrival is flex, we have to assume we can arrive any time, so
        // the weight is simply the duration of the shortest step out.
        for (const Step& step : steps) {
          if (step.DurationSeconds() < weight) {
            weight = step.DurationSeconds();
          }
        }
      } else {
        // The arrival is scheduled.
        int step_idx = 0;
        if (steps.size() > 0 && steps[0].is_flex) {
          if (steps[0].FlexDurationSeconds() < weight) {
            weight = steps[0].FlexDurationSeconds();
          }
          step_idx = 1;
        }
        for (const TimeSinceServiceStart arrival_time : std::get<ArrivalTimesScheduled>(arrival_times_to_origin).times) {
          while (step_idx < steps.size() && steps[step_idx].origin_time < arrival_time) {
            step_idx += 1;
          }
          if (step_idx >= steps.size()) {
            break;
          }
          int duration = steps[step_idx].destination_time.seconds - arrival_time.seconds;
          if (duration < weight) {
            weight = duration;
          }
        }
      }

      if (weight < std::numeric_limits<int>::max()) {
        TarelState origin{origin_vertex.first, get_partition_id(origin_vertex.first, origin_vertex.second)};
        TarelState dest{dest_vertex.first, get_partition_id(dest_vertex.first, dest_vertex.second)};

        edges.push_back(TarelEdge{
          .origin=origin,
          .destination=dest,
          .weight=weight,
          .original_origins={origin},
          .original_destinations={dest},
        });
      }
    }
  }

  std::cout << "start is " << boundary.start << ", end is " << boundary.end << "\n";

  return TarelEdgesResult{
    .edges = std::move(edges),
    .state_descriptions = std::move(state_descriptions),
  };
}

struct EdgeSignature {
 std::vector<std::pair<TarelState, int>> outgoing;

 bool operator<(const EdgeSignature& other) const {
   return outgoing < other.outgoing;
 }

 std::string ToString(
   const SolutionState& state,
   const std::unordered_map<StepPartitionId, std::string>& state_descriptions
 ) const {
   std::ostringstream oss;
   oss << "EdgeSignature {\n";
   oss << "  outgoing (" << outgoing.size() << "):\n";
   for (const auto& [ts, weight] : outgoing) {
     oss << "    " << state.StopName(ts.stop) << " [" << state_descriptions.at(ts.partition) << "] w=" << weight << "\n";
   }
   oss << "}";
   return oss.str();
 }
};

std::vector<TarelEdge> MergeEquivalentTarelStates(const std::vector<TarelEdge>& edges, const SolutionState& state, const std::unordered_map<StepPartitionId, std::string>& state_descriptions) {
  // Two tarel states are "equivalent" if they have the same `origin.stop`, and
  // they have the same set of `destination`s and they have matching `weights`
  // to each `destination`.

  bool has_start = false;
  for (const TarelEdge& edge : edges) {
    if (edge.origin.stop == state.boundary.start || edge.destination.stop == state.boundary.start) {
      has_start = true;
      break;
    }
  }
  assert(has_start);

  // Step 1: Build the "edge signature" for each TarelState.
  // The signature is the sorted list of (destination, weight) pairs.
  std::unordered_map<TarelState, EdgeSignature> signatures;
  for (const TarelEdge& edge : edges) {
    signatures[edge.origin].outgoing.push_back({edge.destination, edge.weight});
  }
  for (auto& [_, signature] : signatures) {
    std::ranges::sort(signature.outgoing, [](const auto& a, const auto& b) {
      if (a.first != b.first) return a.first < b.first;
      return a.second < b.second;
    });
    // auto [out_begin, out_end] = std::ranges::unique(signature.outgoing);
    // signature.outgoing.erase(out_begin, out_end);
  }

  // Step 2: For each stop, group partitions by their edge signature and pick a canonical one.
  std::unordered_map<TarelState, TarelState> canonical_state;

  std::unordered_map<StopId, std::vector<TarelState>> states_by_stop;
  for (const auto& [origin, _] : signatures) {
    states_by_stop[origin.stop].push_back(origin);
  }
  assert(states_by_stop[state.boundary.start].size() > 0);
  assert(states_by_stop[state.boundary.end].size() > 0);

  for (const auto& [stop, states] : states_by_stop) {
    std::map<EdgeSignature, TarelState> signature_to_canonical;

    for (const TarelState& ts : states) {
      const auto& signature = signatures.at(ts);
      auto [it, _] = signature_to_canonical.try_emplace(signature, ts);
      canonical_state[ts] = it->second;
    }

    // if (stop == state.boundary.end) {
    //   for (const auto& [sig, _] : signature_to_canonical) {
    //     std::cout << sig.ToString(state, state_descriptions) << "\n";
    //   }
    // }
  }

  // Step 3: Collect all canonical states and reassign contiguous partition IDs per stop.
  std::unordered_set<TarelState> all_canonical_states;
  for (const auto& [ts, canonical] : canonical_state) {
    all_canonical_states.insert(canonical);
  }

  // Group canonical states by stop and assign contiguous IDs.
  std::unordered_map<StopId, std::vector<TarelState>> canonical_by_stop;
  for (const TarelState& ts : all_canonical_states) {
    canonical_by_stop[ts.stop].push_back(ts);
  }

  std::unordered_map<TarelState, TarelState> renumbered_state;
  for (auto& [stop, states] : canonical_by_stop) {
    // Sort for deterministic ordering.
    std::ranges::sort(states, [](const TarelState& a, const TarelState& b) { return a < b; });
    for (int i = 0; i < states.size(); ++i) {
      renumbered_state[states[i]] = TarelState{stop, StepPartitionId{i}};
    }
  }

  // Compose: original -> canonical -> renumbered
  auto final_state = [&](const TarelState& ts) -> TarelState {
    return renumbered_state[canonical_state[ts]];
  };

  // Collect original states and weight for each merged edge.
  // Key is (new_origin, new_dest).
  struct MergedEdgeData {
    int weight;
    std::vector<TarelState> original_origins;
    std::vector<TarelState> original_destinations;
  };
  std::map<std::pair<TarelState, TarelState>, MergedEdgeData> merged_edges;
  for (const TarelEdge& edge : edges) {
    TarelState new_origin = final_state(edge.origin);
    TarelState new_dest = final_state(edge.destination);
    auto [it, inserted] = merged_edges.try_emplace({new_origin, new_dest}, MergedEdgeData{.weight = edge.weight});
    auto& data = it->second;
    if (!inserted) {
      // TODO: Think harder about whether taking the min merged edge weight makes sense.
      if (edge.weight < data.weight) {
        data.weight = edge.weight;
      }

      // if (data.weight != edge.weight) {
      //   std::cout << "Merged edge weight mismatch!!!\n";
      //   for (int i = 0; i < data.original_origins.size(); ++i) {
      //     std::cout << "Existing edge: "
      //       << state.StopName(data.original_origins[i].stop) << " " << state_descriptions.at(data.original_origins[i].partition) << " -> "
      //       << state.StopName(data.original_destinations[i].stop) << " " << state_descriptions.at(data.original_destinations[i].partition) << "\n";
      //   }
      //   std::cout << "New edge: "
      //     << state.StopName(new_origin.stop) << " " << state_descriptions.at(edge.origin.partition) << " -> "
      //     << state.StopName(new_dest.stop) << " " << state_descriptions.at(edge.destination.partition) << "\n";
      //   std::cout << "Existing weight: " << data.weight << "\n";
      //   std::cout << "New weight: " << edge.weight << "\n";

      //   // auto signature = outgoing_edges[edge.destination];
      //   // std::cout << "Signature:\n";
      //   // for (const auto& [ts, w] : signature) {
      //   //   std::cout << state.StopName(ts.stop)  << " " << state_descriptions.at(ts.partition) << ": " << w << "\n";
      //   // }
      // }
      // assert(data.weight == edge.weight);
    }
    for (const TarelState& orig : edge.original_origins) {
      data.original_origins.push_back(orig);
    }
    for (const TarelState& orig : edge.original_destinations) {
      data.original_destinations.push_back(orig);
    }
  }

  std::vector<TarelEdge> result;
  for (const auto& [key, data] : merged_edges) {
    const auto& [new_origin, new_dest] = key;
    auto sorted_origins = data.original_origins;
    auto sorted_destinations = data.original_destinations;
    std::ranges::sort(sorted_origins, [](const TarelState& a, const TarelState& b) { return a < b; });
    std::ranges::sort(sorted_destinations, [](const TarelState& a, const TarelState& b) { return a < b; });
    sorted_origins.erase(std::unique(sorted_origins.begin(), sorted_origins.end()), sorted_origins.end());
    sorted_destinations.erase(std::unique(sorted_destinations.begin(), sorted_destinations.end()), sorted_destinations.end());
    result.push_back(TarelEdge{
      .origin = new_origin,
      .destination = new_dest,
      .weight = data.weight,
      .original_origins = std::move(sorted_origins),
      .original_destinations = std::move(sorted_destinations),
    });
  }

  return result;
}

constexpr int kCycleEdgeWeight = -1000;

void SolveTarelTspInstance(
  const std::vector<TarelEdge>& edges,
  const SolutionState& state,
  const StepPathsAdjacencyList& completed,
  const std::unordered_map<StepPartitionId, std::string>& state_descriptions
) {
  // Assign contiguous ids to all TarelStates.
  std::vector<TarelState> state_by_id;
  std::unordered_map<TarelState, StopId> id_by_state;
  auto insert_state_if_new = [&](const TarelState& state) {
    auto [_, inserted] = id_by_state.try_emplace(state, StopId{static_cast<int>(state_by_id.size())});
    if (inserted) {
      state_by_id.push_back(state);
    }
  };
  for (const TarelEdge& edge : edges) {
    insert_state_if_new(edge.origin);
    insert_state_if_new(edge.destination);
  }

  // Count how many `TarelStates` each stop has.
  // (Then, the states should be {stop, 0}, {stop, 1}, ..., {stop, num_states - 1}).
  std::unordered_map<StopId, int> num_states_by_stop;
  for (const TarelState& state : state_by_id) {
    num_states_by_stop[state.stop] += 1;
  }
  assert(num_states_by_stop[state.boundary.start] == 1);
  // std::cout << num_states_by_stop[state.boundary.end] << "\n";
  // assert(num_states_by_stop[state.boundary.end] == 1);

  // Start building up TSP edges.
  std::vector<WeightedEdge> tsp_edges;

  // Add TSP edges: within-stop-cycles.
  int expected_num_cycle_edges = 0;
  for (const auto& [stop, num_states] : num_states_by_stop) {
    for (StepPartitionId partition{0}; partition.v < num_states; ++partition.v) {
      StepPartitionId next_partition{(partition.v + 1) % num_states};
      tsp_edges.push_back(WeightedEdge{
        .origin=id_by_state.at(TarelState{stop, partition}),
        .destination=id_by_state.at(TarelState{stop, next_partition}),
        .weight_seconds=kCycleEdgeWeight,
      });
    }
    expected_num_cycle_edges += num_states - 1;
  }

  // Add TSP edges: inter-stop travel.
  for (const TarelEdge& edge : edges) {
    // Offset origin by -1 for TSP trick.
    TarelState origin = edge.origin;
    origin.partition.v -= 1;
    if (origin.partition.v < 0) {
      origin.partition.v += num_states_by_stop.at(origin.stop);
    }
    tsp_edges.push_back(WeightedEdge{
      .origin=id_by_state.at(origin),
      .destination=id_by_state.at(edge.destination),
      .weight_seconds=edge.weight,
    });
  }

  // Old: 596 vertices and 28077 edges
  // New: 596 vertices and 29984 edges

  // Solve TSP!!!!
  std::cout << "Solving TSP with " << state_by_id.size() << " vertices and " << tsp_edges.size() << " edges...\n";
  ConcordeSolution solution = SolveTspWithConcorde(MakeRelaxedAdjacencyListFromEdges(tsp_edges));

  // Adjust optimal value and rotate tour.
  solution.optimal_value -= expected_num_cycle_edges * kCycleEdgeWeight;
  auto tour_start_it = std::find(solution.tour.begin(), solution.tour.end(), id_by_state.at(TarelState{state.boundary.start, 0}));
  assert(tour_start_it != solution.tour.end());
  std::rotate(solution.tour.begin(), tour_start_it, solution.tour.end());
  assert(*(solution.tour.end() - 1) == id_by_state.at(TarelState{state.boundary.end, 0}));
  std::cout << "Tarel TSP optimal value: " << TimeSinceServiceStart{solution.optimal_value}.ToString() << "\n";

  // Build lookup for tarel edge weights: (origin, destination) -> weight
  auto find_tarel_edge = [&edges](const TarelState& origin, const TarelState& dest) -> TarelEdge {
    for (const TarelEdge& edge : edges) {
      if (edge.origin == origin && edge.destination == dest) {
        return edge;
      }
    }
    assert(false);
  };

  // Validate tour and extract original StopIds.
  std::vector<std::string> tour_errors;
  std::vector<StopId> original_stop_tour;
  std::vector<TimeSinceServiceStart> cumulative_weights;
  std::vector<TarelEdge> tour_edges;  // All edges including START->first and last->END
  TimeSinceServiceStart accumulated_weight{0};
  TarelState cur_state = TarelState{state.boundary.start, 0};
  int cur_stop_visited_states = 1;
  for (int tour_idx = 1; tour_idx < solution.tour.size() + 1; ++tour_idx) {
    int cur_stop_num_states = num_states_by_stop.at(cur_state.stop);

    if (tour_idx == solution.tour.size() || state_by_id[solution.tour[tour_idx].v].stop != cur_state.stop) {
      // We're exiting the current stop: validate that we visited all of its states.
      if (cur_stop_visited_states != cur_stop_num_states) {
        tour_errors.push_back(
          "Visited only "
          + std::to_string(cur_stop_visited_states) + " / " + std::to_string(cur_stop_num_states)
          + " for " + std::to_string(cur_state.stop.v)
        );
      }
    }

    if (tour_idx == solution.tour.size()) {
      continue;
    }

    TarelState next_state = state_by_id[solution.tour[tour_idx].v];
    if (next_state.stop == cur_state.stop) {
      if (next_state.partition.v != (cur_state.partition.v + 1) % cur_stop_num_states) {
        tour_errors.push_back(
          "Forbidden transition "
          + std::to_string(cur_state.partition.v) + " -> " + std::to_string(next_state.partition.v)
          + " for " + std::to_string(cur_state.stop.v)
        );
      }
      cur_stop_visited_states += 1;
    } else {
      // Inter-stop transition: compute the tarel edge weight.
      // The TSP edge from cur_state to next_state corresponds to a tarel edge
      // with origin partition = (cur_state.partition + 1) % num_states.
      TarelState tarel_origin = cur_state;
      tarel_origin.partition.v = (cur_state.partition.v + 1) % cur_stop_num_states;
      TarelEdge edge = find_tarel_edge(tarel_origin, next_state);
      accumulated_weight.seconds += edge.weight;
      cumulative_weights.push_back(accumulated_weight);
      tour_edges.push_back(edge);

      cur_stop_visited_states = 1;
      if (cur_state.stop != state.boundary.start) {
        // Note: Intentionally missing the last stop of `tour` because that is END.
        original_stop_tour.push_back(cur_state.stop);
      }
    }

    cur_state = next_state;
  }
  for (const std::string& error : tour_errors) {
    std::cout << error << "\n";
  }
  assert(tour_errors.size() == 0);

  // Compute min duration feasible path.
  std::vector<Step> feasible_paths = {ZeroEdge(state.boundary.start, state.boundary.start)};
  for (const auto& edge : tour_edges) {
    feasible_paths = PairwiseMergedSteps(
      feasible_paths,
      completed.MergedStepsBetween(edge.origin.stop, edge.destination.stop)
    );
  }

  std::optional<TimeSinceServiceStart> min_duration_origin_time;
  if (!feasible_paths.empty()) {
    const Step* min_duration_step = &feasible_paths[0];
    for (const Step& step : feasible_paths) {
      if (step.DurationSeconds() < min_duration_step->DurationSeconds()) {
        min_duration_step = &step;
      }
    }
    min_duration_origin_time = min_duration_step->origin_time;
    std::cout << "\nMin duration feasible path:\n";
    std::cout << "  Start time: " << min_duration_step->origin_time.ToString() << "\n";
    std::cout << "  End time:   " << min_duration_step->destination_time.ToString() << "\n";
    std::cout << "  Duration:   " << TimeSinceServiceStart{min_duration_step->DurationSeconds()}.ToString() << "\n";
  } else {
    std::cout << "No feasible paths!?\n";
  }

  // Compute departure times for the min duration feasible path.
  //
  // feasible_arrivals[i] is the time we **arrive** the i-th stop of the
  // feasible path, to match the cumulative tarel weights.
  std::vector<TimeSinceServiceStart> feasible_arrivals;
  std::optional<Step> cur_step;
  if (min_duration_origin_time.has_value()) {
    cur_step = ZeroEdge(state.boundary.start, state.boundary.start);
    cur_step->origin_time = *min_duration_origin_time;
    cur_step->destination_time = *min_duration_origin_time;
  }
  for (int edge_idx = 0; edge_idx < tour_edges.size(); ++edge_idx) {
    const TarelEdge& edge = tour_edges[edge_idx];
    if (!cur_step.has_value()) {
      feasible_arrivals.push_back(TimeSinceServiceStart{-1});
      continue;
    }
    feasible_arrivals.push_back(cur_step->destination_time);
    cur_step = SelectBestNextStep(
      *cur_step,
      // TODO: AAAA unfortunate allocation.
      completed.MergedStepsBetween(edge.origin.stop, edge.destination.stop)
    );
  }

  // feasible_durations[i] is the time between arriving at the START and
  // arriving at the (i+1)-th stop of the feasible path. (i+1 to disregard the
  // START).
  std::vector<TimeSinceServiceStart> feasible_durations;
  for (int i = 1; i < feasible_arrivals.size(); ++i) {
    feasible_durations.push_back(TimeSinceServiceStart{feasible_arrivals[i].seconds - feasible_arrivals[0].seconds});
  }

  // Print tour with cumulative tarel weight and feasible duration.
  const int align_spacing = 50;
  int prev_diff = 0;
  for (int i = 0; i < original_stop_tour.size(); ++i) {
    std::cout << std::left << std::setw(align_spacing) << state.StopName(original_stop_tour[i])
      << cumulative_weights[i].ToString();
    if (feasible_durations[i].seconds >= 0) {
      std::cout << "  " << feasible_durations[i].ToString();
      int cur_diff = feasible_durations[i].seconds - cumulative_weights[i].seconds;
      int delta = cur_diff - prev_diff;
      std::cout << "  " << std::right << std::setw(2) << (delta / 60) << ":" << std::setfill('0') << std::setw(2) << (delta % 60) << std::setfill(' ') << std::left;
      prev_diff = cur_diff;
    }
    std::cout << "\n";

    TarelEdge& edge = tour_edges[i + 1];
    for (auto origin : edge.original_origins) {
      std::cout << "  [origin] " << state_descriptions.at(origin.partition) << "\n";
    }

    for (auto destination : edge.original_destinations) {
      std::cout << "  [dest] " << state_descriptions.at(destination.partition) << "\n";
    }
  }
}

int main() {
    const std::string gtfs_path = "../data/RG_20260108_all";

    std::cout << "Loading GTFS data from: " << gtfs_path << std::endl;
    GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);

    gtfs_day = GtfsNormalizeStops(gtfs_day);
    StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});

    std::unordered_set<StopId> bart_stops =
        GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

    std::cout << "Initializing solution state...\n";
    SolutionState state = InitializeSolutionState(steps_from_gtfs, bart_stops);

    // Dummy code for typechecking.
    StepPathsAdjacencyList completed =
      ReduceToMinimalSystemPaths(state.adj, state.stops, /*keep_through_other_destination=*/true);

    // Add END -> START edge.
    // It's safe to push a new group without checking for an existing group with
    // `start` dest because no edges (other than this one we're adding right
    // now) go into `start`.
    assert(completed.adjacent[state.boundary.end].size() == 0);
    completed.adjacent[state.boundary.end].push_back({ZeroPath(state.boundary.end, state.boundary.start)});

    auto tarel_result = MakeTarelEdges(
      completed,
      state.boundary,
      std::function<std::string(Step)>([&](const Step& s) -> std::string {
        if (s.is_flex) {
          return "flex";
        }
        return state.dest_trip_id_to_partition[s.destination_trip];
      }),
      std::function([&](std::string pk) -> std::string {
        return pk;
      }));
    auto tarel_es_2 = MergeEquivalentTarelStates(tarel_result.edges, state, tarel_result.state_descriptions);
    SolveTarelTspInstance(tarel_es_2, state, completed, tarel_result.state_descriptions);
    // SolveTarelTspInstance(tarel_es_2, state, completed);
    // TODO: Think about whether it's possible for there to be a situation where
    // merging multiple times makes progress each time.
    // ... it seems like merging states could make it be so that some states who
    // previously had distinct dest states could now have the same dest states.
    // But:
    // - Maybe there's a reason why anything that ends up with same dest states must have started with same dest states anyways.
    // - Or not quite, but where there has to be a very unlikely coincidence of unrelated weights for dest states to get merged in such a way.
    // auto tarel_es_3 = MergeEquivalentTarelStates(tarel_es_2, state, tarel_result.state_descriptions);

    return 0;
}
