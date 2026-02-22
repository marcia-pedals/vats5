#include "solver/tarel_graph.h"

#include <algorithm>
#include <fstream>
#include <ios>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "solver/concorde.h"
#include "solver/data.h"
#include "solver/relaxed_adjacency_list.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"

namespace vats5 {

ProblemState ProblemState::WithRequiredStops(
    const std::unordered_set<StopId>& stops
) const {
  return MakeProblemState(
      minimal, boundary, stops, stop_infos, step_partition_names, original_edges
  );
}

void showValue(const ProblemState& state, std::ostream& os) {
  std::vector<Step> steps = state.minimal.AllSteps();
  StopId start = state.boundary.start;
  StopId end = state.boundary.end;

  auto sort_key = [&](const Step& s) {
    bool adj_start = (s.origin.stop == start || s.destination.stop == start);
    bool adj_end = (s.origin.stop == end || s.destination.stop == end);
    int group;
    if (adj_start)
      group = 2;
    else if (adj_end)
      group = 3;
    else if (!s.is_flex)
      group = 0;
    else
      group = 1;
    return std::tuple(group, s.origin.stop.v, s.destination.stop.v);
  };
  std::sort(steps.begin(), steps.end(), [&](const Step& a, const Step& b) {
    return sort_key(a) < sort_key(b);
  });

  std::vector<StopId> all_stop_ids;
  for (const auto& [id, name] : state.stop_infos) {
    all_stop_ids.push_back(id);
  }
  std::sort(all_stop_ids.begin(), all_stop_ids.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });

  std::vector<StopId> required_stop_ids(
      state.required_stops.begin(), state.required_stops.end()
  );
  std::sort(
      required_stop_ids.begin(),
      required_stop_ids.end(),
      [](StopId a, StopId b) { return a.v < b.v; }
  );

  os << "ProblemState{\n";
  os << "  stop_infos=[";
  for (size_t i = 0; i < all_stop_ids.size(); ++i) {
    if (i > 0) os << ", ";
    os << state.stop_infos.at(all_stop_ids[i]).stop_name;
  }
  os << "]\n";
  os << "  required_stops=[";
  for (size_t i = 0; i < required_stop_ids.size(); ++i) {
    if (i > 0) os << ", ";
    os << state.stop_infos.at(required_stop_ids[i]).stop_name;
  }
  os << "]\n";
  os << "  steps=[\n";
  for (const auto& step : steps) {
    os << "    " << state.stop_infos.at(step.origin.stop).stop_name << " -> "
       << state.stop_infos.at(step.destination.stop).stop_name;
    if (step.is_flex) {
      os << " (flex "
         << TimeSinceServiceStart{step.FlexDurationSeconds()}.ToString() << ")";
    } else {
      os << " [" << step.origin.time.ToString() << " -> "
         << step.destination.time.ToString() << "]";
    }
    os << " p=" << step.destination.partition.v << "\n";
  }
  os << "  ]\n";

  std::vector<Step> merged_steps = state.completed.AllMergedSteps();
  std::unordered_set<StopId> completed_origins;
  std::unordered_set<StopId> completed_destinations;
  for (const Step& step : merged_steps) {
    completed_origins.insert(step.origin.stop);
    completed_destinations.insert(step.destination.stop);
  }

  std::vector<StopId> origin_ids(
      completed_origins.begin(), completed_origins.end()
  );
  std::sort(origin_ids.begin(), origin_ids.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });
  std::vector<StopId> dest_ids(
      completed_destinations.begin(), completed_destinations.end()
  );
  std::sort(dest_ids.begin(), dest_ids.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });

  os << "  completed_origins=[";
  for (size_t i = 0; i < origin_ids.size(); ++i) {
    if (i > 0) os << ", ";
    os << state.stop_infos.at(origin_ids[i]).stop_name;
  }
  os << "]\n";
  os << "  completed_destinations=[";
  for (size_t i = 0; i < dest_ids.size(); ++i) {
    if (i > 0) os << ", ";
    os << state.stop_infos.at(dest_ids[i]).stop_name;
  }
  os << "]\n";

  os << "}";
}

ProblemState MakeProblemState(
    StepsAdjacencyList minimal,
    ProblemBoundary boundary,
    std::unordered_set<StopId> stops,
    std::unordered_map<StopId, ProblemStateStopInfo> stop_infos,
    std::unordered_map<StepPartitionId, std::string> step_partition_names,
    std::unordered_map<StopId, PlainEdge> original_edges
) {
  StepPathsAdjacencyList completed = CompleteShortestPathsGraph(minimal, stops);
  // Add END->START edge to complete the cycle for TSP formulation.
  completed.adjacent[boundary.end].push_back(
      {ZeroPath(boundary.end, boundary.start)}
  );
  return ProblemState{
      std::move(minimal),
      std::move(completed),
      boundary,
      std::move(stops),
      std::move(stop_infos),
      std::move(step_partition_names),
      std::move(original_edges),
  };
}

void AddBoundary(
    std::vector<Step>& steps,
    std::unordered_set<StopId>& stops,
    std::unordered_map<StopId, ProblemStateStopInfo>& stop_infos,
    ProblemBoundary bounday
) {
  // ... with 0-duration flex steps START->* and *->END.
  for (StopId stop : stops) {
    steps.push_back(ZeroEdge(bounday.start, stop));
    steps.push_back(ZeroEdge(stop, bounday.end));
  }

  // Add the "START" and "END" vertices.
  stops.insert(bounday.start);
  stop_infos[bounday.start] = ProblemStateStopInfo{GtfsStopId{""}, "START"};
  stops.insert(bounday.end);
  stop_infos[bounday.end] = ProblemStateStopInfo{GtfsStopId{""}, "END"};
}

struct XEdge {
  StopId a;
  StopId b;

  bool operator==(const XEdge& other) const {
    return a == other.a && b == other.b;
  }
};
}  // namespace vats5

template <>
struct std::hash<vats5::XEdge> {
  std::size_t operator()(const vats5::XEdge& e) const {
    std::size_t h1 = std::hash<vats5::StopId>{}(e.a);
    std::size_t h2 = std::hash<vats5::StopId>{}(e.b);
    return h1 ^ (h2 << 1);
  }
};

namespace vats5 {

struct StopGoodness {
  StopId stop;
  int goodness;
};

struct GoodnessResult {
  std::vector<StopGoodness> goodness;
  int num_edges;
};

GoodnessResult ComputeStopGoodness(
    const StepPathsAdjacencyList& minimal_paths_sparse
) {
  std::unordered_set<StopId> all_stops;
  std::unordered_set<XEdge> all_edges;
  std::unordered_map<StopId, std::unordered_set<StopId>> origins_thru;
  std::unordered_map<StopId, std::unordered_set<StopId>> destinations_thru;
  std::unordered_map<StopId, std::unordered_set<XEdge>> edges_thru;
  for (const Path& path : minimal_paths_sparse.AllPaths()) {
    std::vector<StopId> path_stops;
    assert(path.steps.size() > 0);
    path_stops.push_back(path.steps[0].origin.stop);
    for (const Step& step : path.steps) {
      path_stops.push_back(step.destination.stop);
    }
    for (StopId stop : path_stops) {
      XEdge edge{
          path.merged_step.origin.stop, path.merged_step.destination.stop
      };
      all_stops.insert(stop);
      all_edges.insert(edge);
      // Only count edges where stop is intermediate, not an endpoint
      if (stop != edge.a && stop != edge.b) {
        origins_thru[stop].insert(edge.a);
        destinations_thru[stop].insert(edge.b);
        edges_thru[stop].insert(edge);
      }
    }
  }

  std::cout << "Num stops: " << all_stops.size() << "\n";
  std::cout << "Num edges: " << all_edges.size() << "\n";

  std::vector<StopGoodness> goodness;
  for (StopId stop : all_stops) {
    int new_edges = static_cast<int>(origins_thru[stop].size()) +
                    static_cast<int>(destinations_thru[stop].size());
    goodness.push_back(
        StopGoodness{
            stop, static_cast<int>(edges_thru[stop].size()) - new_edges
        }
    );
  }
  std::ranges::sort(goodness, [](const StopGoodness& a, const StopGoodness& b) {
    return a.goodness > b.goodness;
  });

  return GoodnessResult{
      std::move(goodness), static_cast<int>(all_edges.size())
  };
}

ProblemState InitializeProblemState(
    const StepsFromGtfs& steps_from_gtfs,
    const std::unordered_set<StopId> system_stops,
    bool optimize_edges
) {
  // Assign partitions to steps based on their trips.
  std::unordered_map<std::string, StepPartitionId> route_desc_to_step_partition;
  std::unordered_map<StepPartitionId, std::string> step_partition_to_route_desc;
  std::vector<Step> steps_with_partitions = steps_from_gtfs.steps;
  for (Step& step : steps_with_partitions) {
    if (!step.is_flex) {
      auto [it, inserted] = route_desc_to_step_partition.try_emplace(
          steps_from_gtfs.mapping.trip_id_to_route_desc.at(
              step.destination.trip
          ),
          StepPartitionId{static_cast<int>(route_desc_to_step_partition.size())}
      );
      if (inserted) {
        step_partition_to_route_desc[it->second] = it->first;
      }
      step.origin.partition = it->second;
      step.destination.partition = it->second;
    }
  }

  // Compute minimal adj list.
  StepPathsAdjacencyList minimal_paths_sparse = ReduceToMinimalSystemPaths(
      MakeAdjacencyList(steps_with_partitions), system_stops
  );

  if (optimize_edges) {
    int prev_num_edges = std::numeric_limits<int>::max();
    int iteration = 0;
    while (true) {
      GoodnessResult result = ComputeStopGoodness(minimal_paths_sparse);

      std::cout << "Iteration " << iteration << ": " << result.num_edges
                << " edges\n";
      if (result.num_edges >= prev_num_edges) {
        std::cout << "Edges stopped decreasing, stopping.\n";
        break;
      }

      if (result.goodness.empty() || result.goodness[0].goodness <= 0) {
        std::cout << "No positive goodness stops, stopping.\n";
        break;
      }

      std::cout << "Top goodness: "
                << steps_from_gtfs.mapping.stop_id_to_stop_name.at(
                       result.goodness[0].stop
                   )
                << ": " << result.goodness[0].goodness << "\n";

      StopId split_stop = result.goodness[0].stop;
      minimal_paths_sparse = SplitPathsAtStop(minimal_paths_sparse, split_stop);
      prev_num_edges = result.num_edges;
      iteration++;
    }
  }

  StepsAdjacencyList minimal_steps_sparse =
      MakeAdjacencyList(minimal_paths_sparse.AllMergedSteps());

  // Compact minimal adj list and remap stop names.
  CompactStopIdsResult minimal_compact = CompactStopIds(minimal_steps_sparse);

  std::unordered_set<StopId> required_stops;
  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos;
  for (int i = 0; i < minimal_compact.mapping.new_to_original.size(); ++i) {
    StopId stop = StopId{i};
    StopId original_stop = minimal_compact.mapping.new_to_original[i];
    if (system_stops.contains(original_stop)) {
      required_stops.insert(stop);
    }
    auto gtfs_stop_id_it =
        steps_from_gtfs.mapping.stop_id_to_gtfs_stop_id.find(original_stop);
    assert(
        gtfs_stop_id_it != steps_from_gtfs.mapping.stop_id_to_gtfs_stop_id.end()
    );
    auto stop_name_it =
        steps_from_gtfs.mapping.stop_id_to_stop_name.find(original_stop);
    assert(stop_name_it != steps_from_gtfs.mapping.stop_id_to_stop_name.end());
    stop_infos[stop] =
        ProblemStateStopInfo{gtfs_stop_id_it->second, stop_name_it->second};
  }

  int num_actual_stops = minimal_compact.list.NumStops();
  ProblemBoundary boundary{
      .start = StopId{num_actual_stops}, .end = StopId{num_actual_stops + 1}
  };

  std::vector<Step> steps = minimal_compact.list.AllSteps();
  AddBoundary(steps, required_stops, stop_infos, boundary);

  return MakeProblemState(
      MakeAdjacencyList(steps),
      boundary,
      required_stops,
      stop_infos,
      step_partition_to_route_desc,
      {}
  );
}

bool TarelState::operator<(const TarelState& other) const {
  if (stop.v != other.stop.v) return stop.v < other.stop.v;
  return partition.v < other.partition.v;
}

TarelEdgeIntermediateData ComputeTarelIntermediateData(
    const std::vector<Step>& steps
) {
  TarelEdgeIntermediateData data;

  for (const Step& step : steps) {
    TarelState destination_state{
        step.destination.stop, step.destination.partition
    };

    // Preserves sorted-and-minimal property because: The paths within
    // `path_group` are sorted and minimal, they all have the same
    // `step.destination.stop`, no other path groups fom this origin have
    // the same destination stop, and order-preserving partitions preserve
    // sortedness and minimality.
    data.steps_from[step.origin.stop][destination_state].push_back(step);

    auto& arrival_times = data.arrival_times_to[destination_state];
    if (step.is_flex) {
      arrival_times.has_flex = true;
    } else {
      arrival_times.times.push_back(step.destination.time);
    }
  }

  for (const auto& [origin, groups_from] : data.steps_from) {
    for (const auto& [dest, group_from] : groups_from) {
      if (!CheckSortedAndMinimal(group_from)) {
        std::cout << "Not sorted and minimal: " << origin << " -> " << dest
                  << "\n";
        assert(false);
      }
    }
  }

  for (auto& [_, arrival_times] : data.arrival_times_to) {
    std::ranges::sort(arrival_times.times);
    auto [first, last] = std::ranges::unique(arrival_times.times);
    arrival_times.times.erase(first, last);
  }

  return data;
}

std::vector<TarelEdge> BuildTarelEdgesFromIntermediateData(
    const TarelEdgeIntermediateData& data
) {
  std::vector<TarelEdge> edges;
  for (const auto& [origin, arrival_times_to_origin] : data.arrival_times_to) {
    auto it = data.steps_from.find(origin.stop);
    if (it == data.steps_from.end()) {
      continue;
    }
    for (const auto& [dest, steps] : it->second) {
      int weight = std::numeric_limits<int>::max();
      if (arrival_times_to_origin.has_flex) {
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
        for (const TimeSinceServiceStart arrival_time :
             arrival_times_to_origin.times) {
          while (step_idx < steps.size() &&
                 steps[step_idx].origin.time < arrival_time) {
            step_idx += 1;
          }
          if (step_idx >= steps.size()) {
            break;
          }
          int duration =
              steps[step_idx].destination.time.seconds - arrival_time.seconds;
          if (duration < weight) {
            weight = duration;
          }
        }
      }

      if (weight < std::numeric_limits<int>::max()) {
        edges.push_back(
            TarelEdge{
                .origin = origin,
                .destination = dest,
                .weight = weight,
                .original_origins = {origin},
                .original_destinations = {dest},
            }
        );
      }
    }
  }

  return edges;
}

std::vector<TarelEdge> MakeTarelEdges(const StepPathsAdjacencyList& adj) {
  TarelEdgeIntermediateData data =
      ComputeTarelIntermediateData(adj.AllMergedSteps());
  return BuildTarelEdgesFromIntermediateData(data);
}

struct EdgeSignature {
  std::vector<std::pair<TarelState, int>> outgoing;

  bool operator<(const EdgeSignature& other) const {
    return outgoing < other.outgoing;
  }
};

std::vector<TarelEdge> MergeEquivalentTarelStates(
    const std::vector<TarelEdge>& edges
) {
  // Two tarel states are "equivalent" if they have the same `origin.stop`, and
  // they have the same set of `destination`s and they have matching `weights`
  // to each `destination`.

  // Step 1: Build the "edge signature" for each TarelState.
  // The signature is the sorted list of (destination, weight) pairs.
  std::unordered_map<TarelState, EdgeSignature> signatures;
  for (const TarelEdge& edge : edges) {
    signatures[edge.origin].outgoing.push_back({edge.destination, edge.weight});
    // Ensure destination-only states (those without outgoing edges) get an
    // entry so they participate in merging.
    signatures[edge.destination];
  }
  for (auto& [_, signature] : signatures) {
    std::ranges::sort(signature.outgoing, [](const auto& a, const auto& b) {
      if (a.first != b.first) return a.first < b.first;
      return a.second < b.second;
    });
    // auto [out_begin, out_end] = std::ranges::unique(signature.outgoing);
    // signature.outgoing.erase(out_begin, out_end);
  }

  // Step 2: For each stop, group partitions by their edge signature and pick a
  // canonical one.
  std::unordered_map<TarelState, TarelState> canonical_state;

  std::unordered_map<StopId, std::vector<TarelState>> states_by_stop;
  for (const auto& [state, _] : signatures) {
    states_by_stop[state.stop].push_back(state);
  }

  for (const auto& [stop, states] : states_by_stop) {
    std::map<EdgeSignature, TarelState> signature_to_canonical;

    for (const TarelState& ts : states) {
      const auto& signature = signatures.at(ts);
      auto [it, _] = signature_to_canonical.try_emplace(signature, ts);
      canonical_state[ts] = it->second;
    }
  }

  // Step 3: Collect all canonical states and reassign contiguous partition IDs
  // per stop.
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
    std::ranges::sort(states, [](const TarelState& a, const TarelState& b) {
      return a < b;
    });
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
    auto [it, inserted] = merged_edges.try_emplace(
        {new_origin, new_dest}, MergedEdgeData{.weight = edge.weight}
    );
    auto& data = it->second;
    if (!inserted) {
      // TODO: Think harder about whether taking the min merged edge weight
      // makes sense.
      // TODO: Consider whether we have to clear `original_origins` and
      // `original_destinations`.
      if (edge.weight < data.weight) {
        data.weight = edge.weight;
      }
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
    std::ranges::sort(
        sorted_origins,
        [](const TarelState& a, const TarelState& b) { return a < b; }
    );
    std::ranges::sort(
        sorted_destinations,
        [](const TarelState& a, const TarelState& b) { return a < b; }
    );
    sorted_origins.erase(
        std::unique(sorted_origins.begin(), sorted_origins.end()),
        sorted_origins.end()
    );
    sorted_destinations.erase(
        std::unique(sorted_destinations.begin(), sorted_destinations.end()),
        sorted_destinations.end()
    );

    result.push_back(
        TarelEdge{
            .origin = new_origin,
            .destination = new_dest,
            .weight = data.weight,
            .original_origins = std::move(sorted_origins),
            .original_destinations = std::move(sorted_destinations),
        }
    );
  }

  return result;
}

TspGraphData MakeTspGraphEdges(
    const std::vector<TarelEdge>& edges, const ProblemBoundary& boundary
) {
  TspGraphData result;

  // Assign contiguous ids to all TarelStates.
  auto insert_state_if_new = [&](const TarelState& state) {
    auto [_, inserted] = result.id_by_state.try_emplace(
        state, StopId{static_cast<int>(result.state_by_id.size())}
    );
    if (inserted) {
      result.state_by_id.push_back(state);
    }
  };
  for (const TarelEdge& edge : edges) {
    insert_state_if_new(edge.origin);
    insert_state_if_new(edge.destination);
  }

  // Count how many `TarelStates` each stop has.
  for (const TarelState& state : result.state_by_id) {
    result.num_states_by_stop[state.stop] += 1;
  }
  // assert(result.num_states_by_stop[boundary.start] == 1);
  // assert(result.num_states_by_stop[boundary.end] == 1);

  // Add TSP edges: within-stop-cycles.
  result.expected_num_cycle_edges = 0;
  for (const auto& [stop, num_states] : result.num_states_by_stop) {
    for (StepPartitionId partition{0}; partition.v < num_states;
         ++partition.v) {
      StepPartitionId next_partition{(partition.v + 1) % num_states};
      result.tsp_edges.push_back(
          WeightedEdge{
              .origin = result.id_by_state.at(TarelState{stop, partition}),
              .destination =
                  result.id_by_state.at(TarelState{stop, next_partition}),
              .weight_seconds = kCycleEdgeWeight,
          }
      );
    }
    result.expected_num_cycle_edges += num_states - 1;
  }

  // Add TSP edges: inter-stop travel.
  for (const TarelEdge& edge : edges) {
    // Offset origin by -1 for TSP trick.
    TarelState origin = edge.origin;
    origin.partition.v -= 1;
    if (origin.partition.v < 0) {
      origin.partition.v += result.num_states_by_stop.at(origin.stop);
    }
    result.tsp_edges.push_back(
        WeightedEdge{
            .origin = result.id_by_state.at(origin),
            .destination = result.id_by_state.at(edge.destination),
            .weight_seconds = edge.weight,
        }
    );
  }

  return result;
}

std::optional<TspTourResult> SolveTspAndExtractTour(
    const std::vector<TarelEdge>& edges,
    const TspGraphData& graph,
    const ProblemBoundary& boundary,
    std::optional<int> ub,
    std::ostream* tsp_log
) {
  // Solve TSP!!!!
  if (tsp_log) {
    *tsp_log << "Solving TSP with " << graph.state_by_id.size()
             << " vertices and " << graph.tsp_edges.size() << " edges...\n";
  }

  std::optional<int> atsp_ub;
  if (ub.has_value()) {
    atsp_ub = *ub + graph.expected_num_cycle_edges * kCycleEdgeWeight;
  }

  std::optional<ConcordeSolution> solution = SolveTspWithConcorde(
      MakeRelaxedAdjacencyListFromEdges(graph.tsp_edges), atsp_ub, tsp_log
  );
  if (!solution.has_value()) {
    return std::nullopt;
  }

  // Adjust optimal value and rotate tour.
  solution->optimal_value -= graph.expected_num_cycle_edges * kCycleEdgeWeight;
  auto tour_start_it = std::find(
      solution->tour.begin(),
      solution->tour.end(),
      graph.id_by_state.at(TarelState{boundary.start, 0})
  );
  assert(tour_start_it != solution->tour.end());
  std::rotate(solution->tour.begin(), tour_start_it, solution->tour.end());
  assert(
      *(solution->tour.end() - 1) ==
      graph.id_by_state.at(TarelState{boundary.end, 0})
  );
  if (tsp_log) {
    *tsp_log << "Tarel TSP optimal value: "
             << TimeSinceServiceStart{solution->optimal_value}.ToString()
             << "\n";
  }

  // Build lookup for tarel edge weights: (origin, destination) -> weight
  auto FindTarelEdge =
      [&edges](const TarelState& origin, const TarelState& dest) -> TarelEdge {
    for (const TarelEdge& edge : edges) {
      if (edge.origin == origin && edge.destination == dest) {
        return edge;
      }
    }
    assert(false);
  };

  // Validate tour and extract original StopIds.
  std::vector<std::string> tour_errors;
  TspTourResult result;
  result.optimal_value = solution->optimal_value;
  TimeSinceServiceStart accumulated_weight{0};
  TarelState cur_state = TarelState{boundary.start, 0};
  int cur_stop_visited_states = 1;
  for (int tour_idx = 1; tour_idx < solution->tour.size() + 1; ++tour_idx) {
    int cur_stop_num_states = graph.num_states_by_stop.at(cur_state.stop);

    if (tour_idx == solution->tour.size() ||
        graph.state_by_id[solution->tour[tour_idx].v].stop != cur_state.stop) {
      // We're exiting the current stop: validate that we visited all of its
      // states.
      if (cur_stop_visited_states != cur_stop_num_states) {
        tour_errors.push_back(
            "Visited only " + std::to_string(cur_stop_visited_states) + " / " +
            std::to_string(cur_stop_num_states) + " for " +
            std::to_string(cur_state.stop.v)
        );
      }
    }

    if (tour_idx == solution->tour.size()) {
      continue;
    }

    TarelState next_state = graph.state_by_id[solution->tour[tour_idx].v];
    if (next_state.stop == cur_state.stop) {
      if (next_state.partition.v !=
          (cur_state.partition.v + 1) % cur_stop_num_states) {
        tour_errors.push_back(
            "Forbidden transition " + std::to_string(cur_state.partition.v) +
            " -> " + std::to_string(next_state.partition.v) + " for " +
            std::to_string(cur_state.stop.v)
        );
      }
      cur_stop_visited_states += 1;
    } else {
      // Inter-stop transition: compute the tarel edge weight.
      // The TSP edge from cur_state to next_state corresponds to a tarel edge
      // with origin partition = (cur_state.partition + 1) % num_states.
      TarelState tarel_origin = cur_state;
      tarel_origin.partition.v =
          (cur_state.partition.v + 1) % cur_stop_num_states;
      TarelEdge edge = FindTarelEdge(tarel_origin, next_state);
      accumulated_weight.seconds += edge.weight;
      result.cumulative_weights.push_back(accumulated_weight);
      result.tour_edges.push_back(edge);

      cur_stop_visited_states = 1;
      if (cur_state.stop != boundary.start) {
        // Note: Intentionally missing the last stop of `tour` because that is
        // END.
        result.original_stop_tour.push_back(cur_state.stop);
      }
    }

    cur_state = next_state;
  }
  if (!tour_errors.empty()) {
    std::string msg = "SolveTspAndExtractTour: tour errors:\n";
    for (const std::string& error : tour_errors) {
      msg += error + "\n";
    }
    throw InvalidTourStructure(msg);
  }

  return result;
}

std::optional<TspTourResult> ComputeTarelLowerBound(
    const ProblemState& state, std::optional<int> ub, std::ostream* tsp_log
) {
  // Check that every `state.required_stops` appears as both an origin and
  // destination in `state.completed`.
  //
  // This is necessary for correctness because `MakeTarelEdges` only produces
  // states for stops that appear as origins and destinations, and if it misses
  // any stops, then the TSP will simply not visit those stops.
  std::unordered_set<StopId> origins;
  std::unordered_set<StopId> destinations;
  for (const auto& [origin_stop, path_groups] : state.completed.adjacent) {
    for (const auto& path_group : path_groups) {
      if (!path_group.empty()) {
        origins.insert(path_group[0].merged_step.origin.stop);
        destinations.insert(path_group[0].merged_step.destination.stop);
      }
    }
  }
  for (StopId stop : state.required_stops) {
    if (!origins.contains(stop) || !destinations.contains(stop)) {
      return std::nullopt;
    }
  }

  ProblemState reduced_state = state;
  auto edges = MakeTarelEdges(reduced_state.completed);
  auto merged_edges = MergeEquivalentTarelStates(edges);
  auto graph = MakeTspGraphEdges(merged_edges, state.boundary);
  return SolveTspAndExtractTour(
      merged_edges, graph, state.boundary, ub, tsp_log
  );
}

void WriteTarelSummary(
    const ProblemState& state,
    const std::string& dir,
    const std::vector<TarelEdge>& edges,
    const std::unordered_map<StepPartitionId, std::string>& state_descriptions
) {
  std::map<std::string, std::map<std::string, std::vector<TarelEdge>>>
      edge_by_stops;
  for (const TarelEdge& e : edges) {
    edge_by_stops[state.StopName(e.origin.stop)]
                 [state.StopName(e.destination.stop)]
                     .push_back(e);
  }

  for (auto& [origin, edge_by_dest] : edge_by_stops) {
    std::string filename = origin;
    std::erase(filename, '/');
    std::ofstream out(dir + "/" + filename);
    for (auto& [dest, od_edges] : edge_by_dest) {
      std::sort(
          od_edges.begin(),
          od_edges.end(),
          [](const TarelEdge& a, const TarelEdge& b) -> bool {
            return a.weight < b.weight;
          }
      );

      out << dest << "\n";
      for (TarelEdge& e : od_edges) {
        out << "  (";
        for (int i = 0; i < e.original_origins.size(); ++i) {
          if (i > 0) {
            out << ", ";
          }
          out << state_descriptions.at(e.original_origins[i].partition);
        }
        out << ") -> (";
        for (int i = 0; i < e.original_destinations.size(); ++i) {
          if (i > 0) {
            out << ", ";
          }
          out << state_descriptions.at(e.original_destinations[i].partition);
        }
        out << "): " << TimeSinceServiceStart{e.weight}.ToString() << "\n";
      }
      out << "\n";
    }
    out << std::flush;
  }
}

}  // namespace vats5
