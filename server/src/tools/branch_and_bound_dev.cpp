#include <algorithm>
#include <asio/execution/start.hpp>
#include <iostream>
#include <limits>
#include <optional>
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

struct TourStopEntry {
  StopId stop_id;
  TimeSinceServiceStart accumulated_weight;
  // Defined for non-intermediate stops (first stop of each edge), nullopt for intermediate stops.
  std::optional<std::pair<StopId, StopId>> tour_edge;
};

struct SolutionState {
  std::unordered_set<StopId> stops;
  StepsAdjacencyList adj;
  SolutionBoundary boundary;
  SolutionMetadata metadata;

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
  };
}

void ExtendFeasiblePaths(
    std::vector<Step>& feasible_paths,
    const StepPathsAdjacencyList& completed,
    StopId a,
    StopId b) {
  auto path_groups_it = completed.adjacent.find(a);
  if (path_groups_it == completed.adjacent.end()) {
    std::cout << "Forbidden feasible edge?!\n";
    feasible_paths.clear();
    return;
  }
  const std::vector<std::vector<Path>>& path_groups = path_groups_it->second;
  auto path_group_it = std::find_if(path_groups.begin(), path_groups.end(), [&](const auto& path_group) -> bool {
    return path_group.size() > 0 && path_group[0].merged_step.destination_stop == b;
  });
  if (path_group_it == path_groups.end()) {
    std::cout << "Forbidden feasible edge?!\n";
    feasible_paths.clear();
    return;
  }
  std::vector<Step> next_steps;
  for (const Path& path : *path_group_it) {
    next_steps.push_back(path.merged_step);
  }
  feasible_paths = PairwiseMergedSteps(std::move(feasible_paths), std::move(next_steps));
}

struct ExtremeDeltaEdge {
  StopId a;
  StopId b;
  int delta_seconds;
};

ExtremeDeltaEdge ProcessSearchNode(const SolutionState& state) {
  StepPathsAdjacencyList completed =
    ReduceToMinimalSystemPaths(state.adj, state.stops, /*keep_through_other_destination=*/true);

  std::vector<WeightedEdge> relaxed_edges = MakeRelaxedEdges(completed);
  relaxed_edges.push_back(
    WeightedEdge{
      .origin=state.boundary.end,
      .destination=state.boundary.start,
      .weight_seconds=0
    }
  );

  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(relaxed_edges);
  ConcordeSolution solution = SolveTspWithConcorde(relaxed);

  std::cout << "Concorde optimal value " << TimeSinceServiceStart{solution.optimal_value}.ToString() << "\n";
  auto start_it = std::find(solution.tour.begin(), solution.tour.end(), state.boundary.start);
  if (start_it != solution.tour.end()) {
    std::rotate(solution.tour.begin(), start_it, solution.tour.end());
  }
  assert(*(solution.tour.end() - 1) == state.boundary.end);

  // Accumulates relaxed weight along the tour.
  TimeSinceServiceStart accumulated_weight{0};

  // Accumulates actual feasible paths along the tour.
  std::vector<Step> feasible_paths = {ZeroEdge(state.boundary.start, state.boundary.start)};

  // Collect tour stop entries for printing.
  std::vector<TourStopEntry> tour_entries;

  for (int i = 0; i < solution.tour.size() - 1; ++i) {
    StopId a = solution.tour[i];
    StopId b = solution.tour[i + 1];

    ExtendFeasiblePaths(feasible_paths, completed, a, b);

    const auto& path_groups = completed.adjacent.at(a);
    auto path_group_it = std::find_if(path_groups.begin(), path_groups.end(), [&](const auto& path_group) -> bool {
      return path_group.size() > 0 && path_group[0].merged_step.destination_stop == b;
    });
    if (path_group_it == path_groups.end()) {
      std::cout << "Forbidden edge!?\n";
      continue;
    }
    const std::vector<Path>& path_group = *path_group_it;

    const Path* min_duration_path = &path_group[0];
    for (const Path& candidate : path_group) {
      if (candidate.DurationSeconds() < min_duration_path->DurationSeconds()) {
        min_duration_path = &candidate;
      }
    }

    int cur_seconds = min_duration_path->merged_step.origin_time.seconds;
    for (int j = 0; j < min_duration_path->steps.size(); ++j) {
      tour_entries.push_back(TourStopEntry{
        .stop_id = min_duration_path->steps[j].origin_stop,
        .accumulated_weight = accumulated_weight,
        .tour_edge = (j == 0) ? std::make_optional(std::make_pair(a, b)) : std::nullopt
      });

      const Step& step = min_duration_path->steps[j];
      accumulated_weight.seconds += step.destination_time.seconds - cur_seconds;

      // Assert that the steps in the path are actual times relative to the
      // start time. (i.e. flex steps start actually when you get there instead
      // of having a base time of 0).
      if (step.origin_time.seconds < cur_seconds) {
        std::cout << "Assert about to fail! Path steps:\n";
        std::cout << "  merged_step.origin_time: " << min_duration_path->merged_step.origin_time.ToString() << "\n";
        for (const Step& s : min_duration_path->steps) {
          std::cout << "  " << state.StopName(s.origin_stop) << " -> " << state.StopName(s.destination_stop)
            << " " << s.origin_time.ToString() << " -> " << s.destination_time.ToString()
            << " (flex=" << s.is_flex << ")\n";
        }
      }
      assert(step.origin_time.seconds >= cur_seconds);
      cur_seconds = step.destination_time.seconds;
    }
  }
  assert(solution.optimal_value == accumulated_weight.seconds);

  // Find the feasible path with minimum duration.
  TimeSinceServiceStart min_duration_origin_time{0};
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

  // TODO: Explain what I'm doing here. Maybe do it less-hackily.
  std::vector<Step> min_duration_feasible = {ZeroEdge(state.boundary.start, state.boundary.start)};

  // Print tour entries.
  const int align_spacing = 50;
  int prev_diff = 0;
  ExtremeDeltaEdge mde{StopId{-1}, StopId{-1}, std::numeric_limits<int>::max()};
  for (int i = 0; i < tour_entries.size(); ++i) {
    const TourStopEntry& entry = tour_entries[i];

    const std::string indent = entry.tour_edge.has_value() ? "" : "  ";
    std::cout << indent << std::left << std::setw(align_spacing - indent.size())
      << state.StopName(entry.stop_id)
      << entry.accumulated_weight.ToString();
    if (entry.tour_edge.has_value() && min_duration_feasible.size() == 1) {
      std::cout << "  " << TimeSinceServiceStart{min_duration_feasible[0].DurationSeconds()}.ToString();
      int cur_diff = min_duration_feasible[0].DurationSeconds() - entry.accumulated_weight.seconds;
      int delta = cur_diff - prev_diff;
      std::cout << "  " << std::right << std::setw(2) << (delta / 60) << ":" << std::setfill('0') << std::setw(2) << (delta % 60) << std::setfill(' ') << std::left;
      prev_diff = cur_diff;

      // TODO: Add mde calculation to the ->END edge.
      if (i > 0 && delta < mde.delta_seconds) {
        mde.b = entry.tour_edge->first;
        mde.delta_seconds = delta;
      }
    }
    std::cout << "\n";

    if (entry.tour_edge.has_value()) {
      ExtendFeasiblePaths(
        min_duration_feasible,
        completed,
        entry.tour_edge->first,
        entry.tour_edge->second
      );
      if (min_duration_feasible.size() > 1) {
        auto it = std::find_if(min_duration_feasible.begin(), min_duration_feasible.end(),
          [&](const Step& step) { return step.origin_time == min_duration_origin_time; });
        if (it != min_duration_feasible.end()) {
          min_duration_feasible = {*it};
        } else {
          std::cout << "Didn't find step corresponding to min duration feasible path!?\n";
          std::cout << "  Looking for: " << min_duration_origin_time.ToString() << "\n";
          std::cout << "  Available origin times:\n";
          for (const Step& step : min_duration_feasible) {
            std::cout << "    " << step.origin_time.ToString() << "\n";
          }
          min_duration_feasible.clear();
        }
      }
    }
  }
  std::cout << std::left << std::setw(align_spacing)
    << state.StopName(*(solution.tour.end() - 1))
    << accumulated_weight.ToString();
  if (min_duration_feasible.size() == 1) {
    std::cout << "  " << TimeSinceServiceStart{min_duration_feasible[0].DurationSeconds()}.ToString();
    int cur_diff = min_duration_feasible[0].DurationSeconds() - accumulated_weight.seconds;
    int delta = cur_diff - prev_diff;
    std::cout << "  " << std::right << std::setw(2) << (delta / 60) << ":" << std::setfill('0') << std::setw(2) << (delta % 60) << std::setfill(' ') << std::left;
  }
  std::cout << "\n";

  for (int i = 0; i < tour_entries.size(); ++i) {
    const TourStopEntry& entry = tour_entries[i];
    if (entry.tour_edge.has_value() && entry.tour_edge->second == mde.b) {
      mde.a = entry.tour_edge->first;
      break;
    }
  }

  std::cout << "Min delta edge: "
    << state.StopName(mde.a) << " -> "
    << state.StopName(mde.b) << ": "
    << TimeSinceServiceStart{mde.delta_seconds}.ToString() << "\n";

  return mde;
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
std::vector<TarelEdge> MakeTarelEdges(const StepPathsAdjacencyList& adj, std::function<PartitionKey(Step)> partition) {
  // Maps (stop, partition_key) -> StepPartitionId, assigning contiguous ids per stop.
  std::unordered_map<StopId, std::unordered_map<PartitionKey, StepPartitionId>> partition_id_map;

  auto get_partition_id = [&partition_id_map](StopId stop, PartitionKey key) -> StepPartitionId {
    auto& stop_map = partition_id_map[stop];
    auto it = stop_map.find(key);
    if (it != stop_map.end()) {
      return it->second;
    }
    StepPartitionId new_id{static_cast<int>(stop_map.size())};
    stop_map[key] = new_id;
    return new_id;
  };

  std::unordered_set<TarelState> all_tarel_states;

  // steps_from[x] is all steps from x, grouped by dest stop and step partition.
  // Within each group, the steps are sorted and minimal.
  std::unordered_map<StopId, std::unordered_map<TarelState, std::vector<Step>>> steps_from;

  // arrival_times_to[(x, p)] is all partition-p arrival times to stop x.
  // Note that I've been careful to put `ArrivalTimesScheduled` as the first
  // alternative so that when you []-access a value that doesn't exist yet, it
  // starts as an empty `ArrivalTimesScheduled`.
  // After we have constructed this, times in each value are sorted ascending
  // and unique.
  std::unordered_map<TarelState, std::variant<ArrivalTimesScheduled, ArrivalTimesFlex>> arrival_times_to;

  for (const auto& [origin_stop, path_groups] : adj.adjacent) {
    for (const auto& path_group : path_groups) {
      for (const Path& path : path_group) {
        const Step& step = path.merged_step;
        PartitionKey pk = partition(step);

        StepPartitionId origin_partition = get_partition_id(step.origin_stop, pk);
        StepPartitionId dest_partition = get_partition_id(step.destination_stop, pk);

        TarelState origin_ts{step.origin_stop, origin_partition};
        TarelState dest_ts{step.destination_stop, dest_partition};

        all_tarel_states.insert(origin_ts);
        all_tarel_states.insert(dest_ts);

        // Preserves sorted-and-minimal property because: The paths within
        // `path_group` are sorted and minimal, they all have the same
        // `step.destination_stop`, no other path groups fom this origin have
        // the same destination stop, and order-preserving partitions preserve
        // sortedness and minimality.
        steps_from[step.origin_stop][dest_ts].push_back(step);

        auto& arrival_times = arrival_times_to[dest_ts];
        if (step.is_flex || std::holds_alternative<ArrivalTimesFlex>(arrival_times)) {
          arrival_times = ArrivalTimesFlex();
        } else {
          std::get<ArrivalTimesScheduled>(arrival_times).times.push_back(step.destination_time);
        }
      }
    }
  }

  for (const auto& [_, groups_from] : steps_from) {
    for (const auto& [_, group_from] : groups_from) {
      assert(CheckSortedAndMinimal(group_from));
    }
  }

  for (auto& [_, times_to] : arrival_times_to) {
    if (std::holds_alternative<ArrivalTimesScheduled>(times_to)) {
      std::vector<TimeSinceServiceStart>& times = std::get<ArrivalTimesScheduled>(times_to).times;
      std::ranges::sort(times);
      auto [first, last] = std::ranges::unique(times);
      times.erase(first, last);
    }
  }

  std::vector<TarelEdge> result;
  for (const auto& [origin_vertex, arrival_times_to_origin] : arrival_times_to) {
    for (const auto& [dest_vertex, steps] : steps_from[origin_vertex.stop]) {
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
        result.push_back(TarelEdge{
          .origin=origin_vertex,
          .destination=dest_vertex,
          .weight=weight,
        });
      }
    }
  }

  return result;
}

std::vector<TarelEdge> MergeEquivalentTarelStates(const std::vector<TarelEdge>& edges) {
  // Two tarel states are "equivalent" if they have the same `origin.stop`, and
  // they have the same set of `destination`s and they have matching `weights`
  // to each `destination`.

  // Step 1: Build the "edge signature" for each TarelState.
  // The signature is the sorted list of (destination, weight) pairs.
  std::unordered_map<TarelState, std::vector<std::pair<TarelState, int>>> outgoing_edges;
  for (const TarelEdge& edge : edges) {
    outgoing_edges[edge.origin].push_back({edge.destination, edge.weight});

    // Insert empty-vector for the destination if it doesn't already have a
    // value. This ensures that all states, even those that never appear as
    // `origin`, appear in `outgoing_edges`.
    outgoing_edges.try_emplace(edge.destination);
  }
  for (auto& [_, edge_list] : outgoing_edges) {
    std::ranges::sort(edge_list, [](const auto& a, const auto& b) {
      if (a.first != b.first) return a.first < b.first;
      return a.second < b.second;
    });
  }

  // Step 2: For each stop, group partitions by their edge signature and pick a canonical one.
  std::unordered_map<TarelState, TarelState> canonical_state;

  std::unordered_map<StopId, std::vector<TarelState>> states_by_stop;
  for (const auto& [origin, _] : outgoing_edges) {
    states_by_stop[origin.stop].push_back(origin);
  }

  for (const auto& [stop, states] : states_by_stop) {
    std::map<std::vector<std::pair<TarelState, int>>, TarelState> signature_to_canonical;

    for (const TarelState& ts : states) {
      const auto& signature = outgoing_edges.at(ts);
      auto [it, _] = signature_to_canonical.try_emplace(signature, ts);
      canonical_state[ts] = it->second;
    }
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

  std::vector<TarelEdge> result;
  for (const TarelEdge& edge : edges) {
    TarelState canonical_origin = canonical_state[edge.origin];
    // Only emit edges from canonical origins (skip duplicates).
    if (canonical_origin != edge.origin) {
      continue;
    }
    TarelState new_origin = final_state(edge.origin);
    TarelState new_dest = final_state(edge.destination);
    result.push_back(TarelEdge{
      .origin = new_origin,
      .destination = new_dest,
      .weight = edge.weight,
    });
  }

  return result;
}

constexpr int kCycleEdgeWeight = -1000;

void SolveTarelTspInstance(const std::vector<TarelEdge>& edges, const SolutionState& state) {
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
  assert(num_states_by_stop.at(state.boundary.start) == 1);
  assert(num_states_by_stop.at(state.boundary.end) == 1);

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

  // Validate tour and extract original StopIds.
  std::vector<std::string> tour_errors;
  std::vector<StopId> original_stop_tour;
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

  for (const StopId stop : original_stop_tour) {
    std::cout << state.StopName(stop) << "\n";
  }
}

void ProcessSearchNodeWithTarel(const SolutionState& state) {

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

    auto tarel_es = MakeTarelEdges(completed, std::function<std::pair<StopId, bool>(Step)>([](const Step& s) {
      return std::make_pair(s.origin_stop, s.is_flex);
    }));
    // auto tarel_es = MakeTarelEdges(completed, std::function<StopId(Step)>([](const Step& s) {
    //   return s.origin_stop;
    // }));
    // auto tarel_es = MakeTarelEdges(completed, std::function<StopId(Step)>([](const Step& s) {
    //   return StopId{0};
    // }));
    auto tarel_es_2 = MergeEquivalentTarelStates(tarel_es);
    SolveTarelTspInstance(tarel_es_2, state);
    // TODO: Think about whether it's possible for there to be a situation where
    // merging multiple times makes progress each time.
    // ... it seems like merging states could make it be so that some states who
    // previously had distinct dest states could now have the same dest states.
    // But:
    // - Maybe there's a reason why anything that ends up with same dest states must have started with same dest states anyways.
    // - Or not quite, but where there has to be a very unlikely coincidence of unrelated weights for dest states to get merged in such a way.
    // MergeEquivalentTarelStates(tarel_es_2);

    return 0;

    for (int i = 0; i < 10; ++i) {
      std::cout << "Forbid step " << i << "\n";
      ExtremeDeltaEdge mde = ProcessSearchNode(state);
      std::cout << "\n";

      // Forbid the ExtremeDeltaEdge.
      std::vector<Step> steps_with_forbid = state.adj.AllSteps();
      std::erase_if(steps_with_forbid, [&](const Step& step) -> bool {
        return step.origin_stop == mde.a && step.destination_stop == mde.b;
      });
      state.adj = MakeAdjacencyList(steps_with_forbid);
    }

    return 0;
}
