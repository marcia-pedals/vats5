#include <algorithm>
#include <asio/execution/start.hpp>
#include <iostream>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>

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
  // Transfer time used for this edge (only for edges after the first).
  std::optional<int> transfer_time_seconds;
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

struct MinTransferTime {
  StopId a;
  StopId b;
  StopId c;
  int seconds;
};

// Helper that processes a tour (already rotated with start first) and prints details.
// Returns the edge with minimum delta between feasible and relaxed times.
// If transfer_times is provided, adds transfer times to accumulated_weight.
ExtremeDeltaEdge ProcessTour(
    const SolutionState& state,
    const StepPathsAdjacencyList& completed,
    const ConcordeSolution& solution,
    const std::vector<MinTransferTime>& transfer_times = {}) {
  const std::vector<StopId>& tour = solution.tour;

  // Accumulates relaxed weight along the tour.
  TimeSinceServiceStart accumulated_weight{0};

  // Accumulates actual feasible paths along the tour.
  std::vector<Step> feasible_paths = {ZeroEdge(state.boundary.start, state.boundary.start)};

  // Collect tour stop entries for printing.
  std::vector<TourStopEntry> tour_entries;

  for (int i = 0; i < tour.size() - 1; ++i) {
    StopId a = tour[i];
    StopId b = tour[i + 1];

    // Look up transfer time if available (for edges after the first one).
    std::optional<int> transfer_time_seconds;
    if (i > 0 && !transfer_times.empty()) {
      StopId prev_a = tour[i - 1];
      auto it = std::find_if(transfer_times.begin(), transfer_times.end(),
        [&](const MinTransferTime& t) {
          return t.a == prev_a && t.b == a && t.c == b;
        });
      if (it != transfer_times.end()) {
        transfer_time_seconds = it->seconds;
      }
    }

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
        .tour_edge = (j == 0) ? std::make_optional(std::make_pair(a, b)) : std::nullopt,
        .transfer_time_seconds = (j == 0) ? transfer_time_seconds : std::nullopt,
      });

      const Step& step = min_duration_path->steps[j];
      // accumulated_weight.seconds += step.destination_time.seconds - cur_seconds;

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

    if (transfer_time_seconds.has_value()) {
      accumulated_weight.seconds += *transfer_time_seconds;
    }
  }
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
      << state.StopName(entry.stop_id);
    // Print transfer time column (or blank if not applicable).
    if (entry.transfer_time_seconds.has_value()) {
      std::cout << std::right << std::setw(5) << (*entry.transfer_time_seconds / 60) << ":"
                << std::setfill('0') << std::setw(2) << (*entry.transfer_time_seconds % 60)
                << std::setfill(' ') << std::left << "  ";
    } else {
      std::cout << "          ";
    }
    std::cout << entry.accumulated_weight.ToString();
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
    << state.StopName(*(tour.end() - 1))
    << "          "  // blank transfer time column
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

  // std::cout << "Min delta edge: "
  //   << state.StopName(mde.a) << " -> "
  //   << state.StopName(mde.b) << ": "
  //   << TimeSinceServiceStart{mde.delta_seconds}.ToString() << "\n";
  std::cout << "Concorde - Accumulated = " << solution.optimal_value - accumulated_weight.seconds << "\n";

  return mde;
}

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

  return ProcessTour(state, completed, solution);
}

MinTransferTime ComputeMinTransferTime(const StepsAdjacencyList& adj, const StopId a, const StepGroup& ab, const StepGroup& bc) {
  MinTransferTime min_transfer{
    .a=a,
    .b=ab.destination_stop,
    .c=bc.destination_stop,
    .seconds=std::numeric_limits<int>::max(),
  };

  const std::span<const AdjacencyListStep> ab_steps = adj.GetSteps(ab);
  const std::span<const AdjacencyListStep> bc_steps = adj.GetSteps(bc);

  if (bc.flex_step.has_value()) {
    min_transfer.seconds = std::min(
      min_transfer.seconds,
      bc.flex_step->FlexDurationSeconds()
    );
  }
  if (ab.flex_step.has_value()) {
    for (const AdjacencyListStep& step : bc_steps) {
      min_transfer.seconds = std::min(
        min_transfer.seconds,
        step.destination_time.seconds - step.origin_time.seconds
      );
    }
  }

  size_t j = 0;
  for (size_t i = 0; i < ab_steps.size(); ++i) {
    while (j < bc_steps.size() && bc_steps[j].origin_time < ab_steps[i].destination_time) {
      ++j;
    }

    if (j >= bc_steps.size()) {
      break;
    }
    int transfer = bc_steps[j].destination_time.seconds - ab_steps[i].destination_time.seconds;
    min_transfer.seconds = std::min(min_transfer.seconds, transfer);
  }

  return min_transfer;
}

// TODO: Think harder about this and whether it is correct.
void PruneUselessFlexSteps(
  StepPathsAdjacencyList& adj,
  const TimeSinceServiceStart min_origin_time,
  const TimeSinceServiceStart max_destination_time
) {
  int num_pruned = 0;
  int num_flex = 0;
  for (auto& [origin, groups] : adj.adjacent) {
    for (std::vector<Path>& group : groups) {
      if (group.size() == 0 || !group[0].merged_step.is_flex) {
        continue;
      }
      num_flex += 1;
      int flex_duration = group[0].merged_step.FlexDurationSeconds();

      TimeSinceServiceStart confirmed_departure_without_flex = min_origin_time;
      int i = 1;
      while (confirmed_departure_without_flex.seconds <= max_destination_time.seconds - flex_duration) {
        if (
          i < group.size() &&
          group[i].merged_step.destination_time.seconds <= confirmed_departure_without_flex.seconds + flex_duration
        ) {
          if (group[i].merged_step.origin_time > confirmed_departure_without_flex) {
            confirmed_departure_without_flex = group[i].merged_step.origin_time;
          }
          i += 1;
        } else {
          break;
        }
      }

      if (confirmed_departure_without_flex.seconds >= max_destination_time.seconds - flex_duration) {
        // Flex step is useless, prune it!
        group.erase(group.begin());
        num_pruned += 1;
      }
    }
  }
  std::cout << "pruned flex steps: " << num_pruned << "/" << num_flex << "\n";
}

struct GroupedTxEdge {
  std::vector<StopId> as;
  std::unordered_map<StopId, int> tx_times_by_c;
};

void InvestigateTransferTimes(const SolutionState& state) {
  StepPathsAdjacencyList completed = ReduceToMinimalSystemPaths(
    state.adj, state.stops, /*keep_through_other_destination=*/true);
  // PruneUselessFlexSteps(completed, TimeSinceServiceStart{6 * 3600}, TimeSinceServiceStart{22 * 3600});
  StepsAdjacencyList adj = MakeAdjacencyList(completed.AllMergedSteps());

  // Number of stop-to-stop edges.
  int num_edges = adj.groups.size();
  std::cout << "num_edges: " << num_edges << "\n";

  std::vector<MinTransferTime> transfer_times;
  for (StopId a = StopId{0}; a.v < adj.group_offsets.size(); ++a.v) {
    for (const StepGroup& ab : adj.GetGroups(a)) {
      StopId b = ab.destination_stop;
      for (const StepGroup& bc : adj.GetGroups(b)) {
        StopId c = bc.destination_stop;
        transfer_times.push_back(ComputeMinTransferTime(adj, a, ab, bc));
      }
    }
  }

  std::sort(transfer_times.begin(), transfer_times.end(), [](const MinTransferTime& x, const MinTransferTime& y) -> bool {
    return x.seconds > y.seconds;
  });

  int num_feasible_zero = 0;
  int num_feasible_positive = 0;
  int num_infeasible = 0;
  for (const MinTransferTime& transfer_time : transfer_times) {
    if (transfer_time.seconds == std::numeric_limits<int>::max()) {
      num_infeasible += 1;
      continue;
    } else if (transfer_time.seconds == 0) {
      num_feasible_zero += 1;
      continue;
    }
    num_feasible_positive += 1;
    // std::cout
    //   << state.StopName(transfer_time.a) <<  "->"
    //   << state.StopName(transfer_time.b) <<  "->"
    //   << state.StopName(transfer_time.c) <<  ": "
    //   << TimeSinceServiceStart{transfer_time.seconds}.ToString() << "\n"
    //   << "  " << transfer_time.b_arrive.ToString() << " -> " << transfer_time.b_depart.ToString() << "\n";
  }
  std::cout << "num_feasible_zero: " << num_feasible_zero << "\n";
  std::cout << "num_feasible_positive: " << num_feasible_positive << "\n";
  std::cout << "num_infeasible: " << num_infeasible << "\n";

  std::unordered_map<StopId, std::unordered_map<StopId, std::vector<MinTransferTime>>> tts_by_ba;
  for (const MinTransferTime& transfer_time : transfer_times) {
    tts_by_ba[transfer_time.b][transfer_time.a].push_back(transfer_time);
  }

  int num_groups = 0;
  std::unordered_map<StopId, std::vector<GroupedTxEdge>> groups_by_b;
  for (const auto& [b, tts_by_a] : tts_by_ba) {
    std::vector<GroupedTxEdge>& groups = groups_by_b[b];
    for (const auto& [a, tts] : tts_by_a) {
      // First check if we match any existing groups.
      bool group_match = false;
      for (GroupedTxEdge& group : groups) {
        if (tts.size() != group.tx_times_by_c.size()) {
          continue;
        }

        group_match = true;
        for (const MinTransferTime& tt : tts) {
          auto group_it = group.tx_times_by_c.find(tt.c);
          if (group_it == group.tx_times_by_c.end() || group_it->second != tt.seconds) {
            group_match = false;
            break;
          }
        }

        if (group_match) {
          group.as.push_back(a);
          break;
        }
      }
      if (group_match) {
        continue;
      }

      // We don't match: add our own group.
      GroupedTxEdge group{
        .as={a},
        .tx_times_by_c={},
      };
      for (const MinTransferTime& tt : tts) {
        group.tx_times_by_c[tt.c] = tt.seconds;
      }
      groups.push_back(std::move(group));
    }

    num_groups += groups.size();
  }

  // Add groups for START and END. (The above loop doesn't add them because they
  // never appear as intermediate stops in paths a->b->c).
  GroupedTxEdge start_group, end_group;
  // START has edges to all non-boundary stops, and END has edges from all
  // non-boundary stops, so add this info to their grouped edges accordingly.
  for (StopId a = StopId{0}; a.v < adj.group_offsets.size(); ++a.v) {
    if (a == state.boundary.start || a == state.boundary.end) {
      continue;
    }
    start_group.tx_times_by_c[a] = 0;
    end_group.as.push_back(a);
  }
  groups_by_b[state.boundary.start].push_back(start_group);
  groups_by_b[state.boundary.end].push_back(end_group);
  num_groups += 2;

  std::cout << "num_groups: " << num_groups << "\n";

  std::vector<WeightedEdge> txr_edges;

  // First make mappings between naive and txr stops and add the within-stop-cycle edges.
  struct TxrStopInfo {
    StopId naive_stop;
    int group_idx;
  };
  StopId next_txr_stop{0};
  std::unordered_map<StopId, std::vector<StopId>> naive_to_txr_stops;
  std::unordered_map<StopId, TxrStopInfo> txr_to_naive_stop;
  const int cycle_edge_weight = -1000;
  int expected_num_cycle_edges = 0;
  for (const auto& [b, groups] : groups_by_b) {
    std::vector<StopId>& b_txr_stops = naive_to_txr_stops[b];
    for (int group_idx = 0; group_idx < groups.size(); ++group_idx) {
      b_txr_stops.push_back(next_txr_stop);
      txr_to_naive_stop[next_txr_stop] = TxrStopInfo{.naive_stop = b, .group_idx = group_idx};
      next_txr_stop.v += 1;
    }

    for (int group_idx = 0; group_idx < b_txr_stops.size(); ++group_idx) {
      txr_edges.push_back(WeightedEdge{
        .origin=b_txr_stops[group_idx],
        .destination=b_txr_stops[(group_idx + 1) % b_txr_stops.size()],
        .weight_seconds=cycle_edge_weight,
      });
    }
    expected_num_cycle_edges += b_txr_stops.size() - 1;
  }

  // Next build all the tx+travel edges for the txr graph.
  for (const auto& [b, groups] : groups_by_b) {
    for (int group_idx = 0; group_idx < groups.size(); ++group_idx) {
      const GroupedTxEdge& group = groups[group_idx];
      for (const auto& [c, tx_time] : group.tx_times_by_c) {
        // TODO: Could build a mapping to make this lookup more efficient.
        const std::vector<GroupedTxEdge>& c_groups = groups_by_b.at(c);
        auto c_group_it = std::find_if(c_groups.begin(), c_groups.end(), [&](const GroupedTxEdge& edge) -> bool {
          return std::find(edge.as.begin(), edge.as.end(), b) != edge.as.end();
        });
        assert(c_group_it != c_groups.end());
        int bc_group_idx = c_group_it - c_groups.begin();

        txr_edges.push_back(WeightedEdge{
          // Offset origin by -1 for TSP trick.
          .origin=naive_to_txr_stops.at(b)[group_idx == 0 ? groups.size() - 1 : group_idx - 1],
          .destination=naive_to_txr_stops.at(c)[bc_group_idx],
          .weight_seconds=tx_time,
        });
      }
    }
  }

  // Finally add the END -> START edge to the txr graph and build the adjacency list.
  const std::vector<StopId>& start_txr_stops = naive_to_txr_stops.at(state.boundary.start);
  assert(start_txr_stops.size() == 1);
  const std::vector<StopId>& end_txr_stops = naive_to_txr_stops.at(state.boundary.end);
  assert(end_txr_stops.size() == 1);
  txr_edges.push_back(
    WeightedEdge{
      .origin=end_txr_stops[0],
      .destination=start_txr_stops[0],
      .weight_seconds=0
    }
  );

  RelaxedAdjacencyList txr_adj = MakeRelaxedAdjacencyListFromEdges(txr_edges);

  ConcordeSolution solution = SolveTspWithConcorde(txr_adj);
  solution.optimal_value -= expected_num_cycle_edges * cycle_edge_weight;
  std::cout
    << "Concorde optimal value "
    << TimeSinceServiceStart{solution.optimal_value}.ToString() << "\n";

  // Rotate the tour so that "start" vertex is first.
  StopId start_txr_stop = start_txr_stops[0];
  auto start_it = std::find(solution.tour.begin(), solution.tour.end(), start_txr_stop);
  assert(start_it != solution.tour.end());
  std::rotate(solution.tour.begin(), start_it, solution.tour.end());

  // Check that the tour cycles through all vertices of a naive stop before going to the next.
  StopId current_naive_stop = txr_to_naive_stop.at(solution.tour[0]).naive_stop;
  int current_group_idx = txr_to_naive_stop.at(solution.tour[0]).group_idx;
  int expected_visits_for_stop = naive_to_txr_stops.at(current_naive_stop).size();
  int visits_in_current_stop = 0;

  for (size_t i = 0; i < solution.tour.size(); ++i) {
    StopId txr_stop = solution.tour[i];
    const TxrStopInfo& info = txr_to_naive_stop.at(txr_stop);

    if (info.naive_stop == current_naive_stop) {
      // Still at the same naive stop - check that group_idx is incrementing by 1 (mod size).
      int expected_group_idx = (current_group_idx + visits_in_current_stop) % expected_visits_for_stop;
      if (info.group_idx != expected_group_idx) {
        std::cout << "ERROR at tour position " << i << ": at naive stop " << current_naive_stop.v
                  << ", expected group_idx " << expected_group_idx << " but got " << info.group_idx << "\n";
      }
      visits_in_current_stop++;
    } else {
      // Moved to a new naive stop - check that we visited all vertices of the previous stop.
      if (visits_in_current_stop != expected_visits_for_stop) {
        std::cout << "ERROR: left naive stop " << current_naive_stop.v << " after visiting "
                  << visits_in_current_stop << " vertices, but expected " << expected_visits_for_stop << "\n";
      }
      // Reset for the new naive stop.
      current_naive_stop = info.naive_stop;
      current_group_idx = info.group_idx;
      expected_visits_for_stop = naive_to_txr_stops.at(current_naive_stop).size();
      visits_in_current_stop = 1;
    }
  }

  // Check the final stop was fully visited.
  if (visits_in_current_stop != expected_visits_for_stop) {
    std::cout << "ERROR: ended at naive stop " << current_naive_stop.v << " after visiting "
              << visits_in_current_stop << " vertices, but expected " << expected_visits_for_stop << "\n";
  }

  std::cout << "Tour validation complete.\n";

  // Convert txr tour to naive tour (deduplicate consecutive same naive stops).
  std::vector<StopId> naive_tour;
  for (StopId txr_stop : solution.tour) {
    StopId naive_stop = txr_to_naive_stop.at(txr_stop).naive_stop;
    if (naive_tour.empty() || naive_tour.back() != naive_stop) {
      naive_tour.push_back(naive_stop);
    }
  }

  std::cout << "\n=== ProcessTour for InvestigateTransferTimes ===\n";
  ProcessTour(state, completed, ConcordeSolution{naive_tour, solution.optimal_value}, transfer_times);
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

    InvestigateTransferTimes(state);

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
