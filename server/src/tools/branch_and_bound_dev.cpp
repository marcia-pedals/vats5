#include <algorithm>
#include <asio/execution/start.hpp>
#include <iostream>
#include <limits>
#include <optional>
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
};

struct SolutionState {
  std::unordered_set<StopId> stops;
  StepsAdjacencyList adj;
  SolutionBoundary boundary;
  SolutionMetadata metadata;
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

ExtremeDeltaEdge ProcessSearchNode(const SolutionState& state) {
  auto StopName = [&](StopId stop) -> std::string {
    return state.metadata.stop_names[stop.v];
  };

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
          std::cout << "  " << StopName(s.origin_stop) << " -> " << StopName(s.destination_stop)
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
      << StopName(entry.stop_id)
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
    << StopName(*(solution.tour.end() - 1))
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
    << StopName(mde.a) << " -> "
    << StopName(mde.b) << ": "
    << TimeSinceServiceStart{mde.delta_seconds}.ToString() << "\n";

  return mde;
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
