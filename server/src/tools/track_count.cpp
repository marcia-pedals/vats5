#include <CLI/CLI.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <unordered_map>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

StopId FindStopByGtfsId(
    const ProblemState& problem, const std::string& gtfs_stop_id_str
) {
  const GtfsStopId target_gtfs_id{gtfs_stop_id_str};
  for (const auto& [stop_id, info] : problem.stop_infos) {
    if (info.gtfs_stop_id == target_gtfs_id) {
      return stop_id;
    }
  }
  throw std::runtime_error(
      "Stop " + gtfs_stop_id_str + " not found in stop_infos"
  );
}

void ListStops(const ProblemState& problem) {
  for (const auto& [stop_id, info] : problem.stop_infos) {
    if (stop_id == problem.boundary.start || stop_id == problem.boundary.end) {
      continue;
    }
    std::cout << info.gtfs_stop_id.v << " - " << info.stop_name << "\n";
  }
}

void PrintDepartures(const ProblemState& problem, StopId stop) {
  struct Departure {
    TimeSinceServiceStart time;
    std::string dest_name;
  };
  std::vector<Departure> departures;

  for (const auto& group : problem.minimal.GetGroups(stop)) {
    const auto& dest_name =
        problem.stop_infos.at(group.destination_stop).stop_name;
    for (const auto& step : problem.minimal.GetSteps(group)) {
      departures.push_back({step.origin_time, dest_name});
    }
  }

  std::ranges::sort(departures, {}, &Departure::time);

  std::cout << "Departure times from " << problem.stop_infos.at(stop).stop_name
            << ":\n";
  for (size_t i = 0; i < departures.size();) {
    size_t j = i;
    while (j < departures.size() && departures[j].time == departures[i].time) {
      ++j;
    }
    std::cout << "  " << departures[i].time.ToString() << " -> ";
    for (size_t k = i; k < j; ++k) {
      if (k > i) std::cout << ", ";
      std::cout << departures[k].dest_name;
    }
    std::cout << "\n";
    i = j;
  }
}

struct StopExitStats {
  int min_exit_seconds;
  int max_exit_seconds;
};

struct ExitStats {
  std::unordered_map<StopId, StopExitStats> individual;
  std::vector<int> sorted_min_exit_seconds;
  std::vector<int> sorted_max_exit_seconds;
};

ExitStats ComputeExitStats(
    const StepsAdjacencyList& completed, const ProblemBoundary& boundary
) {
  ExitStats result;
  for (StopId s{0}; s.v < completed.NumStops(); ++s.v) {
    auto groups = completed.GetGroups(s);
    if (s == boundary.start || s == boundary.end || groups.size() == 0) {
      continue;
    }
    StopExitStats& stats = result.individual[s];
    stats.min_exit_seconds = std::numeric_limits<int>::max();
    stats.max_exit_seconds = 0;
    auto VisitStep = [&](const AdjacencyListStep& step) {
      int duration = step.destination_time.seconds - step.origin_time.seconds;
      stats.min_exit_seconds = std::min(stats.min_exit_seconds, duration);
      stats.max_exit_seconds = std::max(stats.max_exit_seconds, duration);
    };
    for (const StepGroup& g : groups) {
      if (g.destination_stop == boundary.start ||
          g.destination_stop == boundary.end) {
        continue;
      }
      if (g.flex_step.has_value()) {
        VisitStep(*g.flex_step);
      }
      for (const AdjacencyListStep& step : completed.GetSteps(g)) {
        VisitStep(step);
      }
    }
    result.sorted_min_exit_seconds.push_back(stats.min_exit_seconds);
    result.sorted_max_exit_seconds.push_back(stats.max_exit_seconds);
  }

  std::ranges::sort(result.sorted_min_exit_seconds);
  std::ranges::sort(result.sorted_max_exit_seconds, std::greater{});

  for (int i = 0; i < result.sorted_min_exit_seconds.size(); ++i) {
    std::cout << TimeSinceServiceStart{result.sorted_min_exit_seconds[i]}
              << "  "
              << TimeSinceServiceStart{result.sorted_max_exit_seconds[i]}
              << "\n";
  }

  return result;
}

struct StepState {
  std::vector<TimeSinceServiceStart> arrival_times;

  void SortAndDedupe() {
    std::sort(arrival_times.begin(), arrival_times.end());
    auto it = std::unique(arrival_times.begin(), arrival_times.end());
    arrival_times.erase(it, arrival_times.end());
  }
};

std::vector<std::unordered_map<StopId, StepState>> ComputeStepStates(
    const StepsAdjacencyList& completed,
    const RequiredStops& required,
    const ProblemBoundary& boundary,
    StopId s0,
    TimeSinceServiceStart t0,
    int lb_rel,
    int ub_rel
) {
  assert(required.size() >= 4);

  // TODO: Consider whether we can tighten these up enough to be useful.
  // ExitStats exit_stats = ComputeExitStats(graph, problem.boundary);
  // // We use all the exits except for 1 to do all the steps.
  // assert(exit_stats.sorted_max_exit_seconds.size() - 1 ==
  // problem.required.size() - 3);
  // assert(exit_stats.sorted_max_exit_seconds.size() ==
  // exit_stats.sorted_min_exit_seconds.size());

  TimeSinceServiceStart t_lb = t0;
  if (lb_rel >= 0) {
    t_lb.seconds = t0.seconds + lb_rel;
  }

  TimeSinceServiceStart t_ub;
  if (ub_rel < 0) {
    for (const Step& step : completed.AllSteps()) {
      t_ub = std::max(t_ub, step.destination.time);
    }
  } else {
    t_ub.seconds = t0.seconds + ub_rel;
  }

  // step_states[0]: Base case, s0@t0
  // step_states[k]: All possible states after making k steps.
  //
  // Note that at step_states[k] we have visited k+1 stops, so if there are n
  // stops then we need k+1=n-2 <=> k=n-3. (-2 because we ignore boundary
  // stops).
  // TODO: Handle boundary stops more elegantly.
  std::vector<std::unordered_map<StopId, StepState>> step_states;

  {
    std::unordered_map<StopId, StepState> step_state_0;
    step_state_0[s0].arrival_times.push_back(t0);
    step_states.push_back(std::move(step_state_0));
  }

  for (int k = 1; k <= static_cast<int>(required.size()) - 3; ++k) {
    step_states.push_back({});
    std::unordered_map<StopId, StepState>& step_state_k =
        *(step_states.end() - 1);
    const std::unordered_map<StopId, StepState>& step_state_kminus1 =
        *(step_states.end() - 2);

    for (const auto& [s_cur, step_state_cur] : step_state_kminus1) {
      for (const StepGroup& g_next : completed.GetGroups(s_cur)) {
        StopId s_next = g_next.destination_stop;
        if (s_next == boundary.start || s_next == boundary.end) {
          continue;
        }

        for (const TimeSinceServiceStart& t_cur :
             step_state_cur.arrival_times) {
          // TODO: Deal with flex steps.
          std::span<const AdjacencyListStep> group_steps =
              completed.GetSteps(g_next);
          size_t t_next_i = FindDepartureAtOrAfter(completed, g_next, t_cur);
          if (t_next_i >= group_steps.size()) {
            continue;
          }
          TimeSinceServiceStart t_next = group_steps[t_next_i].destination_time;
          if (t_next > t_ub) {
            continue;
          }
          step_state_k[s_next].arrival_times.push_back(t_next);
        }
      }
    }

    for (auto& [_, step_state] : step_state_k) {
      step_state.SortAndDedupe();
    }
  }

  // Anything arriving at the end before t_lb is too good to be true.
  for (auto& [s, step_state] : step_states.back()) {
    std::erase_if(
        step_state.arrival_times,
        [&](const TimeSinceServiceStart& t) { return t < t_lb; }
    );
  }

  // Now go backwardsly and filter any arrival times that do not lead to actual
  // arrival times that we have.
  for (int k = static_cast<int>(step_states.size()) - 2; k >= 0; --k) {
    std::unordered_map<StopId, StepState>& step_state_k = step_states[k];
    const std::unordered_map<StopId, StepState>& step_state_kplus1 =
        step_states[k + 1];

    for (auto& [s_cur, step_state_cur] : step_state_k) {
      StepState step_state_cur_filtered;

      for (const StepGroup& g_next : completed.GetGroups(s_cur)) {
        StopId s_next = g_next.destination_stop;
        if (s_next == boundary.start || s_next == boundary.end) {
          continue;
        }

        auto step_state_next_it = step_state_kplus1.find(s_next);
        if (step_state_next_it == step_state_kplus1.end()) {
          continue;
        }
        const StepState& step_state_next = step_state_next_it->second;

        for (const TimeSinceServiceStart& t_cur :
             step_state_cur.arrival_times) {
          // TODO: Deal with flex steps.
          std::span<const AdjacencyListStep> group_steps =
              completed.GetSteps(g_next);
          size_t t_next_i = FindDepartureAtOrAfter(completed, g_next, t_cur);
          if (t_next_i >= group_steps.size()) {
            continue;
          }
          TimeSinceServiceStart t_next = group_steps[t_next_i].destination_time;
          auto it = std::find_if(
              step_state_next.arrival_times.begin(),
              step_state_next.arrival_times.end(),
              [&](const TimeSinceServiceStart& t) { return t == t_next; }
          );
          if (it != step_state_next.arrival_times.end()) {
            step_state_cur_filtered.arrival_times.push_back(t_cur);
          }
        }
      }

      step_state_cur_filtered.SortAndDedupe();
      step_state_cur = std::move(step_state_cur_filtered);
    }
  }

  // for (int k = 0; k < step_states.size(); ++k) {
  //   const std::unordered_map<StopId, StepState>& step_state_k =
  //   step_states[k]; int max_smear = 0; TimeSinceServiceStart
  //   min_t{std::numeric_limits<int>::max()}, max_t{0}; for (const auto& [s,
  //   step_state] : step_state_k) {
  //     if (step_state.arrival_times.size() == 0) {
  //       continue;
  //     }
  //     int smear = step_state.arrival_times.back().seconds -
  //                 step_state.arrival_times[0].seconds;
  //     max_smear = std::max(max_smear, smear);
  //     min_t = std::min(min_t, step_state.arrival_times[0]);
  //     max_t = std::max(max_t, step_state.arrival_times.back());
  //   }
  //   std::cout << "k = " << k << "\n"
  //             << "  [" << min_t << ", " << max_t << "] (difference "
  //             << TimeSinceServiceStart{max_t.seconds - min_t.seconds} <<
  //             ")\n"
  //             << "  max smear " << TimeSinceServiceStart{max_smear} << "\n";
  // }

  return step_states;
}

std::optional<TspTourResult> DoTSP(
    const StepsAdjacencyList& completed,
    const RequiredStops& required,
    const ProblemBoundary& boundary,
    const std::vector<std::unordered_map<StopId, StepState>>& step_states,
    int ub_rel
) {
  // Convention:
  // StepPartitionId::NONE is the partition for START and END.
  // StepPartitionId{k} is the partition for where you arrive after the k-th
  // step:
  // - StepPartitionId{0} is the first partition you get to after START.
  // - StepPartitionId{required.size() - 3} is the last partition after stepping
  // to END.
  std::vector<TarelEdge> edges;

  // END->START edge.
  TarelState start_state{boundary.start, StepPartitionId::NONE};
  TarelState end_state{boundary.end, StepPartitionId::NONE};
  edges.push_back({
      .origin = end_state,
      .destination = start_state,
      .weight = 0,
  });

  // Entry (START->*) edges.
  for (const auto& [s, step_state] : step_states[0]) {
    edges.push_back({
        .origin = start_state,
        .destination = TarelState{s, StepPartitionId{0}},
        .weight = 0,
    });
  }

  // Exit (*->END) edges.
  for (const auto& [s, step_state] : step_states.back()) {
    edges.push_back({
        .origin =
            TarelState{
                s, StepPartitionId{static_cast<int>(required.size()) - 3}
            },
        .destination = end_state,
        .weight = 0,
    });
  }

  // Inter-stop edges.
  for (int k = 1; k <= required.size() - 3; ++k) {
    const std::unordered_map<StopId, StepState>& step_state_kminus1 =
        step_states[k - 1];

    for (const auto& [s_cur, step_state_cur] : step_state_kminus1) {
      for (const StepGroup& g_next : completed.GetGroups(s_cur)) {
        StopId s_next = g_next.destination_stop;
        if (s_next == boundary.start || s_next == boundary.end) {
          continue;
        }

        int best_dur = std::numeric_limits<int>::max();
        for (const TimeSinceServiceStart& t_cur :
             step_state_cur.arrival_times) {
          // TODO: Deal with flex steps.
          std::span<const AdjacencyListStep> group_steps =
              completed.GetSteps(g_next);
          size_t t_next_i = FindDepartureAtOrAfter(completed, g_next, t_cur);
          if (t_next_i >= group_steps.size()) {
            continue;
          }
          TimeSinceServiceStart t_next = group_steps[t_next_i].destination_time;
          int dur = t_next.seconds - t_cur.seconds;
          best_dur = std::min(best_dur, dur);
        }

        if (best_dur < std::numeric_limits<int>::max()) {
          edges.push_back({
              .origin = TarelState{s_cur, StepPartitionId{k - 1}},
              .destination = TarelState{s_next, StepPartitionId{k}},
              .weight = best_dur,
          });
        }
      }
    }
  }

  // SOLVE!!!
  TarelStateRemapResult remap = RemapTarelStates(edges, required);
  TspGraphData graph = MakeTspGraphEdges(remap.edges, boundary);

  // Check that at least one representative from each group of required stops
  // appears in `graph`.
  //
  // This is necessary for correctness because the above construction can omit
  // stops from `graph`, and if it does, then the TSP on `graph` will give a
  // solution that does not reach all the stops. (Specifically, stops that don't
  // appear as both origins and destinations are omitted).
  std::unordered_set<StopId> representatives_in_graph;
  for (const TarelState& tarel_state : graph.state_by_id) {
    representatives_in_graph.insert(required.Representative(tarel_state.stop));
  }
  for (StopId rep : required.GroupRepresentatives()) {
    if (!representatives_in_graph.contains(rep)) {
      return std::nullopt;
    }
  }

  std::optional<TspTourResult> result = SolveTspAndExtractTour(
      remap.edges,
      graph,
      boundary,
      ub_rel == -1 ? std::nullopt : std::optional(ub_rel),
      nullptr,
      nullptr
  );
  if (!result.has_value()) {
    return std::nullopt;
  }

  // Map `result` states back to original states.
  for (TarelEdge& edge : result->tour_edges) {
    edge.origin = remap.mapped_to_original.at(edge.origin);
    edge.destination = remap.mapped_to_original.at(edge.destination);
  }

  return result;
}

int main(int argc, char* argv[]) {
  CLI::App app{"Track count tool"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  std::string gtfs_stop_id_str;
  app.add_option("gtfs_stop_id", gtfs_stop_id_str, "GTFS stop ID");

  std::string time_str;
  app.add_option("time", time_str, "Departure time in hh:mm:ss format");

  std::string lb_str;
  app.add_option(
      "--lb",
      lb_str,
      "Lower bound in hh:mm:ss format (relative to departure time)"
  );

  std::string ub_str;
  app.add_option(
      "--ub",
      ub_str,
      "Upper bound in hh:mm:ss format (relative to departure time)"
  );

  CLI11_PARSE(app, argc, argv);

  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << "\n";
    return 1;
  }

  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState problem = j.get<ProblemState>();
  in.close();

  // Reduce it to just the group representatives (eliminates non-required stops
  // and multi-stop groups which the later codes do not support yet).
  //
  // TODO: Make later codes support this stuff.
  auto fooized = ReduceToMinimalSystemPaths(
      problem.minimal, problem.required.GroupRepresentatives()
  );
  problem.minimal = MakeAdjacencyList(fooized.AllMergedSteps());

  if (gtfs_stop_id_str.empty()) {
    ListStops(problem);
    return 0;
  }

  StopId stop = FindStopByGtfsId(problem, gtfs_stop_id_str);

  int lb_rel = -1;
  if (!lb_str.empty()) {
    lb_rel = TimeSinceServiceStart::Parse(lb_str).seconds;
  }

  int ub_rel = -1;
  if (!ub_str.empty()) {
    ub_rel = TimeSinceServiceStart::Parse(ub_str).seconds;
  }

  // Collect departure times to iterate over.
  std::vector<TimeSinceServiceStart> departure_times;
  if (time_str.empty()) {
    // Gather all distinct departure times from this stop.
    std::set<TimeSinceServiceStart> times_set;
    for (const auto& group : problem.minimal.GetGroups(stop)) {
      for (const auto& step : problem.minimal.GetSteps(group)) {
        times_set.insert(step.origin_time);
      }
    }
    departure_times.assign(times_set.begin(), times_set.end());
  } else {
    departure_times.push_back(TimeSinceServiceStart::Parse(time_str));
  }

  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());

  for (const TimeSinceServiceStart& t0 : departure_times) {
    std::cout << t0.ToString() << ": ";
    std::vector<std::unordered_map<StopId, StepState>> step_states =
        ComputeStepStates(
            completed,
            problem.required,
            problem.boundary,
            stop,
            t0,
            lb_rel,
            ub_rel
        );
    std::optional<TspTourResult> result = DoTSP(
        completed, problem.required, problem.boundary, step_states, ub_rel
    );
    if (result.has_value()) {
      std::cout << TimeSinceServiceStart{result->optimal_value} << "\n";

      TimeSinceServiceStart t_relaxed = t0;
      TimeSinceServiceStart t_actual = t0;
      std::cout << "  " << problem.StopName(result->tour_edges[0].origin.stop)
                << " @ " << t_relaxed << " / " << t_actual << "\n";
      for (int i = 1; i + 1 < result->tour_edges.size(); ++i) {
        const TarelEdge& edge = result->tour_edges[i];
        t_relaxed.seconds += edge.weight;

        bool found = false;
        for (const StepGroup& g : completed.GetGroups(edge.origin.stop)) {
          if (g.destination_stop != edge.destination.stop) {
            continue;
          }
          std::span<const AdjacencyListStep> group_steps =
              completed.GetSteps(g);
          size_t t_next_i = FindDepartureAtOrAfter(completed, g, t_actual);
          assert(t_next_i < group_steps.size());
          t_actual = group_steps[t_next_i].destination_time;
          found = true;
          break;
        }
        assert(found);

        std::cout << "  " << problem.StopName(edge.destination.stop) << " @ "
                  << t_relaxed << " / " << t_actual << "\n";
      }

      int ub_actual = t_actual.seconds - t0.seconds;
      std::cout << "  ub actual: " << TimeSinceServiceStart{ub_actual} << "\n";
      if (ub_rel == -1) {
        ub_rel = ub_actual;
      }
      ub_rel = std::min(ub_rel, ub_actual);

      // TODO: Because of periodicity, paths departing at other times that
      // follow the same sequence might also be pretty good or even better, so
      // we should consider them!!

    } else {
      std::cout << "No result\n";
    }
  }

  return 0;
}
