#include <CLI/CLI.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"

using namespace vats5;

std::vector<int> MinCompletionTimePerStep(const ProblemState& problem) {
  // Oh no this needs to be updated to account for non-required stops and for
  // required stop _groups_.

  auto graph =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());

  std::vector<int> min_out_durs;
  for (StopId s{0}; s.v < graph.NumStops(); ++s.v) {
    if (s == problem.boundary.start || s == problem.boundary.end) {
      continue;
    }

    if (graph.GetGroups(s).size() == 0) {
      continue;
    }

    int min_out_dur = std::numeric_limits<int>::max();
    for (const StepGroup& g : graph.GetGroups(s)) {
      if (g.destination_stop == problem.boundary.start ||
          g.destination_stop == problem.boundary.end) {
        continue;
      }
      if (g.flex_step.has_value()) {
        min_out_dur = std::min(min_out_dur, g.flex_step->FlexDurationSeconds());
      }
      for (const AdjacencyListStep& step : graph.GetSteps(g)) {
        min_out_dur = std::min(
            min_out_dur,
            step.destination_time.seconds - step.origin_time.seconds
        );
      }
    }
    min_out_durs.push_back(min_out_dur);
  }

  std::ranges::sort(min_out_durs);
  int cumsum = 0;
  std::vector<int> lb_at_step(min_out_durs.size());
  for (int i = 0; i < min_out_durs.size(); ++i) {
    int lb_at_step_i = static_cast<int>(min_out_durs.size()) - 1 - i;
    std::cout << "step " << lb_at_step_i << " : "
              << TimeSinceServiceStart{cumsum} << " lb\n";
    lb_at_step[lb_at_step_i] = cumsum;
    cumsum += min_out_durs[i];
  }
  return lb_at_step;
}

struct DPArrivalState {
  TimeSinceServiceStart time;
  std::set<std::vector<StopId>> paths;
};

struct DPArrivalData {
  std::vector<DPArrivalState> states;

  void SortAndDeduplicate() {
    std::sort(
        states.begin(),
        states.end(),
        [](const DPArrivalState& a, const DPArrivalState& b) {
          return a.time < b.time;
        }
    );
    // Merge states with the same time by unioning their paths.
    size_t write = 0;
    for (size_t read = 0; read < states.size();) {
      size_t end = read + 1;
      while (end < states.size() && states[end].time == states[read].time) {
        ++end;
      }
      DPArrivalState merged{
          .time = states[read].time, .paths = std::move(states[read].paths)
      };
      for (size_t k = read + 1; k < end; ++k) {
        merged.paths.merge(std::move(states[k].paths));
      }
      states[write++] = std::move(merged);
      read = end;
    }
    states.resize(write);
  }
};

int CountTimes(const std::vector<DPArrivalData>& state) {
  int count = 0;
  for (const DPArrivalData& arrival : state) {
    count += arrival.states.size();
  }
  return count;
}

void DPTimes(
    const ProblemState& problem, StopId s0, TimeSinceServiceStart t0, int ub_rel
) {
  constexpr int kMaxPathLen = 1;

  auto graph =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());

  TimeSinceServiceStart t_ub;
  if (ub_rel < 0) {
    for (const Step& step : graph.AllSteps()) {
      t_ub = std::max(t_ub, step.destination.time);
    }
  } else {
    t_ub.seconds = t0.seconds + ub_rel;
  }

  std::vector<int> lb_at_iter = MinCompletionTimePerStep(problem);

  std::vector<DPArrivalData> state(graph.NumStops());
  state[s0.v].states.push_back(
      {.time = t0, .paths = {{problem.boundary.start}}}
  );

  std::vector<DPArrivalData> acc_state = state;

  // TODO: Think about how many iters, including what to do about required "stop
  // groups".
  for (int iter = 0; iter < problem.required.size() - 3; ++iter) {
    std::cout << "Iter " << iter << ": " << CountTimes(state) << " cur, "
              << CountTimes(acc_state) << " acc\n";

    std::vector<DPArrivalData> next_state(graph.NumStops());
    for (StopId s_cur{0}; s_cur.v < graph.NumStops(); ++s_cur.v) {
      for (const DPArrivalState& arr_cur : state[s_cur.v].states) {
        TimeSinceServiceStart t_cur = arr_cur.time;
        for (const StepGroup& g_next : graph.GetGroups(s_cur)) {
          // TODO: Deal with flex steps.
          StopId s_next = g_next.destination_stop;

          std::set<std::vector<StopId>> next_paths;
          for (const std::vector<StopId>& arrival_path : arr_cur.paths) {
            bool in_arrival_path = false;
            for (StopId arrival_stop : arrival_path) {
              if (s_next == arrival_stop) {
                in_arrival_path = true;
                break;
              }
            }
            if (in_arrival_path) {
              continue;
            }

            int chop = arrival_path.size() == kMaxPathLen ? 1 : 0;
            std::vector<StopId> next_path(
                arrival_path.begin() + chop, arrival_path.end()
            );
            next_path.push_back(s_next);
            next_paths.insert(std::move(next_path));
          }
          if (next_paths.empty()) {
            continue;
          }

          std::span<const AdjacencyListStep> group_steps =
              graph.GetSteps(g_next);
          size_t t_next_i = FindDepartureAtOrAfter(graph, g_next, t_cur);
          if (t_next_i >= group_steps.size()) {
            continue;
          }
          TimeSinceServiceStart t_next = group_steps[t_next_i].destination_time;
          if (t_next.seconds + lb_at_iter[iter + 1] > t_ub.seconds) {
            continue;
          }
          next_state[s_next.v].states.push_back(
              {.time = t_next, .paths = next_paths}
          );
          acc_state[s_next.v].states.push_back(
              {.time = t_next, .paths = next_paths}
          );
        }
      }
    }
    for (StopId s{0}; s.v < graph.NumStops(); ++s.v) {
      next_state[s.v].SortAndDeduplicate();
      acc_state[s.v].SortAndDeduplicate();
    }
    state = std::move(next_state);
  }
}

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

int main(int argc, char* argv[]) {
  CLI::App app{"Precise times tool"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  std::string gtfs_stop_id_str;
  app.add_option("gtfs_stop_id", gtfs_stop_id_str, "GTFS stop ID");

  std::string time_str;
  app.add_option("time", time_str, "Departure time in hh:mm:ss format");

  std::string ub_str;
  app.add_option("--ub", ub_str, "Upper bound time in hh:mm:ss format");

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

  if (time_str.empty()) {
    PrintDepartures(problem, stop);
    return 0;
  }

  int ub_rel = -1;
  if (!ub_str.empty()) {
    ub_rel = TimeSinceServiceStart::Parse(ub_str).seconds;
  }

  TimeSinceServiceStart t0 = TimeSinceServiceStart::Parse(time_str);
  std::cout << "Stops: " << problem.minimal.NumStops() << "\n";
  std::cout << "Required stop groups: " << problem.required.size() << "\n";
  DPTimes(problem, stop, t0, ub_rel);
  return 0;
}
