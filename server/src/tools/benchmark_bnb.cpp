#include <CLI/CLI.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "solver/branch_and_bound.h"
#include "solver/search_event.h"
#include "solver/tarel_graph.h"

using namespace vats5;

std::string FormatDuration(int ms) {
  if (ms < 1000) {
    return std::to_string(ms) + " ms";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1) << (ms / 1000.0) << " s";
  return ss.str();
}

namespace {

// The duration, from arriving at the origin of a tarel edge to arriving at its
// destination, that `step` achieves. This is the best `step` can do: it assumes
// the latest arrival that still catches it.
//
// The min of this over all the steps of an edge is the edge's weight, so a step
// achieving the min is what makes the weight what it is.
//
// Returns nullopt if no arrival catches `step`.
std::optional<int> ArrivalToArrivalDuration(
    const ArrivalTimes& arrivals, const Step& step
) {
  if (arrivals.has_flex) {
    // We can arrive any time, so we can arrive just in time for the step.
    return step.DurationSeconds();
  }
  if (step.is_flex) {
    // The step can be taken any time, so it can be taken on arrival.
    return step.FlexDurationSeconds();
  }
  auto after_step = std::ranges::upper_bound(arrivals.times, step.origin.time);
  if (after_step == arrivals.times.begin()) {
    // Every arrival is after the step departs.
    return std::nullopt;
  }
  return step.destination.time.seconds - std::prev(after_step)->seconds;
}

// How the steps of one tarel edge stack up against its weight.
//
// NOTE: assumes unforwarded weights (weight == min arrive-to-arrive duration
// over the steps); run with VATS5_TAREL_FWD_PASSES=0, or this throws on edges
// that slack forwarding strengthened.
struct TarelEdgeStepStats {
  TarelEdge edge;

  // Steps from the edge's origin stop that arrive at its destination state.
  int num_steps = 0;

  // Of those, ones that no arrival at the edge's origin state catches.
  int num_unusable_steps = 0;

  // Of the rest, ones whose duration is the edge's weight, i.e. ones that would
  // have to get slower before the weight did.
  int num_binding_steps = 0;

  // How much slower than the weight the fastest non-binding step is, or nullopt
  // if every usable step is binding.
  std::optional<int> next_smallest_gap;
};

TarelEdgeStepStats ComputeTarelEdgeStepStats(
    const ProblemState& state,
    const TarelEdgeIntermediateData& data,
    const TarelEdge& edge
) {
  const ArrivalTimes& arrivals = data.arrival_times_to.at(edge.origin);
  const std::vector<Step>& steps =
      data.steps_from.at(edge.origin.stop).at(edge.destination);

  TarelEdgeStepStats stats{.edge = edge};
  stats.num_steps = static_cast<int>(steps.size());
  for (const Step& step : steps) {
    std::optional<int> duration = ArrivalToArrivalDuration(arrivals, step);
    if (!duration.has_value()) {
      stats.num_unusable_steps += 1;
      continue;
    }
    if (*duration < edge.weight) {
      throw std::runtime_error(
          "Step of " + edge.Debug(state) + " takes " +
          TimeSinceServiceStart{*duration}.ToString() +
          ", which beats the edge weight"
      );
    }
    if (*duration == edge.weight) {
      stats.num_binding_steps += 1;
    } else if (!stats.next_smallest_gap.has_value() ||
               *duration - edge.weight < *stats.next_smallest_gap) {
      stats.next_smallest_gap = *duration - edge.weight;
    }
  }
  if (stats.num_binding_steps == 0) {
    throw std::runtime_error(
        "No step of " + edge.Debug(state) + " achieves the edge weight"
    );
  }
  return stats;
}

// For every tarel edge, print how many steps it is built from and how many of
// them are binding on its weight. An edge with few binding steps and a big gap
// to the next-smallest step is one whose weight rests on a single lucky
// connection.
void PrintTarelEdgeStepStats(const ProblemState& state) {
  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();
  TarelEdgeIntermediateData data =
      ComputeTarelIntermediateData(completed.AllMergedSteps());
  std::vector<TarelEdge> edges = BuildTarelEdgesFromIntermediateData(data);

  std::vector<TarelEdgeStepStats> stats;
  stats.reserve(edges.size());
  for (const TarelEdge& edge : edges) {
    stats.push_back(ComputeTarelEdgeStepStats(state, data, edge));
  }

  std::ranges::sort(stats, [](const auto& a, const auto& b) {
    if (a.num_binding_steps != b.num_binding_steps) {
      return a.num_binding_steps < b.num_binding_steps;
    }
    // Tie-break on everything else, for a deterministic printout.
    if (a.num_steps != b.num_steps) {
      return a.num_steps < b.num_steps;
    }
    return std::tie(
               a.edge.origin.stop.v,
               a.edge.origin.partition.v,
               a.edge.destination.stop.v,
               a.edge.destination.partition.v
           ) < std::tie(
                   b.edge.origin.stop.v,
                   b.edge.origin.partition.v,
                   b.edge.destination.stop.v,
                   b.edge.destination.partition.v
               );
  });

  // An edge whose every usable step is binding has no next-smallest step to
  // compare its weight against, so there is nothing to say about it here.
  int num_all_binding =
      std::ranges::count_if(stats, [](const TarelEdgeStepStats& s) {
        return !s.next_smallest_gap.has_value();
      });

  std::cout << (stats.size() - num_all_binding) << " of " << stats.size()
            << " tarel edges have a non-binding step, by number of binding "
               "steps ascending:\n";
  for (const TarelEdgeStepStats& s : stats) {
    if (!s.next_smallest_gap.has_value()) {
      continue;
    }
    std::cout << "  " << s.edge.Debug(state) << ": " << s.num_steps
              << " steps, " << s.num_binding_steps << " binding";
    if (s.num_unusable_steps > 0) {
      std::cout << ", " << s.num_unusable_steps << " unusable";
    }
    std::cout << ", next smallest +" << *s.next_smallest_gap << " s\n";
  }
}

// How much predecessor conditioning could strengthen each tarel edge.
//
// A 2-hop (predecessor-conditioned) scheme may charge a tour arriving at the
// edge's origin from stop P the weight computed over only the arrivals that
// steps from P produce. The spread of that conditioned weight over P, versus
// the unconditioned weight (their min), measures the headroom such a scheme
// has over per-edge weights: it is what the fantasy arrivals from other
// predecessors currently give away.
void PrintPredecessorStats(const ProblemState& state) {
  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();
  std::vector<Step> steps = completed.AllMergedSteps();
  TarelEdgeIntermediateData data = ComputeTarelIntermediateData(steps);

  // Arrival lists conditioned on the producing step's origin stop.
  struct CondArrivals {
    std::vector<TimeSinceServiceStart> times;
    bool has_flex = false;
  };
  std::unordered_map<TarelState, std::unordered_map<StopId, CondArrivals>>
      arrivals_by_pred;
  for (const Step& step : steps) {
    TarelState d{step.destination.stop, step.destination.partition};
    CondArrivals& ca = arrivals_by_pred[d][step.origin.stop];
    if (step.is_flex) {
      ca.has_flex = true;
    } else {
      ca.times.push_back(step.destination.time);
    }
  }
  for (auto& [_, preds] : arrivals_by_pred) {
    for (auto& [_2, ca] : preds) {
      std::ranges::sort(ca.times);
      auto [first, last] = std::ranges::unique(ca.times);
      ca.times.erase(first, last);
    }
  }

  // Unforwarded weight of one arrival list against one step group.
  auto weight_for = [](const CondArrivals& ca,
                       const std::vector<Step>& group,
                       int min_duration) -> int {
    int weight = std::numeric_limits<int>::max();
    if (ca.has_flex) {
      return min_duration;
    }
    size_t step_idx = 0;
    if (group.size() > 0 && group[0].is_flex) {
      weight = group[0].FlexDurationSeconds();
      step_idx = 1;
    }
    for (const TimeSinceServiceStart t : ca.times) {
      while (step_idx < group.size() && group[step_idx].origin.time < t) {
        step_idx += 1;
      }
      if (step_idx >= group.size()) {
        break;
      }
      weight = std::min(
          weight, group[step_idx].destination.time.seconds - t.seconds
      );
    }
    return weight;
  };

  struct EdgeRow {
    TarelEdge edge;
    int num_preds = 0;
    int num_infeasible_preds = 0;
    int num_achieving_preds = 0;
    int median_gain = 0;
    int max_gain = 0;
  };
  std::vector<EdgeRow> rows;
  long total_pred_edges = 0;
  long total_infeasible = 0;

  for (const auto& [origin, preds] : arrivals_by_pred) {
    auto it = data.steps_from.find(origin.stop);
    if (it == data.steps_from.end()) {
      continue;
    }
    for (const auto& [dest, group] : it->second) {
      int min_duration = data.min_duration_from_to.at({origin.stop, dest});
      std::vector<int> per_pred;
      int infeasible = 0;
      for (const auto& [_, ca] : preds) {
        int w = weight_for(ca, group, min_duration);
        if (w == std::numeric_limits<int>::max()) {
          infeasible += 1;
        } else {
          per_pred.push_back(w);
        }
      }
      if (per_pred.empty()) {
        continue;
      }
      std::ranges::sort(per_pred);
      int w = per_pred[0];
      EdgeRow row{
          .edge = TarelEdge{origin, dest, w},
          .num_preds = static_cast<int>(per_pred.size()) + infeasible,
          .num_infeasible_preds = infeasible,
          .num_achieving_preds = static_cast<int>(std::ranges::count(per_pred, w)),
          .median_gain = per_pred[per_pred.size() / 2] - w,
          .max_gain = per_pred.back() - w,
      };
      total_pred_edges += row.num_preds;
      total_infeasible += infeasible;
      rows.push_back(row);
    }
  }

  auto bucket = [](int gain) -> const char* {
    if (gain == 0) return "0";
    if (gain <= 60) return "1-60s";
    if (gain <= 300) return "1-5min";
    return ">5min";
  };
  std::map<std::string, int> median_buckets, max_buckets;
  int single_achieving = 0;
  for (const EdgeRow& r : rows) {
    median_buckets[bucket(r.median_gain)] += 1;
    max_buckets[bucket(r.max_gain)] += 1;
    if (r.num_achieving_preds == 1 && r.num_preds > 1) {
      single_achieving += 1;
    }
  }

  std::cout << rows.size() << " tarel edges; " << total_pred_edges
            << " (edge, predecessor) pairs, of which " << total_infeasible
            << " are infeasible (pair can never be traversed)\n";
  std::cout << "Edges whose weight only one of several predecessors achieves: "
            << single_achieving << "\n";
  std::cout << "Median conditioned gain per edge:";
  for (const auto& [b, n] : median_buckets) std::cout << "  " << b << ": " << n;
  std::cout << "\nMax conditioned gain per edge:   ";
  for (const auto& [b, n] : max_buckets) std::cout << "  " << b << ": " << n;
  std::cout << "\n\nTop edges by median conditioned gain:\n";
  std::ranges::sort(rows, [](const EdgeRow& a, const EdgeRow& b) {
    return a.median_gain > b.median_gain;
  });
  for (size_t i = 0; i < rows.size() && i < 15; ++i) {
    const EdgeRow& r = rows[i];
    std::cout << "  " << r.edge.Debug(state) << " w=" << r.edge.weight
              << " preds=" << r.num_preds << " (infeas "
              << r.num_infeasible_preds << ", achieving "
              << r.num_achieving_preds << ") median +" << r.median_gain
              << "s max +" << r.max_gain << "s\n";
  }
}

// Root lower bound of the predecessor-conditioned ("2-hop") tarel TSP.
//
// States become (stop, (partition, predecessor stop)), with the pair interned
// into the partition id. Edges into (Y, (p, P)) exist only from states at stop
// P, so a tour's predecessor choices are consistent by construction, and each
// edge's weight uses only the arrivals its predecessor produces. This bounds
// what a 2-hop scheme could gain at the root, next to the unconditioned bound
// from the same unforwarded weights.
void PrintPredecessorConditionedLb(const ProblemState& state) {
  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();
  std::vector<Step> steps = completed.AllMergedSteps();
  TarelEdgeIntermediateData data = ComputeTarelIntermediateData(steps);

  struct CondArrivals {
    std::vector<TimeSinceServiceStart> times;
    bool has_flex = false;
  };
  std::unordered_map<TarelState, std::unordered_map<StopId, CondArrivals>>
      arrivals_by_pred;
  for (const Step& step : steps) {
    TarelState d{step.destination.stop, step.destination.partition};
    CondArrivals& ca = arrivals_by_pred[d][step.origin.stop];
    if (step.is_flex) {
      ca.has_flex = true;
    } else {
      ca.times.push_back(step.destination.time);
    }
  }
  for (auto& [_, preds] : arrivals_by_pred) {
    for (auto& [_2, ca] : preds) {
      std::ranges::sort(ca.times);
      auto [first, last] = std::ranges::unique(ca.times);
      ca.times.erase(first, last);
    }
  }

  auto weight_for = [](const CondArrivals& ca,
                       const std::vector<Step>& group,
                       int min_duration) -> int {
    int weight = std::numeric_limits<int>::max();
    if (ca.has_flex) {
      return min_duration;
    }
    size_t step_idx = 0;
    if (group.size() > 0 && group[0].is_flex) {
      weight = group[0].FlexDurationSeconds();
      step_idx = 1;
    }
    for (const TimeSinceServiceStart t : ca.times) {
      while (step_idx < group.size() && group[step_idx].origin.time < t) {
        step_idx += 1;
      }
      if (step_idx >= group.size()) {
        break;
      }
      weight = std::min(
          weight, group[step_idx].destination.time.seconds - t.seconds
      );
    }
    return weight;
  };

  // Intern (partition, predecessor stop) pairs into synthetic partition ids.
  std::map<std::pair<int, int>, int> interned;
  auto intern = [&interned](StepPartitionId partition, StopId pred) {
    auto [it, _] = interned.try_emplace(
        {partition.v, pred.v}, static_cast<int>(interned.size())
    );
    return StepPartitionId{it->second};
  };

  std::vector<TarelEdge> cond_edges;
  for (const auto& [origin, preds] : arrivals_by_pred) {
    auto it = data.steps_from.find(origin.stop);
    if (it == data.steps_from.end()) {
      continue;
    }
    for (const auto& [pred, ca] : preds) {
      TarelState cond_origin{origin.stop, intern(origin.partition, pred)};
      for (const auto& [dest, group] : it->second) {
        int min_duration = data.min_duration_from_to.at({origin.stop, dest});
        int w = weight_for(ca, group, min_duration);
        if (w == std::numeric_limits<int>::max()) {
          continue;
        }
        cond_edges.push_back(
            TarelEdge{
                cond_origin,
                TarelState{dest.stop, intern(dest.partition, origin.stop)},
                w,
            }
        );
      }
    }
  }

  auto solve = [&state](const std::vector<TarelEdge>& edges)
      -> std::optional<int> {
    TarelStateRemapResult remap = RemapTarelStates(edges, state.required);
    TspGraphData graph = MakeTspGraphEdges(remap.edges, state.boundary);
    std::unordered_set<StopId> representatives_in_graph;
    for (const TarelState& tarel_state : graph.state_by_id) {
      representatives_in_graph.insert(
          state.required.Representative(tarel_state.stop)
      );
    }
    for (StopId rep : state.required.GroupRepresentatives()) {
      if (!representatives_in_graph.contains(rep)) {
        return std::nullopt;
      }
    }
    std::optional<TspTourResult> result = SolveTspAndExtractTour(
        remap.edges, graph, state.boundary, std::nullopt, nullptr, nullptr
    );
    if (!result.has_value()) {
      return std::nullopt;
    }
    return result->optimal_value;
  };

  // Unconditioned, unforwarded edges via the same machinery: aggregate the
  // conditioned arrival lists over predecessors.
  std::vector<TarelEdge> unforwarded_edges;
  for (const auto& [origin, preds] : arrivals_by_pred) {
    auto it = data.steps_from.find(origin.stop);
    if (it == data.steps_from.end()) {
      continue;
    }
    CondArrivals merged;
    for (const auto& [_, ca] : preds) {
      merged.has_flex = merged.has_flex || ca.has_flex;
      merged.times.insert(merged.times.end(), ca.times.begin(), ca.times.end());
    }
    std::ranges::sort(merged.times);
    auto [first, last] = std::ranges::unique(merged.times);
    merged.times.erase(first, last);
    for (const auto& [dest, group] : it->second) {
      int min_duration = data.min_duration_from_to.at({origin.stop, dest});
      int w = weight_for(merged, group, min_duration);
      if (w != std::numeric_limits<int>::max()) {
        unforwarded_edges.push_back(TarelEdge{origin, dest, w});
      }
    }
  }

  std::vector<TarelEdge> forwarded_edges =
      BuildTarelEdgesFromIntermediateData(data);
  std::cout << "  conditioned states: " << interned.size()
            << ", conditioned edges: " << cond_edges.size()
            << " (plain edges: " << forwarded_edges.size() << ")\n";

  auto ms = [](auto a, auto b) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
  };
  auto report = [&](const char* name, const std::vector<TarelEdge>& edges) {
    auto t0 = std::chrono::steady_clock::now();
    std::optional<int> v = solve(edges);
    auto t1 = std::chrono::steady_clock::now();
    std::cout << "  " << name << ": ";
    if (v.has_value()) {
      std::cout << TimeSinceServiceStart{*v} << " (" << *v << "s)";
    } else {
      std::cout << "none";
    }
    std::cout << " in " << ms(t0, t1) << " ms\n";
  };
  report("LB unforwarded            ", unforwarded_edges);
  report("LB forwarded (defaults)   ", forwarded_edges);
  report("LB conditioned unforwarded", cond_edges);
}

// Applies require-edge constraints along `sequence` (stop names, starting from
// START) and prints the LB comparison at each depth, to measure whether
// predecessor conditioning helps in branched subproblems the way slack
// forwarding does.
void PrintPredecessorConditionedLbDeep(
    const ProblemState& root, const std::vector<std::string>& sequence
) {
  ProblemState cur = root;
  StopId prev = cur.boundary.start;
  std::cout << "depth 0 (root):\n";
  PrintPredecessorConditionedLb(cur);
  int depth = 0;
  for (const std::string& name : sequence) {
    std::optional<StopId> target;
    for (const auto& [stop, _] : cur.stop_infos) {
      if (cur.StopName(stop) == name) {
        target = stop;
        break;
      }
    }
    if (!target.has_value()) {
      std::cout << "stop not found: " << name << "\n";
      return;
    }
    ProblemState next =
        ApplyConstraints(cur, {ConstraintRequireEdge{prev, *target}});
    std::optional<StopId> combined;
    for (const auto& [stop, edge] : next.original_edges) {
      if (edge == PlainEdge{prev, *target}) {
        combined = stop;
        break;
      }
    }
    if (!combined.has_value()) {
      std::cout << "combined stop not found after requiring " << name << "\n";
      return;
    }
    prev = *combined;
    cur = std::move(next);
    depth += 1;
    std::cout << "depth " << depth << " (required ... -> " << name << "):\n";
    PrintPredecessorConditionedLb(cur);
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  CLI::App app{"Benchmark Branch and Bound solver"};

  bool pred_stats = false;
  app.add_flag(
      "--pred-stats", pred_stats,
      "Print predecessor-conditioning headroom stats and exit"
  );

  bool pred_lb = false;
  app.add_flag(
      "--pred-lb", pred_lb,
      "Solve the predecessor-conditioned root LB and exit"
  );

  std::string pred_lb_deep;
  app.add_option(
      "--pred-lb-deep", pred_lb_deep,
      "Comma-separated stop names to require in sequence from START, printing "
      "the LB comparison at each depth"
  );

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  int max_iter = -1;
  app.add_option(
         "--max-iter", max_iter, "Maximum iterations (-1 for unlimited)"
  )
      ->default_val(-1);

  CLI11_PARSE(app, argc, argv);

  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << "\n";
    return 1;
  }

  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState state = j.get<ProblemState>();
  in.close();

  // PrintTarelEdgeStepStats(state);
  // return 0;

  if (pred_stats) {
    PrintPredecessorStats(state);
    return 0;
  }

  if (pred_lb) {
    PrintPredecessorConditionedLb(state);
    return 0;
  }

  if (!pred_lb_deep.empty()) {
    std::vector<std::string> sequence;
    std::stringstream ss(pred_lb_deep);
    std::string name;
    while (std::getline(ss, name, ',')) {
      sequence.push_back(name);
    }
    PrintPredecessorConditionedLbDeep(state, sequence);
    return 0;
  }

  std::cout << "Loaded problem state from: " << input_path << "\n";
  std::cout << "Stops: " << state.minimal.NumStops() << "\n";
  std::cout << "Required stops: " << state.required.size() << "\n";
  std::cout << "\n";

  std::vector<TarelSolve> solves;
  auto on_event = [&](const SearchEvent& event) {
    std::visit([&](const TarelSolve& e) { solves.push_back(e); }, event);
  };

  auto start = std::chrono::steady_clock::now();
  auto result =
      BranchAndBoundSolve(state, &std::cerr, std::nullopt, max_iter, on_event);
  auto end = std::chrono::steady_clock::now();

  int total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  int total_concorde_ms = 0;
  for (const auto& s : solves) {
    total_concorde_ms += s.concorde_ms;
  }
  int non_concorde_ms = total_ms - total_concorde_ms;

  std::cout << "\nBest duration: " << TimeSinceServiceStart{result.best_ub}
            << "\n";
  if (!result.best_paths.empty()) {
    const auto& path = result.best_paths[0];
    std::vector<StopId> tour;
    path.VisitAllStops([&](StopId stop) {
      ExpandStop(stop, result.original_edges, tour);
    });
    std::cout << "Tour (" << tour.size() << " stops):\n";
    for (StopId stop : tour) {
      std::cout << "  " << state.StopName(stop) << "\n";
    }
  }

  std::cout << "\nConcorde solves (" << solves.size() << "):\n";
  for (int i = 0; i < static_cast<int>(solves.size()); ++i) {
    const auto& s = solves[i];
    std::cout << "  #" << (i + 1) << ": " << s.vertex_count << " vertices, "
              << s.edge_count << " edges, " << FormatDuration(s.concorde_ms)
              << (s.feasible ? "" : " (infeasible)") << "\n";
  }

  std::cout << "\nTotal time:       " << FormatDuration(total_ms) << "\n";
  std::cout << "Concorde time:    " << FormatDuration(total_concorde_ms)
            << "\n";
  std::cout << "Non-concorde time: " << FormatDuration(non_concorde_ms) << "\n";

  return 0;
}
