#include <CLI/CLI.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <unordered_map>

#include "solver/gtsp.h"
#include "solver/gtsp_concorde.h"
#include "solver/held_karp_dp.h"
#include "solver/tarel_graph.h"
#include "solver/two_opt.h"

using namespace vats5;

namespace {

std::string FormatDuration(int ms) {
  if (ms < 1000) {
    return std::to_string(ms) + " ms";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1) << (ms / 1000.0) << " s";
  return ss.str();
}

int ElapsedMs(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - start
  )
      .count();
}

}  // namespace

int main(int argc, char* argv[]) {
  CLI::App app{
      "Solve the scheduled-steps GTSP of a ProblemState with Concorde "
      "(Noon-Bean + doubling, sparse)"
  };

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();
  bool run_hk = false;
  app.add_flag(
      "--hk", run_hk, "Also run Held-Karp (which uses flex steps) to compare"
  );
  bool quiet = false;
  app.add_flag("--quiet", quiet, "Do not forward Concorde's log to stderr");
  int two_opt_restarts = 1000;
  app.add_option(
      "--two-opt-restarts",
      two_opt_restarts,
      "2-opt restarts used to find Concorde's starting tour (and U for "
      "--tarel-slack); 0 leaves the starting tour to Concorde's own heuristic"
  );
  std::string start_time_str;
  app.add_option(
      "--start-time",
      start_time_str,
      "Fix the tour's start time (HH:MM:SS); START then only leads to each "
      "stop's first departures at or after it. Default: the start of the "
      "2-opt tour. 'none' leaves the start time free"
  );
  bool tarel_slack = false;
  app.add_flag(
      "--tarel-slack",
      tarel_slack,
      "Instead of solving: with U from 2-opt and L from the tarel lower "
      "bound, count the GTSP edges whose step is slower than the fastest "
      "tarel leg by at least U - L (they cannot be in a tour better than U), "
      "then exit"
  );

  CLI11_PARSE(app, argc, argv);

  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << "\n";
    return 1;
  }
  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState state = j.get<ProblemState>();
  in.close();

  std::cout << "Loaded problem state from: " << input_path << "\n";
  std::cout << "Stops: " << state.minimal.NumStops() << "\n";
  std::cout << "Required stops: " << state.required.size() << "\n";

  auto t0 = std::chrono::steady_clock::now();
  StepPathsAdjacencyList completed_paths = state.ComputeCompletedGraph();
  StepsAdjacencyList completed =
      MakeAdjacencyList(completed_paths.AllMergedSteps());
  Gtsp gtsp = BuildGtsp(completed, state.boundary);
  std::vector<int> clusters = GtspClustersFromRequired(gtsp, state.required);
  std::cout << "Completion + GTSP build: " << FormatDuration(ElapsedMs(t0))
            << " (" << gtsp.vertices.size() << " vertices, "
            << gtsp.edges.size() << " edges)\n"
            << std::flush;
  std::optional<TimeSinceServiceStart> start_time;
  bool free_start_time = (start_time_str == "none");
  if (!start_time_str.empty() && !free_start_time) {
    start_time = TimeSinceServiceStart::Parse(start_time_str);
  }

  if (tarel_slack) {
    auto t_2opt = std::chrono::steady_clock::now();
    TwoOptOptions options;
    options.restarts = two_opt_restarts;
    TwoOptResult two_opt = TwoOptSolve(state, 0, options);
    if (two_opt.best_tour.empty()) {
      std::cout << "2-opt found no tour\n";
      return 1;
    }
    int ub = two_opt.best_val;
    std::cout << "U (2-opt, " << two_opt_restarts << " restarts): "
              << TimeSinceServiceStart{ub} << " ("
              << FormatDuration(ElapsedMs(t_2opt)) << ")\n"
              << std::flush;

    auto t_lb = std::chrono::steady_clock::now();
    std::optional<TspTourResult> tarel = ComputeTarelLowerBound(
        state, std::nullopt, quiet ? nullptr : &std::cerr
    );
    if (!tarel) {
      std::cout << "Tarel lower bound: no tour\n";
      return 1;
    }
    int lb = tarel->optimal_value;
    std::cout << "L (tarel): " << TimeSinceServiceStart{lb} << " ("
              << FormatDuration(ElapsedMs(t_lb)) << ")\n";
    int gap = ub - lb;
    std::cout << "U - L: " << gap << " s\n";

    // Fastest tarel leg into (b, q) from a, over all arrival partitions p at
    // a. A GTSP edge realized by a partition-q step a->b is such a leg, and
    // the real leg takes at least the step's duration.
    std::unordered_map<int64_t, int> min_tarel_weight;
    auto key = [](StopId a, StopId b, StepPartitionId q) {
      return (static_cast<int64_t>(a.v) << 40) |
             (static_cast<int64_t>(b.v) << 20) |
             static_cast<int64_t>(static_cast<uint32_t>(q.v) & 0xFFFFF);
    };
    for (const TarelEdge& e : MakeTarelEdges(completed_paths)) {
      auto [it, inserted] = min_tarel_weight.try_emplace(
          key(e.origin.stop, e.destination.stop, e.destination.partition),
          e.weight
      );
      if (!inserted && e.weight < it->second) {
        it->second = e.weight;
      }
    }

    long long step_edges = 0;
    long long no_tarel_edge = 0;
    long long eliminable_by_travel = 0;
    long long eliminable_by_weight = 0;
    for (const GtspEdge& e : gtsp.edges) {
      if (e.from == Gtsp::kStart || e.to == Gtsp::kEnd ||
          e.from == Gtsp::kEnd) {
        continue;
      }
      step_edges++;
      auto it = min_tarel_weight.find(key(
          gtsp.vertices[e.from].stop, gtsp.vertices[e.to].stop, e.partition
      ));
      if (it == min_tarel_weight.end()) {
        no_tarel_edge++;
        continue;
      }
      if (e.travel_seconds - it->second >= gap) {
        eliminable_by_travel++;
      }
      if (e.weight_seconds - it->second >= gap) {
        eliminable_by_weight++;
      }
    }
    std::cout << "GTSP step edges: " << step_edges << "\n";
    std::cout << "  without a tarel edge for (origin, destination, partition): "
              << no_tarel_edge << "\n";
    std::cout << "  step duration - fastest tarel leg >= U - L (sound): "
              << eliminable_by_travel << "\n";
    std::cout << "  edge weight (incl. wait) - fastest tarel leg >= U - L "
                 "(not sound, for comparison): "
              << eliminable_by_weight << "\n";
    return 0;
  }

  // A starting tour from 2-opt: its stop order, scheduled along the GTSP.
  // (2-opt itself may use flex steps, so the order can be infeasible or
  // cost more here.)
  std::optional<GtspSolution> initial_tour;
  if (two_opt_restarts > 0) {
    auto t_2opt = std::chrono::steady_clock::now();
    TwoOptOptions options;
    options.restarts = two_opt_restarts;
    TwoOptResult two_opt = TwoOptSolve(state, 0, options);
    if (two_opt.best_tour.empty()) {
      std::cout << "2-opt found no tour (" << FormatDuration(ElapsedMs(t_2opt))
                << ")\n";
    } else {
      initial_tour = GtspTourAlongStops(gtsp, two_opt.best_tour);
      std::cout << "2-opt tour: " << TimeSinceServiceStart{two_opt.best_val}
                << " with flex steps; along the GTSP: "
                << (initial_tour
                        ? TimeSinceServiceStart{initial_tour->cost_seconds}
                              .ToString()
                        : std::string("infeasible"))
                << " (" << FormatDuration(ElapsedMs(t_2opt)) << ")\n";
      if (initial_tour && !start_time && !free_start_time) {
        start_time = gtsp.vertices[initial_tour->tour[1]].time;
      }
    }
    std::cout << std::flush;
  }

  if (start_time) {
    gtsp = BuildGtsp(completed, state.boundary, *start_time);
    clusters = GtspClustersFromRequired(gtsp, state.required);
    std::cout << "Fixed start time " << *start_time << ": " << gtsp.edges.size()
              << " GTSP edges\n";
    if (initial_tour) {
      std::vector<StopId> order;
      for (int v : initial_tour->tour) {
        order.push_back(gtsp.vertices[v].stop);
      }
      initial_tour = GtspTourAlongStops(gtsp, order);
      std::cout << "  2-opt order from that start: "
                << (initial_tour
                        ? TimeSinceServiceStart{initial_tour->cost_seconds}
                              .ToString()
                        : std::string("infeasible"))
                << "\n";
    }
    std::cout << std::flush;
  }

  auto t1 = std::chrono::steady_clock::now();
  std::optional<GtspSolution> solution = SolveGtspWithConcorde(
      gtsp, clusters, initial_tour, quiet ? nullptr : &std::cerr
  );
  int solve_ms = ElapsedMs(t1);

  if (!solution) {
    std::cout << "\nNo tour exists using scheduled steps only ("
              << FormatDuration(solve_ms) << ")\n";
  } else {
    std::cout << "\nGTSP optimum: " << TimeSinceServiceStart{solution->cost_seconds}
              << " (" << FormatDuration(solve_ms) << ")\n";
    std::cout << "Tour (" << solution->tour.size() << " vertices):\n";
    for (int v : solution->tour) {
      const GtspVertex& vertex = gtsp.vertices[v];
      std::cout << "  " << std::left << std::setw(10) << vertex.time
                << state.StopName(vertex.stop) << "\n";
    }
  }

  if (run_hk) {
    auto t2 = std::chrono::steady_clock::now();
    HeldKarpDPResult hk = HeldKarpDPSolve(state, 0, &std::cerr);
    std::cout << "\nHeld-Karp optimum (with flex steps): "
              << TimeSinceServiceStart{hk.best_val} << " ("
              << FormatDuration(ElapsedMs(t2)) << ")\n";
  }

  return 0;
}
