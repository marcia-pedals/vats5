#include <CLI/CLI.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>

#include "solver/data.h"
#include "solver/embiggener.h"
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

struct BnbState {
  int lb;
  std::vector<PointBound> known_points;
  std::unordered_set<PointInstant> forbidden_points;
  EmbiggenerState state;

  bool operator<(const BnbState& other) const {
    return known_points.size() < other.known_points.size();
  }
};

void Bnb(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    PathCache& cache,
    const BnbState& initial,
    TimeSinceServiceStart t0,
    int ub
) {
  std::vector<BnbState> q;
  q.push_back(initial);

  while (true) {
    std::pop_heap(q.begin(), q.end());
    BnbState cur = std::move(q.back());
    q.pop_back();

    std::cout << "take " << TimeSinceServiceStart{cur.lb} << "\n";
    if (cur.lb >= ub) {
      std::cout << "  pruned: reached ub\n";
      continue;
    }

    std::optional<RefineResult> refine_result =
        DoRefine(problem, completed, cache, cur.state, t0, ub);
    if (!refine_result.has_value()) {
      std::cout << "  pruned: no refine result\n";
      continue;
    }

    std::cout << "  refine result: " << TimeSinceServiceStart{refine_result->lb}
              << " / " << TimeSinceServiceStart{refine_result->ub} << "\n";
    ub = std::min(refine_result->ub, ub);
    std::cout << "  best ub: " << TimeSinceServiceStart{ub} << "\n";

    // cur.known_points is START, k points, END
    // refine_result.ub_tour is START, k points, first not-known point, ..., END
    for (int i = 0; i + 1 < cur.known_points.size(); ++i) {
      if (!(refine_result->ub_tour[i].s == cur.known_points[i].s) ||
          !(refine_result->ub_tour[i].t >= cur.known_points[i].t_lo) ||
          !(refine_result->ub_tour[i].t <= cur.known_points[i].t_hi)) {
        for (int j = 0; j + 1 < cur.known_points.size(); ++j) {
          std::cout << "    " << j << ". "
                    << problem.StopName(refine_result->ub_tour[j].s) << " @ "
                    << refine_result->ub_tour[j].t << " - "
                    << problem.StopName(cur.known_points[j].s) << " @ ["
                    << cur.known_points[j].t_lo << ", "
                    << cur.known_points[j].t_hi << "]\n";
        }
        assert(false);
      }
    }
    PointInstant first_not_known_point =
        refine_result->ub_tour[cur.known_points.size() - 1];
    std::cout << "  next constraint: "
              << problem.StopName(first_not_known_point.s) << " @ "
              << first_not_known_point.t << "\n";

    {
      std::vector<PointBound> known_points = cur.known_points;
      known_points.insert(
          known_points.end() - 1,
          PointBound{
              first_not_known_point.s,
              first_not_known_point.t,
              first_not_known_point.t
          }
      );
      BnbState constrained_require{
          .lb = refine_result->lb,
          .known_points = known_points,
          .forbidden_points = cur.forbidden_points,
          .state = ConstrainEmbiggenerState(
              problem,
              completed,
              cache,
              cur.state,
              known_points,
              cur.forbidden_points
          ),
      };
      q.push_back(std::move(constrained_require));
      std::push_heap(q.begin(), q.end());
    }

    {
      std::unordered_set<PointInstant> forbidden_points = cur.forbidden_points;
      forbidden_points.insert(first_not_known_point);
      BnbState constrained_require{
          .lb = refine_result->lb,
          .known_points = cur.known_points,
          .forbidden_points = forbidden_points,
          .state = ConstrainEmbiggenerState(
              problem,
              completed,
              cache,
              cur.state,
              cur.known_points,
              forbidden_points
          ),
      };
      q.push_back(std::move(constrained_require));
      std::push_heap(q.begin(), q.end());
    }
  }
}

int main(int argc, char* argv[]) {
  CLI::App app{"Refine tool"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  std::string gtfs_stop_id_str;
  app.add_option("gtfs_stop_id", gtfs_stop_id_str, "GTFS stop ID")->required();

  std::string time_str;
  app.add_option("time", time_str, "Departure time in hh:mm:ss format")
      ->required();

  std::string lb_str;
  app.add_option(
         "--lb",
         lb_str,
         "Lower bound in hh:mm:ss format (relative to departure time)"
  )
      ->required();

  std::string ub_str;
  app.add_option(
         "--ub",
         ub_str,
         "Upper bound in hh:mm:ss format (relative to departure time)"
  )
      ->required();

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
  std::cout << "required size: " << problem.required.size() << "\n";

  StopId s0 = FindStopByGtfsId(problem, gtfs_stop_id_str);
  std::cout << "s0: " << problem.StopName(s0) << "\n";

  TimeSinceServiceStart t0 = TimeSinceServiceStart::Parse(time_str);
  std::cout << "t0: " << t0 << "\n";

  int lb_rel = TimeSinceServiceStart::Parse(lb_str).seconds;
  TimeSinceServiceStart t_lb{t0.seconds + lb_rel};
  int ub_rel = TimeSinceServiceStart::Parse(ub_str).seconds;
  TimeSinceServiceStart t_ub{t0.seconds + ub_rel};

  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());

  std::vector<PointBound> known_points{
      PointBound{problem.boundary.start, t0, t0},
      PointBound{s0, t0, t0},
      PointBound{problem.boundary.end, t_lb, t_ub},
  };
  std::unordered_set<PointInstant> forbidden_points;
  PathCache cache;
  EmbiggenerState state = MakeEmbiggenerState(
      problem, completed, cache, known_points, forbidden_points
  );

  BnbState initial{
      .lb = 0,
      .known_points = known_points,
      .forbidden_points = forbidden_points,
      .state = state,
  };
  Bnb(problem, completed, cache, initial, t0, ub_rel);

  // RefineResult refine_result = DoRefine(problem, completed, state, t0,
  // ub_rel); std::cout << "  lb: " << TimeSinceServiceStart{refine_result.lb}
  // << "\n"; std::cout << "  ub: " << TimeSinceServiceStart{refine_result.ub}
  // << "\n";

  return 0;
}
