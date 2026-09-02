#pragma once

#include <optional>
#include <ostream>
#include <vector>

#include "solver/data.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct GlkhOptions {
  // MAX_TRIALS for the main LKH run. <= 0 means omit the parameter, which
  // makes LKH default to the instance dimension (can be very slow for large
  // time-expanded instances).
  int max_trials = 200;

  // Number of independent LKH runs.
  int runs = 1;

  int seed = 1;

  // TIME_LIMIT (seconds) for each LKH run. Unset means no limit.
  std::optional<double> time_limit_seconds;

  // If > 0, keep at most this many time vertices per stop (evenly sampled
  // from the sorted times). Bounds the AGTSP dimension at the cost of
  // solution quality.
  int max_times_per_stop = 0;
};

struct GlkhSolution {
  // Exact duration of the best tour: the optimal duration achievable in the
  // completed graph (flex steps included) for the visit order GLKH found.
  int best_val;

  // The visited stops in visit order, from the boundary START to the boundary
  // END, in the ProblemState's stop ids.
  std::vector<StopId> best_tour;

  // Objective value of the AGTSP tour as scored by the time-expanded graph.
  // An upper bound on best_val (the time-expanded graph excludes flex steps
  // between middle stops and may have thinned arrival times).
  int gtsp_value;

  // Size of the AGTSP instance that was solved.
  int num_vertices;
  int num_clusters;
};

// Heuristically solves the tour problem by encoding it as an asymmetric
// generalized TSP over (stop, arrival time) vertices and running GLKH on it.
//
// Vertices are (stop, time) pairs over the shortest-path-completed graph
// between required stops, where a stop's times are the scheduled arrivals to
// and departures from it; each required group is one GTSP cluster containing
// all of its stops' vertices, and START and END are singleton clusters. Flex
// steps between middle stops are not represented (only scheduled steps); flex
// steps from START and to END are.
//
// Requires the GLKH and LKH executables on PATH (see third_party/glkh.nix).
// Creates temporary files under glkh_work/ in the current directory.
//
// Returns nullopt if the problem is infeasible or if GLKH could not find a
// tour that is feasible in the time-expanded graph (which can happen when a
// tour exists only via flex steps between middle stops).
//
// If glkh_log is non-null, GLKH's output is written to it.
std::optional<GlkhSolution> SolveGtspWithGlkh(
    const ProblemState& state,
    const GlkhOptions& options = {},
    std::ostream* glkh_log = nullptr
);

}  // namespace vats5
