#include "solver/glkh.h"

#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <charconv>
#include <climits>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "solver/pair_steps.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tour_paths.h"

namespace vats5 {
namespace {

// Weight used for edges that must never be taken. Must be larger than any
// real tour duration, and small enough that GLKH's internal Noon-Bean shift
// (INT_MAX / 4 / PRECISION per edge) doesn't overflow when added to it.
constexpr int kForbiddenWeight = 10000000;

constexpr int kUnreachable = kPairStepsUnreachable;

// A vertex of the time-expanded AGTSP: arrived at `stop` at `time`. START and
// END are represented as separate vertices, not through this struct.
struct TimeVertex {
  StopId stop;
  int time;
};

// The time-expanded AGTSP instance over the compacted completed graph.
struct GtspInstance {
  ProblemBoundary boundary;  // In compacted stop ids.
  CompactStopIdsResult compact;

  // Dense pair table indexed a * n_stops + b.
  std::vector<PairSteps> pairs;
  int n_stops;

  // Vertex 0 is START, vertices [1, 1 + middle.size()) are the middle
  // TimeVertexes (stop-major, time-ascending), the last vertex is END.
  std::vector<TimeVertex> middle;

  // cluster_of[v] is the 0-based cluster of vertex v. Cluster 0 is {START},
  // the last cluster is {END}.
  std::vector<int> cluster_of;
  int n_clusters;

  int NumVertices() const { return static_cast<int>(middle.size()) + 2; }
  int StartVertex() const { return 0; }
  int EndVertex() const { return NumVertices() - 1; }

  const PairSteps& Pair(StopId a, StopId b) const {
    return pairs[static_cast<size_t>(a.v) * n_stops + b.v];
  }

  // Directed edge weight between AGTSP vertices, kForbiddenWeight if the edge
  // must not be taken. Weights within a cluster are ignored by GLKH.
  int Weight(int u, int v) const {
    if (u == v) {
      return kForbiddenWeight;
    }
    int start = StartVertex();
    int end = EndVertex();
    if (u == end) {
      // The only way out of END is the zero-weight cycle edge back to START.
      return v == start ? 0 : kForbiddenWeight;
    }
    if (v == start) {
      return kForbiddenWeight;
    }
    if (u == start) {
      if (v == end) {
        // Would make the middle clusters follow END; never valid when middle
        // clusters exist (and this function is only used when they do).
        return kForbiddenWeight;
      }
      // Leave START as late as possible while still arriving by the vertex
      // time (waiting at the stop is allowed).
      const TimeVertex& b = middle[v - 1];
      int elapsed = Pair(boundary.start, b.stop).MinElapsedArrivingBy(b.time);
      return elapsed == kUnreachable ? kForbiddenWeight : elapsed;
    }
    const TimeVertex& a = middle[u - 1];
    if (v == end) {
      int arrival = Pair(a.stop, boundary.end).EarliestArrival(a.time);
      return arrival == kUnreachable ? kForbiddenWeight : arrival - a.time;
    }
    const TimeVertex& b = middle[v - 1];
    if (a.stop == b.stop) {
      // Same cluster; weight is ignored by GLKH.
      return kForbiddenWeight;
    }
    // Scheduled steps only between middle stops: reachable if some scheduled
    // step departing at or after a.time arrives by b.time (arriving early and
    // waiting is allowed). The elapsed time is measured vertex to vertex.
    int arrival = Pair(a.stop, b.stop).EarliestScheduledArrival(a.time);
    if (arrival == kUnreachable || arrival > b.time) {
      return kForbiddenWeight;
    }
    return b.time - a.time;
  }
};

// Builds the time-expanded AGTSP instance. Returns nullopt if the problem is
// infeasible under this encoding (some group has no reachable vertex, or a
// required group vanished from the completed graph).
std::optional<GtspInstance> BuildGtspInstance(
    const ProblemState& state, const GlkhOptions& options, std::ostream* log
) {
  GtspInstance inst;

  std::vector<Step> completed_steps =
      CompleteShortestPathsGraph(
          state.minimal,
          state.required.AllFlat(),
          HorizonCoveringAllDepartures(state.minimal)
      )
          .AllMergedSteps();
  StepsAdjacencyList completed = MakeAdjacencyList(std::move(completed_steps));
  inst.compact = CompactStopIds(completed);

  std::optional<RequiredStops> required =
      RemapRequiredStops(inst.compact.mapping, state.required);
  if (!required.has_value()) {
    return std::nullopt;
  }

  std::optional<StopId> start =
      inst.compact.mapping.MapToNew(state.boundary.start);
  std::optional<StopId> end = inst.compact.mapping.MapToNew(state.boundary.end);
  assert(start.has_value());
  assert(end.has_value());
  inst.boundary = ProblemBoundary{.start = *start, .end = *end};

  int n = inst.compact.list.NumStops();
  inst.n_stops = n;

  inst.pairs = BuildPairStepsTable(inst.compact.list);

  // Time vertices for every stop in a middle group (a group not containing
  // START or END). Times come from scheduled steps only; flex steps between
  // middle stops are not represented.
  //
  // A stop's times are the arrivals of scheduled steps into it plus the
  // departures of scheduled steps out of it. Intermediate vertex times cancel
  // out of the tour cost (edge weights telescope), but the first vertex after
  // START must be able to sit exactly on the departure the tour catches
  // (reaching it via a flex boundary edge "just in time"); without departure
  // times every tour would pay for waiting from some earlier arrival.
  StopId start_rep = required->Representative(inst.boundary.start);
  StopId end_rep = required->Representative(inst.boundary.end);

  auto is_middle = [&](StopId stop) {
    StopId rep = required->Representative(stop);
    return rep != start_rep && rep != end_rep;
  };

  std::vector<std::vector<int>> times_by_stop(n);
  for (StopId a{0}; a.v < n; ++a.v) {
    bool a_is_middle = is_middle(a);
    for (const StepGroup& group : inst.compact.list.GetGroups(a)) {
      StopId b = group.destination_stop;
      bool b_is_middle = is_middle(b);
      if (!a_is_middle && !b_is_middle) {
        continue;
      }
      for (const AdjacencyListStep& step : inst.compact.list.GetSteps(group)) {
        if (a_is_middle) {
          times_by_stop[a.v].push_back(step.origin_time.seconds);
        }
        if (b_is_middle) {
          times_by_stop[b.v].push_back(step.destination_time.seconds);
        }
      }
    }
  }

  // Clusters: 0 = {START}, then one per middle group (in Groups() order),
  // last = {END}.
  std::vector<std::vector<StopId>> groups = required->Groups();
  std::vector<int> cluster_of_stop(n, -1);
  int n_mid_clusters = 0;
  for (const std::vector<StopId>& group : groups) {
    StopId rep = required->Representative(group[0]);
    if (rep == start_rep || rep == end_rep) {
      continue;
    }
    for (StopId stop : group) {
      cluster_of_stop[stop.v] = 1 + n_mid_clusters;
    }
    ++n_mid_clusters;
  }
  inst.n_clusters = n_mid_clusters + 2;

  inst.cluster_of.push_back(0);  // START
  for (StopId s{0}; s.v < n; ++s.v) {
    std::vector<int>& times = times_by_stop[s.v];
    if (times.empty()) {
      continue;
    }
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());
    if (options.max_times_per_stop > 0 &&
        static_cast<int>(times.size()) > options.max_times_per_stop) {
      // Evenly sample, always keeping the first and last time.
      std::vector<int> thinned;
      int keep = options.max_times_per_stop;
      thinned.reserve(keep);
      for (int i = 0; i < keep; ++i) {
        thinned.push_back(
            times[static_cast<size_t>(i) * (times.size() - 1) / (keep - 1)]
        );
      }
      thinned.erase(std::unique(thinned.begin(), thinned.end()), thinned.end());
      times = std::move(thinned);
    }
    for (int t : times) {
      inst.middle.push_back(TimeVertex{s, t});
      inst.cluster_of.push_back(cluster_of_stop[s.v]);
    }
  }
  inst.cluster_of.push_back(inst.n_clusters - 1);  // END

  // Every middle cluster needs at least one vertex; otherwise the encoding
  // cannot visit it (it may still be visitable via flex steps, which this
  // encoding excludes).
  std::vector<bool> cluster_seen(inst.n_clusters, false);
  for (int c : inst.cluster_of) {
    cluster_seen[c] = true;
  }
  for (int c = 0; c < inst.n_clusters; ++c) {
    if (!cluster_seen[c]) {
      if (log != nullptr) {
        *log << "GLKH encoding infeasible: a required group has no scheduled "
                "steps\n";
      }
      return std::nullopt;
    }
  }

  return inst;
}

void AppendInt(std::string& buf, int value) {
  char tmp[16];
  auto [ptr, ec] = std::to_chars(tmp, tmp + sizeof(tmp), value);
  assert(ec == std::errc());
  buf.append(tmp, ptr - tmp);
}

// Writes the AGTSP instance in GTSPLIB format (explicit full matrix).
void WriteGtspFile(const std::string& path, const GtspInstance& inst) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("Failed to open GTSP problem file: " + path);
  }
  int n = inst.NumVertices();
  out << "NAME : vats5\n";
  out << "TYPE : AGTSP\n";
  out << "DIMENSION : " << n << "\n";
  out << "GTSP_SETS : " << inst.n_clusters << "\n";
  out << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  out << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  out << "EDGE_WEIGHT_SECTION\n";

  std::string buf;
  buf.reserve(static_cast<size_t>(n) * 9 + 16);
  for (int u = 0; u < n; ++u) {
    buf.clear();
    for (int v = 0; v < n; ++v) {
      AppendInt(buf, inst.Weight(u, v));
      buf.push_back(v + 1 < n ? ' ' : '\n');
    }
    out << buf;
  }

  out << "GTSP_SET_SECTION\n";
  for (int c = 0; c < inst.n_clusters; ++c) {
    buf.clear();
    AppendInt(buf, c + 1);
    for (int v = 0; v < n; ++v) {
      if (inst.cluster_of[v] == c) {
        buf.push_back(' ');
        AppendInt(buf, v + 1);  // GTSPLIB vertex ids are 1-based.
      }
    }
    buf.append(" -1\n");
    out << buf;
  }
  out << "EOF\n";
  if (!out) {
    throw std::runtime_error("Failed to write GTSP problem file: " + path);
  }
}

void WriteParFile(
    const std::string& path,
    const std::string& problem_file,
    const std::string& tour_file,
    const GlkhOptions& options
) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("Failed to open GLKH parameter file: " + path);
  }
  // Modeled on GLKH's runGLKH script.
  out << "PROBLEM_FILE = " << problem_file << "\n";
  out << "ASCENT_CANDIDATES = 500\n";
  out << "CANDIDATE_SET_TYPE = POPMUSIC\n";
  out << "POPMUSIC_SAMPLE_SIZE = 100\n";
  out << "POPMUSIC_MAX_NEIGHBORS = 30\n";
  out << "POPMUSIC_TRIALS = 0\n";
  out << "INITIAL_PERIOD = 1000\n";
  out << "MAX_CANDIDATES = 30\n";
  if (options.max_trials > 0) {
    out << "MAX_TRIALS = " << options.max_trials << "\n";
  }
  if (options.time_limit_seconds.has_value()) {
    out << "TIME_LIMIT = " << *options.time_limit_seconds << "\n";
  }
  out << "OUTPUT_TOUR_FILE = " << tour_file << "\n";
  out << "POPULATION_SIZE = 1\n";
  out << "PRECISION = 10\n";
  out << "RUNS = " << options.runs << "\n";
  out << "SEED = " << options.seed << "\n";
  out << "TRACE_LEVEL = 1\n";
}

// Runs `GLKH params.par` with `temp_dir` as working directory, streaming its
// output to glkh_log. Throws if the process cannot be run or exits with a
// non-zero status.
void RunGlkh(const std::string& temp_dir, std::ostream* glkh_log) {
  int pipefd[2];
  if (pipe(pipefd) != 0) {
    throw std::runtime_error("Failed to create pipe for GLKH");
  }

  pid_t pid = fork();
  if (pid < 0) {
    close(pipefd[0]);
    close(pipefd[1]);
    throw std::runtime_error("Failed to fork for GLKH");
  }

  if (pid == 0) {
    // Child: redirect stdout+stderr to pipe, chdir, exec GLKH.
    close(pipefd[0]);
    dup2(pipefd[1], STDOUT_FILENO);
    dup2(pipefd[1], STDERR_FILENO);
    close(pipefd[1]);
    if (chdir(temp_dir.c_str()) != 0) {
      _exit(127);
    }
    execlp("GLKH", "GLKH", "params.par", nullptr);
    _exit(127);
  }

  close(pipefd[1]);
  FILE* pipe_read = fdopen(pipefd[0], "r");
  std::string output;
  char buffer[256];
  while (fgets(buffer, sizeof(buffer), pipe_read) != nullptr) {
    if (glkh_log != nullptr) {
      *glkh_log << buffer << std::flush;
    }
    output += buffer;
  }
  fclose(pipe_read);

  int status = 0;
  waitpid(pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    throw std::runtime_error(
        "GLKH failed (is it on PATH?). Output:\n" + output
    );
  }
}

// Reads the g-tour (1-based vertex ids) from GLKH's OUTPUT_TOUR_FILE.
std::vector<int> ReadGTour(const std::string& tour_path, int n_clusters) {
  std::ifstream in(tour_path);
  if (!in) {
    throw std::runtime_error("Failed to open GLKH tour file: " + tour_path);
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind("TOUR_SECTION", 0) == 0) {
      break;
    }
  }
  std::vector<int> tour;
  tour.reserve(n_clusters);
  int v;
  while (in >> v && v != -1) {
    tour.push_back(v - 1);  // Back to 0-based vertex ids.
  }
  if (static_cast<int>(tour.size()) != n_clusters) {
    throw std::runtime_error(
        "GLKH tour has " + std::to_string(tour.size()) +
        " vertices, expected " + std::to_string(n_clusters)
    );
  }
  return tour;
}

}  // namespace

std::optional<GlkhSolution> SolveGtspWithGlkh(
    const ProblemState& state,
    const GlkhOptions& options,
    std::ostream* glkh_log
) {
  std::optional<GtspInstance> maybe_inst =
      BuildGtspInstance(state, options, glkh_log);
  if (!maybe_inst.has_value()) {
    return std::nullopt;
  }
  const GtspInstance& inst = *maybe_inst;

  MinimalPathSetCache path_cache(inst.compact.list);

  // Turns a tour over compacted stop ids into a solution: computes the exact
  // best duration for that visit order and maps back to original stop ids.
  auto make_solution = [&](const std::vector<StopId>& compact_tour,
                           int gtsp_value) -> std::optional<GlkhSolution> {
    std::vector<Path> tour_paths =
        ComputeMinimalFeasiblePathsAlong(compact_tour, path_cache);
    if (tour_paths.empty()) {
      throw std::logic_error(
          "SolveGtspWithGlkh: no path along a tour whose edges were feasible"
      );
    }
    int best_val = kUnreachable;
    for (const Path& path : tour_paths) {
      best_val = std::min(best_val, path.DurationSeconds());
    }
    GlkhSolution solution;
    solution.best_val = best_val;
    for (StopId stop : compact_tour) {
      solution.best_tour.push_back(
          inst.compact.mapping.new_to_original[stop.v]
      );
    }
    solution.gtsp_value = gtsp_value;
    solution.num_vertices = inst.NumVertices();
    solution.num_clusters = inst.n_clusters;
    return solution;
  };

  if (inst.n_clusters == 2) {
    // No middle clusters: the tour is just START -> END; no need to run GLKH.
    const PairSteps& pair = inst.Pair(inst.boundary.start, inst.boundary.end);
    int best = pair.flex_seconds >= 0 ? pair.flex_seconds : kUnreachable;
    for (size_t i = 0; i < pair.deps.size(); ++i) {
      best = std::min(best, pair.arrs[i] - pair.deps[i]);
    }
    if (best == kUnreachable) {
      return std::nullopt;
    }
    return make_solution({inst.boundary.start, inst.boundary.end}, best);
  }

  // Create temp directory under glkh_work/ in cwd so it works in the Claude
  // sandbox (which restricts /tmp) and doesn't clutter the cwd. GLKH also
  // needs a TMP/ subdirectory in its working directory.
  if (mkdir("glkh_work", 0755) != 0 && errno != EEXIST) {
    throw std::runtime_error("Failed to create glkh_work directory");
  }
  std::string temp_dir = "glkh_work/vats5_gtsp_XXXXXX";
  if (mkdtemp(temp_dir.data()) == nullptr) {
    throw std::runtime_error("Failed to create temp directory");
  }
  auto cleanup_temp = [&]() { std::filesystem::remove_all(temp_dir); };

  std::vector<int> gtour;
  try {
    if (mkdir((temp_dir + "/TMP").c_str(), 0755) != 0) {
      throw std::runtime_error("Failed to create TMP directory");
    }
    WriteGtspFile(temp_dir + "/problem.gtsp", inst);
    WriteParFile(
        temp_dir + "/params.par", "problem.gtsp", "solution.tour", options
    );
    RunGlkh(temp_dir, glkh_log);
    gtour = ReadGTour(temp_dir + "/solution.tour", inst.n_clusters);
  } catch (...) {
    cleanup_temp();
    throw;
  }
  cleanup_temp();

  // Rotate the cyclic g-tour so it begins at START.
  auto start_it = std::find(gtour.begin(), gtour.end(), inst.StartVertex());
  if (start_it == gtour.end()) {
    throw std::runtime_error("GLKH tour does not contain the START vertex");
  }
  std::rotate(gtour.begin(), start_it, gtour.end());
  if (gtour.back() != inst.EndVertex()) {
    // The tour does not have the START ... END structure, which can only
    // happen through forbidden edges: no feasible tour was found.
    if (glkh_log != nullptr) {
      *glkh_log << "GLKH tour does not end at END; no feasible tour found\n";
    }
    return std::nullopt;
  }

  // Score the tour and reject it if it uses a forbidden edge.
  int gtsp_value = 0;
  for (size_t i = 0; i + 1 < gtour.size(); ++i) {
    int w = inst.Weight(gtour[i], gtour[i + 1]);
    if (w >= kForbiddenWeight) {
      if (glkh_log != nullptr) {
        *glkh_log << "GLKH tour uses a forbidden edge; no feasible tour "
                     "found\n";
      }
      return std::nullopt;
    }
    gtsp_value += w;
  }

  std::vector<StopId> compact_tour;
  compact_tour.reserve(gtour.size());
  compact_tour.push_back(inst.boundary.start);
  for (size_t i = 1; i + 1 < gtour.size(); ++i) {
    compact_tour.push_back(inst.middle[gtour[i] - 1].stop);
  }
  compact_tour.push_back(inst.boundary.end);

  return make_solution(compact_tour, gtsp_value);
}

}  // namespace vats5
