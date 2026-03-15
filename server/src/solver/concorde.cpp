#include "solver/concorde.h"

#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace vats5 {
namespace {

constexpr int kBruteForceThreshold = 5;

// Solve small ATSP instances by enumerating all permutations.
// Returns nullopt if no valid Hamiltonian cycle exists or the best tour
// cost >= ub (strict upper bound, matching Concorde's -u semantics).
std::optional<ConcordeSolution> SolveTspBruteForce(
    const RelaxedAdjacencyList& relaxed, std::optional<int> ub
) {
  int n = relaxed.NumStops();

  // Build perm = [1, 2, ..., n-1]. We fix node 0 as the start to avoid
  // checking rotations of the same cycle.
  std::vector<int> perm(n - 1);
  std::iota(perm.begin(), perm.end(), 1);

  std::optional<ConcordeSolution> best;

  do {
    // Compute tour cost for cycle: 0 -> perm[0] -> ... -> perm[n-2] -> 0
    int cost = 0;
    bool valid = true;

    // Edge from 0 to perm[0]
    auto w = relaxed.GetWeight(StopId{0}, StopId{perm[0]});
    if (!w.has_value()) {
      continue;
    }
    cost += *w;

    // Edges along the permutation
    for (int i = 0; i + 1 < n - 1; ++i) {
      w = relaxed.GetWeight(StopId{perm[i]}, StopId{perm[i + 1]});
      if (!w.has_value()) {
        valid = false;
        break;
      }
      cost += *w;
    }
    if (!valid) continue;

    // Edge back to 0
    w = relaxed.GetWeight(StopId{perm[n - 2]}, StopId{0});
    if (!w.has_value()) {
      continue;
    }
    cost += *w;

    if (!best.has_value() || cost < best->optimal_value) {
      std::vector<StopId> tour;
      tour.reserve(n);
      tour.push_back(StopId{0});
      for (int v : perm) {
        tour.push_back(StopId{v});
      }
      best = ConcordeSolution{.tour = std::move(tour), .optimal_value = cost};
    }
  } while (std::next_permutation(perm.begin(), perm.end()));

  if (best.has_value() && ub.has_value() && best->optimal_value >= *ub) {
    return std::nullopt;
  }

  return best;
}

// Must be much larger than any legitimate edge weight (after negative-weight
// offset). A too-small value distorts Concorde's LP relaxation and
// tie-breaking, which cascades through reweighting to weaken the LB.
// Concorde's internal edge weight limit is 32768.
constexpr int kForbiddenEdgeWeight = 1000000;
constexpr int kInterVertexOffset = 11000;

// Helper class for computing edge weights in the doubled graph.
// Uses vertex doubling to convert asymmetric TSP to symmetric TSP.
// Vertices: 2i = in(i), 2i+1 = out(i)
class DoubledGraphWeights {
 public:
  explicit DoubledGraphWeights(const RelaxedAdjacencyList& relaxed)
      : n_(relaxed.NumStops()), edge_weights_(n_ * n_, kForbiddenEdgeWeight) {
    for (int origin = 0; origin < n_; ++origin) {
      for (const auto& edge : relaxed.GetEdges(StopId{origin})) {
        edge_weights_[origin * n_ + edge.destination_stop.v] =
            edge.weight_seconds;
      }
    }

    // Offset all weights so the minimum is 0. Concorde doesn't handle negative
    // edge weights correctly.
    int min_weight = 0;
    for (int w : edge_weights_) {
      if (w < kForbiddenEdgeWeight) {
        min_weight = std::min(min_weight, w);
      }
    }
    negative_weight_offset_ = -min_weight;
    if (negative_weight_offset_ > 0) {
      for (int& w : edge_weights_) {
        if (w < kForbiddenEdgeWeight) {
          w += negative_weight_offset_;
          if (w >= kForbiddenEdgeWeight) {
            throw EdgeWeightOverflow(
                "Edge weight " + std::to_string(w - negative_weight_offset_) +
                " + offset " + std::to_string(negative_weight_offset_) + " = " +
                std::to_string(w) + " >= kForbiddenEdgeWeight " +
                std::to_string(kForbiddenEdgeWeight)
            );
          }
        }
      }
    }
  }

  int NumStops() const { return n_; }
  int DoubledN() const { return 2 * n_; }
  int NegativeWeightOffset() const { return negative_weight_offset_; }

  // Get asymmetric weight from original vertex `from` to `to`.
  int GetAsymmetricWeight(int from, int to) const {
    return edge_weights_[from * n_ + to];
  }

  // Compute symmetric edge weight in doubled graph.
  // Requires a < b.
  int GetDoubledWeight(int a, int b) const {
    assert(a < b);
    int a_orig = a / 2;
    int b_orig = b / 2;
    bool a_is_in = (a % 2 == 0);
    bool b_is_in = (b % 2 == 0);

    if (a_orig == b_orig) {
      // Same original vertex: in(i) <-> out(i), weight 0
      return 0;
    }

    // Different original vertices
    if (!a_is_in && b_is_in) {
      // out(a_orig) <-> in(b_orig): asymmetric weight w(a_orig, b_orig)
      int w = GetAsymmetricWeight(a_orig, b_orig);
      if (w >= kForbiddenEdgeWeight) {
        return kForbiddenEdgeWeight;
      }
      return w + kInterVertexOffset;
    } else if (a_is_in && !b_is_in) {
      // in(a_orig) <-> out(b_orig): asymmetric weight w(b_orig, a_orig)
      int w = GetAsymmetricWeight(b_orig, a_orig);
      if (w >= kForbiddenEdgeWeight) {
        return kForbiddenEdgeWeight;
      }
      return w + kInterVertexOffset;
    } else {
      // Both in or both out: forbidden
      return kForbiddenEdgeWeight;
    }
  }

  // Check if an edge in the doubled tour uses a forbidden weight.
  bool IsForbiddenEdge(int a, int b) const {
    if (a > b) std::swap(a, b);
    return GetDoubledWeight(a, b) >= kForbiddenEdgeWeight;
  }

 private:
  int n_;
  int negative_weight_offset_ = 0;
  std::vector<int> edge_weights_;
};

// Output a RelaxedAdjacencyList as a Concorde TSP instance using UPPER_ROW
// format.
void OutputConcordeTsp(std::ostream& out, const DoubledGraphWeights& weights) {
  int doubled_n = weights.DoubledN();

  // Output TSP header
  out << "NAME: vats5\n";
  out << "TYPE: TSP\n";
  out << "DIMENSION: " << doubled_n << "\n";
  out << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
  out << "EDGE_WEIGHT_FORMAT: UPPER_ROW\n";
  out << "EDGE_WEIGHT_SECTION\n";

  // Output upper triangular matrix (excluding diagonal)
  // For row i, output weights to j for all j > i
  for (int i = 0; i < doubled_n; ++i) {
    for (int j = i + 1; j < doubled_n; ++j) {
      int weight = weights.GetDoubledWeight(i, j);
      out << weight;
      if (j < doubled_n - 1) {
        out << " ";
      }
    }
    if (i < doubled_n - 1) {
      out << "\n";
    }
  }
  out << "\nEOF\n";
}

// Read the raw doubled tour from Concorde's solution file.
std::vector<int> ReadDoubledTour(const std::string& solution_path) {
  std::ifstream in(solution_path);
  if (!in) {
    throw std::runtime_error("Failed to open solution file: " + solution_path);
  }

  int doubled_n;
  in >> doubled_n;

  std::vector<int> doubled_tour;
  doubled_tour.reserve(doubled_n);
  for (int i = 0; i < doubled_n; ++i) {
    int node;
    in >> node;
    doubled_tour.push_back(node);
  }
  return doubled_tour;
}

// Check if any edge in the doubled tour uses a forbidden weight.
bool DoubledTourUsesForbiddenEdge(
    const std::vector<int>& doubled_tour, const DoubledGraphWeights& weights
) {
  int doubled_n = static_cast<int>(doubled_tour.size());
  for (int i = 0; i < doubled_n; ++i) {
    int a = doubled_tour[i];
    int b = doubled_tour[(i + 1) % doubled_n];
    if (weights.IsForbiddenEdge(a, b)) {
      return true;
    }
  }
  return false;
}

// Validate the doubled tour structure and extract the original tour.
std::vector<StopId> ValidateAndExtractTour(
    const std::vector<int>& doubled_tour
) {
  int doubled_n = static_cast<int>(doubled_tour.size());
  int n = doubled_n / 2;
  auto orig = [](int v) { return v / 2; };
  auto is_in = [](int v) { return v % 2 == 0; };

  auto format_tour = [&]() {
    std::ostringstream s;
    s << "Doubled tour: ";
    for (int v : doubled_tour) {
      s << (is_in(v) ? "in(" : "out(") << orig(v) << ") ";
    }
    return s.str();
  };

  // Find start of a same-vertex pair. Tour alternates: in/out pairs of same
  // vertex.
  int start_idx = -1;
  for (int i = 0; i < doubled_n; ++i) {
    if (orig(doubled_tour[i]) == orig(doubled_tour[(i + 1) % doubled_n])) {
      start_idx = i;
      break;
    }
  }
  if (start_idx == -1) {
    throw InvalidTourStructure("No in/out pair found. " + format_tour());
  }

  // Validate all pairs and extract tour in one pass.
  bool is_reversed = !is_in(doubled_tour[start_idx]);
  std::vector<StopId> tour;
  tour.reserve(n);

  for (int i = 0; i < n; ++i) {
    int v1 = doubled_tour[(start_idx + 2 * i) % doubled_n];
    int v2 = doubled_tour[(start_idx + 2 * i + 1) % doubled_n];

    if (orig(v1) != orig(v2)) {
      throw InvalidTourStructure(
          "Mismatched pair at position " + std::to_string(i) + ": " +
          std::to_string(orig(v1)) + " vs " + std::to_string(orig(v2)) + ". " +
          format_tour()
      );
    }
    if (is_in(v1) == is_in(v2)) {
      throw InvalidTourStructure(
          "Invalid in/out pattern at position " + std::to_string(i) + ". " +
          format_tour()
      );
    }
    tour.push_back(StopId{orig(v1)});
  }

  if (is_reversed) {
    std::reverse(tour.begin(), tour.end());
  }
  return tour;
}

std::optional<ConcordeSolution> SolveTspWithConcordeImpl(
    const RelaxedAdjacencyList& relaxed,
    std::optional<int> ub,
    std::ostream* tsp_log,
    int seed
) {
  DoubledGraphWeights weights(relaxed);
  int n = weights.NumStops();

  // Create temp directory under concorde_work/ in cwd so it works in Claude
  // sandbox (which restricts /tmp) and doesn't clutter the cwd.
  if (mkdir("concorde_work", 0755) != 0 && errno != EEXIST) {
    throw std::runtime_error("Failed to create concorde_work directory");
  }
  std::string temp_dir = "concorde_work/vats5_tsp_XXXXXX";
  if (mkdtemp(temp_dir.data()) == nullptr) {
    throw std::runtime_error("Failed to create temp directory");
  }
  std::string problem_path = temp_dir + "/problem";
  std::string solution_path = temp_dir + "/solution";

  auto cleanup_temp = [&]() { std::filesystem::remove_all(temp_dir); };

  // Write TSP problem to temp file
  {
    std::ofstream out(problem_path);
    OutputConcordeTsp(out, weights);
  }

  std::optional<int> concorde_ub;
  if (ub.has_value()) {
    concorde_ub =
        *ub + n * kInterVertexOffset + n * weights.NegativeWeightOffset();
  }

  // Invoke Concorde from temp dir so its temp files don't conflict when running
  // in parallel.
  //
  // We use fork/exec instead of popen so we have the child PID. Concorde
  // occasionally crashes with SIGABRT and hangs; having the PID lets us kill
  // it immediately and retry.
  std::string seed_str = std::to_string(seed);
  std::string ub_str;
  if (concorde_ub.has_value()) {
    ub_str = std::to_string(*concorde_ub);
  }

  int pipefd[2];
  if (pipe(pipefd) != 0) {
    cleanup_temp();
    throw std::runtime_error("Failed to create pipe for concorde");
  }

  pid_t pid = fork();
  if (pid < 0) {
    close(pipefd[0]);
    close(pipefd[1]);
    cleanup_temp();
    throw std::runtime_error("Failed to fork for concorde");
  }

  if (pid == 0) {
    // Child: redirect stdout+stderr to pipe, chdir, exec concorde.
    close(pipefd[0]);
    dup2(pipefd[1], STDOUT_FILENO);
    dup2(pipefd[1], STDERR_FILENO);
    close(pipefd[1]);
    if (chdir(temp_dir.c_str()) != 0) {
      _exit(127);
    }
    if (concorde_ub.has_value()) {
      execlp(
          "concorde",
          "concorde",
          "-s",
          seed_str.c_str(),
          "-u",
          ub_str.c_str(),
          "-x",
          "-o",
          "solution",
          "problem",
          nullptr
      );
    } else {
      execlp(
          "concorde",
          "concorde",
          "-s",
          seed_str.c_str(),
          "-x",
          "-o",
          "solution",
          "problem",
          nullptr
      );
    }
    _exit(127);
  }

  // Parent: read concorde output, watching for crashes.
  close(pipefd[1]);
  FILE* pipe_read = fdopen(pipefd[0], "r");

  std::string concorde_output;
  char buffer[256];
  bool crashed = false;
  while (fgets(buffer, sizeof(buffer), pipe_read) != nullptr) {
    if (tsp_log) {
      *tsp_log << buffer << std::flush;
    }
    concorde_output += buffer;
    if (strstr(buffer, "SIGABRT") != nullptr) {
      crashed = true;
      break;
    }
  }

  if (crashed) {
    kill(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
    fclose(pipe_read);
    cleanup_temp();
    throw ConcordeCrash(
        "Concorde crashed with SIGABRT. Output:\n" + concorde_output
    );
  }

  fclose(pipe_read);
  waitpid(pid, nullptr, 0);

  size_t pos = concorde_output.find("Optimal Solution:");
  if (pos == std::string::npos) {
    // throw std::runtime_error(
    //     "Concorde did not find optimal solution. Output:\n" + concorde_output
    // );
    // TODO: Consider whether this is really infeasible always or if there are
    // error cases we should detect and fail for.
    cleanup_temp();
    return std::nullopt;
  }
  int raw_optimal_value =
      static_cast<int>(std::round(std::stod(concorde_output.substr(pos + 17))));

  if (ub.has_value() && raw_optimal_value >= concorde_ub) {
    // Cleanup temp directory before returning
    cleanup_temp();
    return std::nullopt;
  }

  // Read the doubled tour
  std::vector<int> doubled_tour = ReadDoubledTour(solution_path);

  // If the optimal tour uses a forbidden edge, then say that the problem is
  // infeasible.
  //
  // TODO: Think harder about this. The tour could "cheat" and pay
  // kForbiddenEdgeCost somewhere to get a much cheaper tour somewhere else,
  // and thus we'd incorrectly say that the problem is infeasible when
  // actually there is a feasible solution. Seems unlikely to matter in
  // practice, because kForbiddenEdgeCost is so big.
  if (DoubledTourUsesForbiddenEdge(doubled_tour, weights)) {
    // Cleanup temp directory before returning
    cleanup_temp();
    return std::nullopt;
  }

  // Validate structure and extract original tour
  std::vector<StopId> tour = ValidateAndExtractTour(doubled_tour);

  // Subtract the offsets added during graph construction. A proper tour has
  // exactly n inter-vertex edges, each inflated by kInterVertexOffset and
  // by the negative-weight offset.
  int optimal_value = raw_optimal_value - n * kInterVertexOffset -
                      n * weights.NegativeWeightOffset();

  // Cleanup temp directory
  cleanup_temp();

  return ConcordeSolution{
      .tour = std::move(tour), .optimal_value = optimal_value
  };
}

}  // namespace

std::optional<ConcordeSolution> SolveTspWithConcorde(
    const RelaxedAdjacencyList& relaxed,
    std::optional<int> ub,
    std::ostream* tsp_log
) {
  if (relaxed.NumStops() < kBruteForceThreshold) {
    return SolveTspBruteForce(relaxed, ub);
  }

  constexpr int kMaxRetries = 5;
  constexpr int kBaseSeed = 43;
  for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
    try {
      return SolveTspWithConcordeImpl(
          relaxed, ub, tsp_log, kBaseSeed + attempt - 1
      );
    } catch (const InvalidTourStructure&) {
      // Don't retry - indicates insufficient kInterVertexOffset or a bug, not
      // transient.
      throw;
    } catch (const std::exception&) {
      if (attempt == kMaxRetries) {
        throw;
      }
    }
  }
  __builtin_unreachable();
}

}  // namespace vats5
