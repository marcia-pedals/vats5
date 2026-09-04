#include "solver/concorde.h"

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <numeric>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>

#include "solver/concorde_shim.h"

namespace vats5 {
namespace {

// Instances with fewer stops than this are solved by brute force. This must be
// at least 5: the doubled graph handed to Concorde then has >= 10 nodes, which
// is the minimum CCtsp_solve_dat handles (the concorde CLI special-cases
// smaller instances with Held-Karp, but the library entry point does not).
constexpr int kBruteForceThreshold = 5;
static_assert(kBruteForceThreshold >= 5);

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

// Per-solve scratch directory for Concorde's checkpoint files and log.
// Removed on destruction.
class WorkDir {
 public:
  WorkDir() {
    // Create under concorde_work/ in cwd so it works in Claude sandbox (which
    // restricts /tmp) and doesn't clutter the cwd.
    if (mkdir("concorde_work", 0755) != 0 && errno != EEXIST) {
      throw std::runtime_error("Failed to create concorde_work directory");
    }
    std::string temp_dir = "concorde_work/vats5_tsp_XXXXXX";
    if (mkdtemp(temp_dir.data()) == nullptr) {
      throw std::runtime_error("Failed to create temp directory");
    }
    // Absolute, because the shim chdirs into it and must be able to find it
    // regardless of cwd.
    path_ = std::filesystem::absolute(temp_dir).string();
  }

  ~WorkDir() {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  WorkDir(const WorkDir&) = delete;
  WorkDir& operator=(const WorkDir&) = delete;

  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

std::string ReadFile(const std::string& path) {
  std::ifstream in(path);
  return std::string(
      std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()
  );
}

std::optional<ConcordeSolution> SolveTspWithConcordeImpl(
    const RelaxedAdjacencyList& relaxed,
    std::optional<int> ub,
    std::ostream* tsp_log,
    int seed
) {
  DoubledGraphWeights weights(relaxed);
  int n = weights.NumStops();
  int doubled_n = weights.DoubledN();

  // Complete graph on the doubled vertices: every pair a < b.
  int ecount = doubled_n * (doubled_n - 1) / 2;
  std::vector<int> elist;
  std::vector<int> elen;
  elist.reserve(2 * ecount);
  elen.reserve(ecount);
  for (int a = 0; a < doubled_n; ++a) {
    for (int b = a + 1; b < doubled_n; ++b) {
      elist.push_back(a);
      elist.push_back(b);
      elen.push_back(weights.GetDoubledWeight(a, b));
    }
  }

  std::optional<int> concorde_ub;
  std::optional<double> concorde_ub_double;
  if (ub.has_value()) {
    concorde_ub =
        *ub + n * kInterVertexOffset + n * weights.NegativeWeightOffset();
    concorde_ub_double = static_cast<double>(*concorde_ub);
  }

  WorkDir work_dir;
  std::string log_path = work_dir.path() + "/log";

  // The shim redirects fds 1 and 2; flush the C++ streams first so their
  // buffered output doesn't land in Concorde's log.
  std::cout.flush();
  std::cerr.flush();

  int success = 0;
  int found_tour = 0;
  double optval = 0.0;
  std::vector<int> doubled_tour(doubled_n);
  int rval = vats5_concorde_solve(
      doubled_n,
      ecount,
      elist.data(),
      elen.data(),
      kForbiddenEdgeWeight,
      concorde_ub_double.has_value() ? &*concorde_ub_double : nullptr,
      seed,
      work_dir.path().c_str(),
      log_path.c_str(),
      &success,
      &found_tour,
      &optval,
      doubled_tour.data()
  );

  std::string concorde_output = ReadFile(log_path);
  if (tsp_log) {
    *tsp_log << concorde_output << std::flush;
  }

  if (rval != 0 || !success) {
    throw ConcordeFailure(
        "Concorde failed (rval=" + std::to_string(rval) + ", success=" +
        std::to_string(success) + "). Output:\n" + concorde_output
    );
  }

  // No tour: either Concorde proved the LP infeasible, or (with an upper
  // bound) no tour better than the bound exists.
  if (!found_tour) {
    return std::nullopt;
  }

  int raw_optimal_value = static_cast<int>(std::round(optval));

  // Concorde may report a tour whose value equals the bound; the bound is
  // strict, so treat that as no solution.
  if (concorde_ub.has_value() && raw_optimal_value >= *concorde_ub) {
    return std::nullopt;
  }

  // If the optimal tour uses a forbidden edge, then say that the problem is
  // infeasible.
  //
  // TODO: Think harder about this. The tour could "cheat" and pay
  // kForbiddenEdgeCost somewhere to get a much cheaper tour somewhere else,
  // and thus we'd incorrectly say that the problem is infeasible when
  // actually there is a feasible solution. Seems unlikely to matter in
  // practice, because kForbiddenEdgeCost is so big.
  if (DoubledTourUsesForbiddenEdge(doubled_tour, weights)) {
    return std::nullopt;
  }

  // Validate structure and extract original tour
  std::vector<StopId> tour = ValidateAndExtractTour(doubled_tour);

  // Subtract the offsets added during graph construction. A proper tour has
  // exactly n inter-vertex edges, each inflated by kInterVertexOffset and
  // by the negative-weight offset.
  int optimal_value = raw_optimal_value - n * kInterVertexOffset -
                      n * weights.NegativeWeightOffset();

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
    } catch (const EdgeWeightOverflow&) {
      // Don't retry - deterministic property of the input.
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
