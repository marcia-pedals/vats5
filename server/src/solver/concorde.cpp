#include "solver/concorde.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace vats5 {
namespace {

constexpr int kForbiddenEdgeWeight = 16000;
constexpr int kInterVertexOffset = 4000;

// Exception thrown when Concorde returns a tour that doesn't have proper in/out pairing.
// This indicates insufficient kInterVertexOffset or a bug, not a transient error.
class InvalidTourStructure : public std::exception {
public:
    explicit InvalidTourStructure(std::string message) : message_(std::move(message)) {}
    const char* what() const noexcept override { return message_.c_str(); }
private:
    std::string message_;
};

// Helper class for computing edge weights in the doubled graph.
// Uses vertex doubling to convert asymmetric TSP to symmetric TSP.
// Vertices: 2i = in(i), 2i+1 = out(i)
class DoubledGraphWeights {
public:
    explicit DoubledGraphWeights(const RelaxedAdjacencyList& relaxed)
        : n_(relaxed.NumStops()), edge_weights_(n_ * n_, kForbiddenEdgeWeight) {
        for (int origin = 0; origin < n_; ++origin) {
            for (const auto& edge : relaxed.GetEdges(StopId{origin})) {
                edge_weights_[origin * n_ + edge.destination_stop.v] = edge.weight_seconds;
            }
        }
    }

    int NumStops() const { return n_; }
    int DoubledN() const { return 2 * n_; }

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
            return GetAsymmetricWeight(a_orig, b_orig) + kInterVertexOffset;
        } else if (a_is_in && !b_is_in) {
            // in(a_orig) <-> out(b_orig): asymmetric weight w(b_orig, a_orig)
            return GetAsymmetricWeight(b_orig, a_orig) + kInterVertexOffset;
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
    std::vector<int> edge_weights_;
};

// Output a RelaxedAdjacencyList as a Concorde TSP instance using UPPER_ROW format.
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
bool DoubledTourUsesForbiddenEdge(const std::vector<int>& doubled_tour, const DoubledGraphWeights& weights) {
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
std::vector<StopId> ValidateAndExtractTour(const std::vector<int>& doubled_tour) {
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

    // Find start of a same-vertex pair. Tour alternates: in/out pairs of same vertex.
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
            throw InvalidTourStructure("Mismatched pair at position " + std::to_string(i) +
                                       ": " + std::to_string(orig(v1)) + " vs " +
                                       std::to_string(orig(v2)) + ". " + format_tour());
        }
        if (is_in(v1) == is_in(v2)) {
            throw InvalidTourStructure("Invalid in/out pattern at position " +
                                       std::to_string(i) + ". " + format_tour());
        }
        tour.push_back(StopId{orig(v1)});
    }

    if (is_reversed) {
        std::reverse(tour.begin(), tour.end());
    }
    return tour;
}

std::optional<ConcordeSolution> SolveTspWithConcordeImpl(const RelaxedAdjacencyList& relaxed, std::ostream* tsp_log) {
    DoubledGraphWeights weights(relaxed);

    // Create temp directory
    std::string temp_dir = "/tmp/vats5_tsp_XXXXXX";
    if (mkdtemp(temp_dir.data()) == nullptr) {
        throw std::runtime_error("Failed to create temp directory");
    }
    std::string problem_path = temp_dir + "/problem";
    std::string solution_path = temp_dir + "/solution";

    // Write TSP problem to temp file
    {
        std::ofstream out(problem_path);
        OutputConcordeTsp(out, weights);
    }

    // Invoke Concorde from temp dir so its temp files don't conflict when running in parallel
    std::ostringstream cmd;
    cmd << "cd " << temp_dir << " && concorde -x -o solution problem 2>&1";

    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) {
        throw std::runtime_error("Failed to run concorde");
    }

    std::string concorde_output;
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        if (tsp_log) {
            *tsp_log << buffer << std::flush;
        }
        concorde_output += buffer;
    }
    pclose(pipe);

    size_t pos = concorde_output.find("Optimal Solution:");
    if (pos == std::string::npos) {
        throw std::runtime_error("Concorde did not find optimal solution. Output:\n" + concorde_output);
    }
    int raw_optimal_value = static_cast<int>(std::round(std::stod(concorde_output.substr(pos + 17))));

    // Read the doubled tour
    std::vector<int> doubled_tour = ReadDoubledTour(solution_path);

    // Check if tour uses any forbidden edges before structural validation
    if (DoubledTourUsesForbiddenEdge(doubled_tour, weights)) {
        // Cleanup temp directory before returning
        std::remove(problem_path.c_str());
        std::remove(solution_path.c_str());
        rmdir(temp_dir.c_str());
        return std::nullopt;
    }

    // Validate structure and extract original tour
    std::vector<StopId> tour = ValidateAndExtractTour(doubled_tour);
    int n = static_cast<int>(tour.size());

    // Subtract the inter-vertex offset that was added during graph construction.
    // The proper tour has exactly n inter-vertex edges, so we subtract n * offset.
    int optimal_value = raw_optimal_value - n * kInterVertexOffset;

    // Cleanup temp directory
    std::remove(problem_path.c_str());
    std::remove(solution_path.c_str());
    rmdir(temp_dir.c_str());

    return ConcordeSolution{.tour = std::move(tour), .optimal_value = optimal_value};
}

}  // namespace

std::optional<ConcordeSolution> SolveTspWithConcorde(const RelaxedAdjacencyList& relaxed, std::ostream* tsp_log) {
    constexpr int kMaxRetries = 5;
    for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
        try {
            return SolveTspWithConcordeImpl(relaxed, tsp_log);
        } catch (const InvalidTourStructure&) {
            // Don't retry - indicates insufficient kInterVertexOffset or a bug, not transient.
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
