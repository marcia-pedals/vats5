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

constexpr int kForbiddenEdgeWeight = 32767;
constexpr int kInterVertexOffset = 10000;

// Exception thrown when Concorde returns a tour that doesn't have proper in/out pairing.
// This indicates insufficient kInterVertexOffset or a bug, not a transient error.
class InvalidTourStructure : public std::exception {
public:
    explicit InvalidTourStructure(std::string message) : message_(std::move(message)) {}
    const char* what() const noexcept override { return message_.c_str(); }
private:
    std::string message_;
};

// Output a RelaxedAdjacencyList as a Concorde TSP instance using UPPER_ROW format.
// Uses vertex doubling to convert asymmetric TSP to symmetric TSP.
void OutputConcordeTsp(std::ostream& out, const RelaxedAdjacencyList& relaxed) {
    int n = relaxed.NumStops();
    int doubled_n = 2 * n;

    // Build edge lookup: edge_weights[origin * n + dest] = weight
    std::vector<int> edge_weights(n * n, kForbiddenEdgeWeight);
    for (int origin = 0; origin < n; ++origin) {
        for (const auto& edge : relaxed.GetEdges(StopId{origin})) {
            edge_weights[origin * n + edge.destination_stop.v] = edge.weight_seconds;
        }
    }

    // Helper to get asymmetric weight
    auto get_weight = [&](int from, int to) -> int {
        return edge_weights[from * n + to];
    };

    // Helper to compute symmetric edge weight in doubled graph
    // Vertices: 2i = in(i), 2i+1 = out(i)
    auto get_doubled_weight = [&](int a, int b) -> int {
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
            return get_weight(a_orig, b_orig) + kInterVertexOffset;
        } else if (a_is_in && !b_is_in) {
            // in(a_orig) <-> out(b_orig): asymmetric weight w(b_orig, a_orig)
            return get_weight(b_orig, a_orig) + kInterVertexOffset;
        } else {
            // Both in or both out: forbidden
            return kForbiddenEdgeWeight;
        }
    };

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
            int weight = get_doubled_weight(i, j);
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

std::vector<StopId> ParseConcordeSolution(const std::string& solution_path) {
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

ConcordeSolution SolveTspWithConcordeImpl(const RelaxedAdjacencyList& relaxed, std::ostream* tsp_log) {
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
        OutputConcordeTsp(out, relaxed);
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

    // Parse solution
    std::vector<StopId> tour = ParseConcordeSolution(solution_path);

    // Subtract the inter-vertex offset that was added during graph construction.
    // The proper tour has exactly n inter-vertex edges, so we subtract n * offset.
    int n = static_cast<int>(tour.size());
    int optimal_value = raw_optimal_value - n * kInterVertexOffset;

    // Cleanup temp directory
    std::remove(problem_path.c_str());
    std::remove(solution_path.c_str());
    rmdir(temp_dir.c_str());

    return ConcordeSolution{.tour = std::move(tour), .optimal_value = optimal_value};
}

}  // namespace

ConcordeSolution SolveTspWithConcorde(const RelaxedAdjacencyList& relaxed, std::ostream* tsp_log) {
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
