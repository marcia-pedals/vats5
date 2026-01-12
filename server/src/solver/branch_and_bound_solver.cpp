#include "solver/branch_and_bound_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>

#include "solver/data.h"
#include "solver/shortest_path.h"
#include "solver/step_merge.h"

namespace vats5 {

RemappedAdjacencyList RemapStopIds(const StepsAdjacencyList& adj) {
    // Collect all unique stop IDs (origins with edges + all destinations)
    std::unordered_set<int> used_stops;

    for (int origin = 0; origin < adj.NumStops(); ++origin) {
        StopId origin_stop{origin};
        auto groups = adj.GetGroups(origin_stop);
        if (!groups.empty()) {
            used_stops.insert(origin);
            for (const auto& group : groups) {
                used_stops.insert(group.destination_stop.v);
            }
        }
    }

    // Build bidirectional mappings
    RemappedAdjacencyList result;
    result.new_to_original.reserve(used_stops.size());

    for (int old_id : used_stops) {
        StopId new_id{static_cast<int>(result.new_to_original.size())};
        result.original_to_new[old_id] = new_id;
        result.new_to_original.push_back(StopId{old_id});
    }

    int num_new_stops = static_cast<int>(result.new_to_original.size());

    // Build the remapped adjacency list
    // First pass: count groups per new stop
    std::vector<int> groups_per_stop(num_new_stops, 0);
    for (int origin = 0; origin < adj.NumStops(); ++origin) {
        auto groups = adj.GetGroups(StopId{origin});
        if (!groups.empty()) {
            int new_origin = result.original_to_new[origin].v;
            groups_per_stop[new_origin] = static_cast<int>(groups.size());
        }
    }

    // Build group_offsets
    result.adj.group_offsets.resize(num_new_stops);
    int offset = 0;
    for (int i = 0; i < num_new_stops; ++i) {
        result.adj.group_offsets[i] = offset;
        offset += groups_per_stop[i];
    }

    // Allocate groups
    result.adj.groups.resize(offset);

    // Copy steps and departure_times (they don't contain stop IDs)
    result.adj.steps = adj.steps;
    result.adj.departure_times_div10 = adj.departure_times_div10;

    // Second pass: fill in groups with remapped destination stop IDs
    for (int origin = 0; origin < adj.NumStops(); ++origin) {
        auto groups = adj.GetGroups(StopId{origin});
        if (groups.empty()) continue;

        int new_origin = result.original_to_new[origin].v;
        int group_start = result.adj.group_offsets[new_origin];

        for (size_t i = 0; i < groups.size(); ++i) {
            const auto& old_group = groups[i];
            auto& new_group = result.adj.groups[group_start + i];

            new_group.destination_stop =
                result.original_to_new[old_group.destination_stop.v];
            new_group.flex_step = old_group.flex_step;
            new_group.steps_start = old_group.steps_start;
            new_group.steps_end = old_group.steps_end;
        }
    }

    return result;
}

// Output a RelaxedAdjacencyList as a Concorde TSP instance using UPPER_ROW format.
// The graph is symmetrized by taking min(weight(i,j), weight(j,i)).
// Missing edges get a large weight (INT_MAX / 2 to avoid overflow).
// A dummy START node (index n) is added with 0-weight edges to all other nodes.
void OutputConcordeTsp(std::ostream& out, const RelaxedAdjacencyList& relaxed) {
    int n = relaxed.NumStops();
    int n_with_dummy = n + 1;  // Node n is the dummy START node
    constexpr int kMissingEdgeWeight = std::numeric_limits<int>::max() / 2;

    // Build edge lookup: (origin * n + dest) -> weight
    std::unordered_map<int64_t, int> edge_weights;
    for (int origin = 0; origin < n; ++origin) {
        for (const auto& edge : relaxed.GetEdges(StopId{origin})) {
            int64_t key = static_cast<int64_t>(origin) * n + edge.destination_stop.v;
            edge_weights[key] = edge.weight_seconds;
        }
    }

    // Helper to get weight, returning kMissingEdgeWeight if edge doesn't exist
    // Edges to/from dummy node n have weight 0
    auto get_weight = [&](int from, int to) -> int {
        if (from == n || to == n) {
            return 0;  // Dummy START node has 0-weight edges to all
        }
        int64_t key = static_cast<int64_t>(from) * n + to;
        auto it = edge_weights.find(key);
        return (it != edge_weights.end()) ? it->second : kMissingEdgeWeight;
    };

    // Output TSP header
    out << "NAME: vats5\n";
    out << "TYPE: TSP\n";
    out << "DIMENSION: " << n_with_dummy << "\n";
    out << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
    out << "EDGE_WEIGHT_FORMAT: UPPER_ROW\n";
    out << "EDGE_WEIGHT_SECTION\n";

    // Output upper triangular matrix (excluding diagonal)
    // For row i, output weights to j for all j > i
    for (int i = 0; i < n_with_dummy; ++i) {
        for (int j = i + 1; j < n_with_dummy; ++j) {
            int weight_ij = get_weight(i, j);
            int weight_ji = get_weight(j, i);
            int symmetric_weight = std::min(weight_ij, weight_ji);
            out << symmetric_weight;
            if (j < n_with_dummy - 1) {
                out << " ";
            }
        }
        if (i < n_with_dummy - 1) {
            out << "\n";
        }
    }
    out << "\nEOF\n";
}

std::vector<int> ParseConcordeSolution(const std::string& solution_path) {
    std::ifstream in(solution_path);
    if (!in) {
        throw std::runtime_error("Failed to open solution file: " + solution_path);
    }

    int n;
    in >> n;

    std::vector<int> tour;
    tour.reserve(n);
    for (int i = 0; i < n; ++i) {
        int node;
        in >> node;
        tour.push_back(node);
    }
    return tour;
}

ConcordeSolution SolveTspWithConcorde(const RelaxedAdjacencyList& relaxed) {
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

    // Invoke Concorde
    std::ostringstream cmd;
    cmd << "concorde -x -o " << solution_path << " " << problem_path;
    std::cerr << "Running: " << cmd.str() << std::endl;

    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) {
        throw std::runtime_error("Failed to run concorde");
    }

    std::string concorde_output;
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        concorde_output += buffer;
    }
    pclose(pipe);

    size_t pos = concorde_output.find("Optimal Solution:");
    if (pos == std::string::npos) {
        throw std::runtime_error("Concorde did not find optimal solution. Output:\n" + concorde_output);
    }
    int optimal_value = static_cast<int>(std::round(std::stod(concorde_output.substr(pos + 17))));
    std::cerr << "Optimal Solution: " << optimal_value << std::endl;

    // Parse solution
    std::vector<int> tour = ParseConcordeSolution(solution_path);

    // Cleanup temp directory
    std::remove(problem_path.c_str());
    std::remove(solution_path.c_str());
    rmdir(temp_dir.c_str());

    // The dummy START node is the last node (index = number of real stops)
    int dummy_node = relaxed.NumStops();

    // Find the dummy node in the tour and rotate to put it at the end
    auto dummy_it = std::find(tour.begin(), tour.end(), dummy_node);
    if (dummy_it == tour.end()) {
        throw std::runtime_error("Dummy START node not found in tour");
    }
    std::rotate(tour.begin(), dummy_it + 1, tour.end());

    // Remove the dummy node from the tour
    tour.pop_back();

    return ConcordeSolution{.tour = std::move(tour), .optimal_value = optimal_value};
}

ActualPathResult ComputeActualPath(
    const std::vector<int>& tour,
    const RemappedAdjacencyList& remapped,
    const StepsAdjacencyList& minimal_steps,
    const DataGtfsMapping& mapping
) {
    constexpr int kNoPathDuration = 360 * 3600;  // 360 hours in seconds

    std::cout << "Tour (" << tour.size() << " stops):" << std::endl;
    std::vector<Step> accumulated_steps;
    for (size_t i = 0; i < tour.size(); ++i) {
        int remapped_id = tour[i];

        // Print stop name.
        StopId original_id = remapped.new_to_original[remapped_id];
        auto it = mapping.stop_id_to_stop_name.find(original_id);
        if (it != mapping.stop_id_to_stop_name.end()) {
            std::cout << it->second << std::endl;
        } else {
            std::cout << "Unknown stop " << original_id.v << std::endl;
        }

        // Accumulate steps.
        if (i == 0) {
            continue;
        }
        int prev_remapped_id = tour[i - 1];
        StopId prev_original_id = remapped.new_to_original[prev_remapped_id];

        // TODO: Shouldn't need to redo dijkstra's here I think. Can build this up from stuff we've already computed.
        std::unordered_set<StopId> dest;
        dest.insert(original_id);
        const auto soln = FindMinimalPathSet(minimal_steps, prev_original_id, dest).at(original_id);
        std::vector<Step> current_steps;
        current_steps.reserve(soln.size());
        for (const Path& path : soln) {
            current_steps.push_back(path.merged_step);
        }

        if (i == 1) {
            accumulated_steps = std::move(current_steps);
        } else {
            accumulated_steps = MergeSteps(accumulated_steps, current_steps);
        }

        // Merging a flex step with a very early origin time scheduled step can
        // produce negative origin time steps. We don't care about those, so
        // erase them.
        //
        // TODO: Consider if MergeSteps should not emit negative origin times.
        std::erase_if(accumulated_steps, [](const Step& step) -> bool { return step.origin_time.seconds < 0; } );
    }

    for (const Step& step : accumulated_steps) {
        std::cout << step.origin_time.ToString() << " -> " << step.destination_time.ToString() << "(" << TimeSinceServiceStart{step.destination_time.seconds - step.origin_time.seconds}.ToString() << ")";
        if (step.is_flex) {
            std::cout << " flex";
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    int min_duration = kNoPathDuration;
    for (const Step& step : accumulated_steps) {
        int duration = step.destination_time.seconds - step.origin_time.seconds;
        min_duration = std::min(min_duration, duration);
    }
    return ActualPathResult{.duration_seconds = min_duration};
}

void DoIt(
    const DataGtfsMapping& mapping,
    const StepsAdjacencyList& adj,
    const std::unordered_set<StopId>& target_stops
) {
    std::cerr << "Reducing to minimal system paths..." << std::endl;
    PathsAdjacencyList minimal = ReduceToMinimalSystemPaths(adj, target_stops);

    std::cerr << "Converting to steps list..." << std::endl;
    StepsAdjacencyList minimal_steps = AdjacentPathsToStepsList(minimal);

    std::cerr << "Remapping stop IDs..." << std::endl;
    RemappedAdjacencyList remapped = RemapStopIds(minimal_steps);
    RelaxedAdjacencyList relaxed = CompleteShortestRelaxedPaths(MakeRelaxedAdjacencyList(remapped.adj));

    ConcordeSolution solution = SolveTspWithConcorde(relaxed);

    ActualPathResult result = ComputeActualPath(solution.tour, remapped, minimal_steps, mapping);
    std::cerr << "Minimum duration: " << result.duration_seconds << " seconds" << std::endl;
}

}  // namespace vats5
