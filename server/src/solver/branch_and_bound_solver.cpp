#include "solver/branch_and_bound_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <unistd.h>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "solver/data.h"
#include "solver/shortest_path.h"
#include "solver/step_merge.h"

namespace vats5 {

RemappedAdjacencyList RemapStopIds(const StepsAdjacencyList& adj) {
    // Collect all unique stop IDs (origins with edges + all destinations)
    std::unordered_set<int> used_stops_set;

    for (int origin = 0; origin < adj.NumStops(); ++origin) {
        StopId origin_stop{origin};
        auto groups = adj.GetGroups(origin_stop);
        if (!groups.empty()) {
            used_stops_set.insert(origin);
            for (const auto& group : groups) {
                used_stops_set.insert(group.destination_stop.v);
            }
        }
    }

    // Sort to preserve original stop id order.
    std::vector<int> used_stops(used_stops_set.begin(), used_stops_set.end());
    std::sort(used_stops.begin(), used_stops.end());

    // Build bidirectional mappings
    RemappedAdjacencyList result;
    result.mapping.new_to_original.reserve(used_stops.size());

    for (int old_id : used_stops) {
        StopId new_id{static_cast<int>(result.mapping.new_to_original.size())};
        result.mapping.original_to_new[old_id] = new_id;
        result.mapping.new_to_original.push_back(StopId{old_id});
    }

    int num_new_stops = static_cast<int>(result.mapping.new_to_original.size());

    // Build the remapped adjacency list
    // First pass: count groups per new stop
    std::vector<int> groups_per_stop(num_new_stops, 0);
    for (int origin = 0; origin < adj.NumStops(); ++origin) {
        auto groups = adj.GetGroups(StopId{origin});
        if (!groups.empty()) {
            int new_origin = result.mapping.original_to_new[origin].v;
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

        int new_origin = result.mapping.original_to_new[origin].v;
        int group_start = result.adj.group_offsets[new_origin];

        for (size_t i = 0; i < groups.size(); ++i) {
            const auto& old_group = groups[i];
            auto& new_group = result.adj.groups[group_start + i];

            new_group.destination_stop =
                result.mapping.original_to_new[old_group.destination_stop.v];
            new_group.flex_step = old_group.flex_step;
            new_group.steps_start = old_group.steps_start;
            new_group.steps_end = old_group.steps_end;
        }
    }

    return result;
}

// Output a RelaxedAdjacencyList as a Concorde TSP instance using UPPER_ROW format.
// Uses vertex doubling to convert asymmetric TSP to symmetric TSP:
// - For each original vertex i, create two vertices: 2i (in) and 2i+1 (out)
// - Edge (in_i, out_i) = 0 (forces them to be adjacent)
// - Edge (out_i, in_j) = w(i,j) for the asymmetric weight from i to j
// - Edges (in_i, in_j) and (out_i, out_j) are forbidden (large weight)
void OutputConcordeTsp(std::ostream& out, const RelaxedAdjacencyList& relaxed) {
    int n = relaxed.NumStops();
    int doubled_n = 2 * n;
    constexpr int kForbiddenEdgeWeight = 36 * 3600;

    // Build edge lookup: (origin * n + dest) -> weight
    std::unordered_map<int64_t, int> edge_weights;
    for (int origin = 0; origin < n; ++origin) {
        for (const auto& edge : relaxed.GetEdges(StopId{origin})) {
            int64_t key = static_cast<int64_t>(origin) * n + edge.destination_stop.v;
            edge_weights[key] = edge.weight_seconds;
        }
    }

    // Helper to get asymmetric weight, returning kForbiddenEdgeWeight if edge doesn't exist
    auto get_weight = [&](int from, int to) -> int {
        int64_t key = static_cast<int64_t>(from) * n + to;
        auto it = edge_weights.find(key);
        return (it != edge_weights.end()) ? it->second : kForbiddenEdgeWeight;
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
            return get_weight(a_orig, b_orig);
        } else if (a_is_in && !b_is_in) {
            // in(a_orig) <-> out(b_orig): asymmetric weight w(b_orig, a_orig)
            return get_weight(b_orig, a_orig);
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

// Parse Concorde solution from doubled graph back to original vertices.
// The doubled graph has vertices 2i (in) and 2i+1 (out) for each original vertex i.
// The tour alternates: in(a) -> out(a) -> in(b) -> out(b) -> ...
// We extract the original tour by taking the original vertex from each in/out pair.
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

    // The tour should alternate between in/out pairs of the same vertex.
    // Find the pattern: if doubled_tour[i] and doubled_tour[i+1] are a pair (same orig vertex),
    // then that's our in->out transition.
    // We need to extract original vertices from consecutive pairs.

    int n = doubled_n / 2;
    std::vector<StopId> tour;
    tour.reserve(n);

    // The tour should alternate between in/out pairs of the same vertex.
    // Forward direction: in(a) -> out(a) -> in(b) -> out(b) -> ...
    // Reversed direction: out(a) -> in(a) -> out(b) -> in(b) -> ...
    // Find where a proper pair starts and determine direction.

    int start_idx = -1;
    bool is_reversed = false;

    for (int i = 0; i < doubled_n; ++i) {
        int curr = doubled_tour[i];
        int next = doubled_tour[(i + 1) % doubled_n];
        int curr_orig = curr / 2;
        int next_orig = next / 2;
        bool curr_is_in = (curr % 2 == 0);
        bool curr_is_out = (curr % 2 == 1);
        bool next_is_in = (next % 2 == 0);
        bool next_is_out = (next % 2 == 1);

        if (curr_orig == next_orig) {
            if (curr_is_in && next_is_out) {
                // Forward: in(a) -> out(a)
                start_idx = i;
                is_reversed = false;
                break;
            } else if (curr_is_out && next_is_in) {
                // Reversed: out(a) -> in(a)
                start_idx = i;
                is_reversed = true;
                break;
            }
        }
    }

    if (start_idx == -1) {
        std::ostringstream err;
        err << "Could not find in/out pair in Concorde solution.\n";
        err << "Doubled tour (" << doubled_n << " vertices):\n";
        for (int i = 0; i < doubled_n; ++i) {
            int v = doubled_tour[i];
            int orig = v / 2;
            const char* type = (v % 2 == 0) ? "in" : "out";
            err << "  [" << i << "] " << v << " = " << type << "(" << orig << ")\n";
        }
        throw std::runtime_error(err.str());
    }

    // Extract original vertices, stepping by 2 (each pair is one original vertex)
    // If reversed, we need to reverse the final tour to get the correct direction
    for (int i = 0; i < n; ++i) {
        int idx = (start_idx + 2 * i) % doubled_n;
        int doubled_vertex = doubled_tour[idx];
        int orig_vertex = doubled_vertex / 2;
        tour.push_back(StopId{orig_vertex});
    }

    return tour;
}

ConcordeSolution SolveTspWithConcorde(const RelaxedAdjacencyList& relaxed, BoundaryIds boundary) {
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

    // Invoke Concorde (suppress stderr)
    std::ostringstream cmd;
    // cmd << "concorde -x -o " << solution_path << " " << problem_path << " 2>/dev/null";
    cmd << "concorde -x -o " << solution_path << " " << problem_path;

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

    // Parse solution
    std::vector<StopId> tour = ParseConcordeSolution(solution_path);

    // Cleanup temp directory
    std::remove(problem_path.c_str());
    std::remove(solution_path.c_str());
    rmdir(temp_dir.c_str());

    // Rotate tour so anchor is first.
    auto it_start = std::find(tour.begin(), tour.end(), boundary.anchor);
    if (it_start != tour.end()) {
        std::rotate(tour.begin(), it_start, tour.end());
        if (tour[1] == boundary.end) {
            std::reverse(tour.begin(), tour.end());
            std::rotate(tour.begin(), tour.end() - 1, tour.end());
        }
    }

    return ConcordeSolution{.tour = std::move(tour), .optimal_value = optimal_value};
}

ActualPathResult ComputeActualPath(
    const std::vector<StopId>& tour,
    const StepsAdjacencyList& adj,
    const BoundaryIds& boundary
) {
    constexpr int kNoPathDuration = 360 * 3600;  // 360 hours in seconds

    std::optional<std::vector<Step>> accumulated_steps;
    for (size_t i = 0; i < tour.size(); ++i) {
        StopId prev_stop = tour[i];
        StopId cur_stop = tour[(i + 1) % tour.size()];

        // TODO: Shouldn't need to redo dijkstra's here I think. Can build this up from stuff we've already computed.
        // Or maybe not.
        std::unordered_set<StopId> dest;
        dest.insert(cur_stop);
        const auto soln = FindMinimalPathSet(
            adj, prev_stop, dest,
            TimeSinceServiceStart{0}, TimeSinceServiceStart{36 * 3600},
            nullptr, boundary
        ).at(cur_stop);
        std::vector<Step> current_steps;
        current_steps.reserve(soln.size());
        for (const Path& path : soln) {
            current_steps.push_back(path.merged_step);
        }

        if (!accumulated_steps.has_value()) {
            accumulated_steps = std::move(current_steps);
        } else {
            accumulated_steps = MergeSteps(*accumulated_steps, current_steps);
        }

        // Merging a flex step with a very early origin time scheduled step can
        // produce negative origin time steps. We don't care about those, so
        // erase them.
        //
        // TODO: Consider if MergeSteps should not emit negative origin times.
        std::erase_if(*accumulated_steps, [](const Step& step) -> bool { return step.origin_time.seconds < 0; } );
    }

    // for (const Step& step : accumulated_steps) {
    //     std::cout << step.origin_time.ToString() << " -> " << step.destination_time.ToString() << "(" << TimeSinceServiceStart{step.destination_time.seconds - step.origin_time.seconds}.ToString() << ")";
    //     if (step.is_flex) {
    //         std::cout << " flex";
    //     }
    //     std::cout << "\n";
    // }
    // std::cout << "\n";

    int min_duration = kNoPathDuration;
    for (const Step& step : *accumulated_steps) {
        int duration = step.destination_time.seconds - step.origin_time.seconds;
        min_duration = std::min(min_duration, duration);
    }
    return ActualPathResult{.duration_seconds = min_duration};
}

struct SolutionContext {
    const DataGtfsMapping& gtfs_mapping;
};

struct SolutionState {
    StepsAdjacencyList adj;
    BoundaryIds boundary;
    StopIdRemapping remapping;
};

SolutionState MakeBranchForbidEdge(const SolutionState& cur, const StopId a, const StopId b) {
    assert(a != cur.boundary.anchor && b != cur.boundary.anchor);
    std::vector<Step> steps = GetAllSteps(cur.adj);
    std::erase_if(steps, [&](const Step& step) -> bool { return step.origin_stop == a && step.destination_stop == b; });
    return SolutionState{MakeAdjacencyList(steps), cur.boundary, cur.remapping};
};

BoundaryIds RemapBoundary(const StopIdRemapping& mapping, BoundaryIds b) {
    return BoundaryIds{
        mapping.original_to_new.at(b.start.v),
        mapping.original_to_new.at(b.end.v),
        mapping.original_to_new.at(b.anchor.v),
    };
}

SolutionState MakeBranchRequireEdge(const SolutionState& cur, const StopId a, const StopId b) {
    assert(a != cur.boundary.anchor && b != cur.boundary.anchor);

    BoundaryIds adjusted_boundary = cur.boundary;
    if (a == adjusted_boundary.start) {
        adjusted_boundary.start = b;
    }

    std::vector<Step> steps = GetAllSteps(cur.adj);

    std::unordered_map<StopId, std::vector<Step>> x_to_a;
    std::unordered_map<StopId, std::vector<Step>> a_to_x;

    std::vector<Step> a_to_b;
    std::vector<Step> b_to_a;

    std::unordered_map<StopId, std::vector<Step>> x_to_b;
    std::unordered_map<StopId, std::vector<Step>> b_to_x;

    std::vector<Step> result;

    // std::cout << "Collecting steps...\n";
    for (const Step& step : steps) {
        if (step.origin_stop != b && step.destination_stop == a) {
            x_to_a[step.origin_stop].push_back(step);
        } else if (step.origin_stop == a && step.destination_stop != b) {
            a_to_x[step.destination_stop].push_back(step);
        } else if (step.origin_stop == a && step.destination_stop == b) {
            a_to_b.push_back(step);
        } else if (step.origin_stop == b && step.destination_stop == a) {
            b_to_a.push_back(step);
        } else if (step.origin_stop != a && step.destination_stop == b) {
            x_to_b[step.origin_stop].push_back(step);
        } else if (step.origin_stop == b && step.destination_stop != a) {
            b_to_x[step.destination_stop].push_back(step);
        } else {
            result.push_back(step);
        }
    }

    // std::cout << "Merging X->A->B\n";
    for (const auto& [x, xa_steps] : x_to_a) {
        std::vector<Step> combined = MergeSteps(xa_steps, a_to_b);
        for (const auto& s : combined) {
            result.push_back(s);
        }
    }
    // std::cout << "Merging B->X\n";
    for (const auto& [x, bx_steps] : b_to_x) {
        for (const auto& s : bx_steps) {
            result.push_back(s);
        }
    }
    // std::cout << "X->A->Y\n";
    for (const auto& [x, xa_steps] : x_to_a) {
        for (const auto& [y, ay_steps]: a_to_x) {
            if (x == y) {
                continue;
            }
            std::vector<Step> combined = MergeSteps(xa_steps, ay_steps);
            for (const auto& s : combined) {
                result.push_back(s);
            }
        }
    }
    // std::cout << "X->B->Y\n";
    for (const auto& [x, xb_steps] : x_to_b) {
        for (const auto& [y, by_steps] : b_to_x) {
            if (x == y) {
                continue;
            }
            std::vector<Step> combined = MergeSteps(xb_steps, by_steps);
            for (const auto& s : combined) {
                result.push_back(s);
            }
        }
    }
    // std::cout << "X->B->A->Y\n";
    for (const auto& [x, xb_steps] : x_to_b) {
        for (const auto& [y, ay_steps] : a_to_x) {
            if (x == y) {
                continue;
            }
            std::vector<Step> combined = MergeSteps(xb_steps, b_to_a);
            combined = MergeSteps(combined, ay_steps);
            for (const auto& s : combined) {
                result.push_back(s);
            }
        }
    }
    // std::cout << "Done merge\n";

    int count_origin_a = 0;
    int count_dest_a = 0;
    for (const auto& s : result) {
        if (s.origin_stop == a) {
            count_origin_a += 1;
        }
        if (s.destination_stop == a) {
            count_dest_a += 1;
        }
    }
    // std::cout << "Origin A: " << count_origin_a << ", Destination A: " << count_dest_a << "\n";

    // Merging a flex step with a very early origin time scheduled step can
    // produce negative origin time steps. We don't care about those, so
    // erase them.
    //
    // TODO: Consider if MergeSteps should not emit negative origin times.
    std::erase_if(result, [](const Step& step) -> bool { return step.origin_time.seconds < 0; } );

    RemappedAdjacencyList remapped = RemapStopIds(MakeAdjacencyList(result));

    // Compose remappings: original -> cur's IDs -> new IDs
    StopIdRemapping composed_remapping;
    composed_remapping.new_to_original.resize(remapped.mapping.new_to_original.size());
    for (size_t new_id = 0; new_id < remapped.mapping.new_to_original.size(); ++new_id) {
        StopId intermediate_id = remapped.mapping.new_to_original[new_id];
        StopId original_id = cur.remapping.new_to_original[intermediate_id.v];
        composed_remapping.new_to_original[new_id] = original_id;
        composed_remapping.original_to_new[original_id.v] = StopId{static_cast<int>(new_id)};
    }

    return SolutionState{
        remapped.adj,
        RemapBoundary(remapped.mapping, adjusted_boundary),
        composed_remapping,
    };
};

struct SolutionStateEvaluation {
    std::vector<StopId> tour;
    std::vector<int> tour_forwards_weights;
    int lb;
    int ub;
};

SolutionStateEvaluation EvaluateState(const SolutionState& state) {
    RelaxedAdjacencyList relaxed = CompleteShortestRelaxedPaths(MakeRelaxedAdjacencyList(state.adj), state.boundary);

    ConcordeSolution solution;
    constexpr int kMaxRetries = 5;
    for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
        try {
            solution = SolveTspWithConcorde(relaxed, state.boundary);
            break;
        } catch (const std::exception& e) {
            std::cerr << "SolveTspWithConcorde attempt " << attempt << "/" << kMaxRetries << " failed: " << e.what() << "\n";
            if (attempt == kMaxRetries) {
                throw;
            }
        }
    }
    ActualPathResult result = ComputeActualPath(solution.tour, state.adj, state.boundary);
    std::cout << "lb: " << solution.optimal_value << ", ub: " << result.duration_seconds << "\n";

    std::vector<int> tour_forwards_weights;

    for (int i = 0; i < solution.tour.size(); ++i) {
        std::optional<int> fw = relaxed.GetWeight(solution.tour[i], solution.tour[(i + 1) % solution.tour.size()]);
        // TODO: What's up with no value?
        tour_forwards_weights.push_back(fw.has_value() ? *fw : 36 * 3600);
    }

    return SolutionStateEvaluation{
        .tour = solution.tour,
        .tour_forwards_weights = tour_forwards_weights,
        .lb = solution.optimal_value,
        .ub = result.duration_seconds,
    };
}

// struct SolutionContext {

// }

// struct SolutionState {
//     StepsAdjacencyList adj;
// };

void PrintTourEvaluation(
    const DataGtfsMapping& gtfs_mapping,
    const SolutionState& state,
    const SolutionStateEvaluation& eval
) {
    auto stop_name = [&](StopId id) -> std::string {
        if (id == state.boundary.start) {
            return "START";
        }
        if (id == state.boundary.end) {
            return "END";
        }
        if (id == state.boundary.anchor) {
            return "ANCHOR";
        }
        StopId original_id = state.remapping.new_to_original[id.v];
        return gtfs_mapping.stop_id_to_stop_name.at(original_id);
    };

    size_t max_prefix_len = 0;
    int max_fw_width = 0;
    for (size_t i = 0; i < eval.tour.size(); ++i) {
        StopId a = eval.tour[i];
        StopId b = eval.tour[(i + 1) % eval.tour.size()];
        size_t len = stop_name(a).size() + 4 + stop_name(b).size();  // 4 for " -> "
        max_prefix_len = std::max(max_prefix_len, len);
        max_fw_width = std::max(max_fw_width, static_cast<int>(std::to_string(eval.tour_forwards_weights[i]).size()));
    }

    int total = 0;
    for (size_t i = 0; i < eval.tour.size(); ++i) {
        StopId a = eval.tour[i];
        StopId b = eval.tour[(i + 1) % eval.tour.size()];
        std::string prefix = stop_name(a) + " -> " + stop_name(b);
        int fw = eval.tour_forwards_weights[i];
        total += fw;
        std::cout << std::left << std::setw(max_prefix_len) << prefix << ": "
            << std::right << std::setw(max_fw_width) << fw << "\n";
    }
    std::cout << "Total: " << total << "\n";
}

void DoIt(
    const DataGtfsMapping& mapping,
    const StepsAdjacencyList& adj,
    const std::unordered_set<StopId>& target_stops
) {
    std::cerr << "Reducing to minimal system paths..." << std::endl;
    PathsAdjacencyList minimal = ReduceToMinimalSystemPaths(adj, target_stops);

    // Add dummy START and END nodes:
    // - START has 0-duration flex edges TO all other nodes
    // - END has 0-duration flex edges FROM all other nodes
    int max_stop_id = -1;
    for (const auto& [origin, _] : minimal.adjacent) {
        max_stop_id = std::max(max_stop_id, origin.v);
    }
    BoundaryIds boundary{
        max_stop_id + 1000,
        max_stop_id + 1000 + 1,
        max_stop_id + 1000 + 2,
    };

    // Create a flex step with duration 0.
    auto make_zero_flex_step = [](StopId from, StopId to) -> Step {
        return Step{
            .origin_stop = from,
            .destination_stop = to,
            .origin_time = TimeSinceServiceStart{0},
            .destination_time = TimeSinceServiceStart{0},
            .origin_trip = TripId::BOUNDARY_TRIP,
            .destination_trip = TripId::BOUNDARY_TRIP,
            .is_flex = true,
        };
    };

    // Create a path containing just one flex step.
    auto make_zero_flex_path = [&](StopId from, StopId to) -> Path {
        Step step = make_zero_flex_step(from, to);
        return Path{.merged_step = step, .steps = {step}};
    };

    // Collect all existing stops.
    std::vector<StopId> existing_stops;
    for (const auto& [origin, _] : minimal.adjacent) {
        existing_stops.push_back(origin);
    }

    // start -> *
    for (StopId stop : existing_stops) {
        minimal.adjacent[boundary.start].push_back({make_zero_flex_path(boundary.start, stop)});
    }

    // * -> end
    for (StopId stop : existing_stops) {
        minimal.adjacent[stop].push_back({make_zero_flex_path(stop, boundary.end)});
    }

    // Anchor.
    minimal.adjacent[boundary.end].push_back({make_zero_flex_path(boundary.end, boundary.anchor)});
    minimal.adjacent[boundary.anchor].push_back({make_zero_flex_path(boundary.anchor, boundary.start)});

   std::cerr << "Converting to steps list..." << std::endl;
    StepsAdjacencyList minimal_steps = AdjacentPathsToStepsList(minimal);

    RemappedAdjacencyList remapped = RemapStopIds(minimal_steps);
    BoundaryIds remapped_boundary = RemapBoundary(remapped.mapping, boundary);

    // Helper to get stop name from remapped id, with special case for dummy nodes.
    auto stop_name = [&](const SolutionState& state, StopId remapped_id) -> std::string {
        if (remapped_id == state.boundary.start) {
            return "START";
        }
        if (remapped_id == state.boundary.end) {
            return "END";
        }
        if (remapped_id == state.boundary.anchor) {
            return "ANCHOR";
        }
        StopId original_id = state.remapping.new_to_original[remapped_id.v];
        return mapping.stop_id_to_stop_name.at(original_id);
    };

    SolutionContext solution_context{mapping};
    SolutionState solution_initial_state{
        remapped.adj,
        remapped_boundary,
        remapped.mapping,
    };

    // Branch and bound search
    struct BnBNode {
        SolutionState state;
        int lb;  // lower bound
        int left_branches;   // number of left (forbid) branches taken to reach this node
        int right_branches;  // number of right (require) branches taken to reach this node
    };

    auto cmp = [](const BnBNode& a, const BnBNode& b) {
        return a.lb > b.lb;  // min-heap by lower bound
    };
    std::priority_queue<BnBNode, std::vector<BnBNode>, decltype(cmp)> pq(cmp);

    // Evaluate initial state
    std::cout << "Evaluating initial state...\n";
    SolutionStateEvaluation initial_eval = EvaluateState(solution_initial_state);
    // PrintTourEvaluation(mapping, solution_initial_state, initial_eval);

    int best_ub = initial_eval.ub;
    SolutionState best_state = solution_initial_state;
    SolutionStateEvaluation best_eval = initial_eval;

    pq.push(BnBNode{solution_initial_state, initial_eval.lb, 0, 0});

    int nodes_explored = 0;
    while (!pq.empty()) {
        BnBNode node = std::move(const_cast<BnBNode&>(pq.top()));
        pq.pop();

        // Prune if lower bound exceeds best upper bound
        if (node.lb >= best_ub) {
            std::cout << "Pruning node with lb=" << node.lb << " (best_ub=" << best_ub << ")\n";
            continue;
        }

        nodes_explored++;
        std::cout << "\n=== Node " << nodes_explored << " (lb=" << node.lb << ", best_ub=" << best_ub << ", queue=" << pq.size()
                  << ", L=" << node.left_branches << ", R=" << node.right_branches << ") ===\n";

        SolutionStateEvaluation eval = EvaluateState(node.state);
        // PrintTourEvaluation(mapping, node.state, eval);

        // Update best solution if this node has a better upper bound
        if (eval.ub < best_ub) {
            best_ub = eval.ub;
            best_state = node.state;
            best_eval = eval;
            std::cout << "*** New best: ub=" << best_ub << " ***\n";
        }

        // If lb == ub, this is optimal for this subtree, no need to branch
        if (eval.lb >= best_ub) {
            std::cout << "Node is pruned (lb=" << eval.lb << " >= best_ub=" << best_ub << ")\n";
            continue;
        }

        // Find an edge to branch on.
        // Collect all valid edges and randomly choose one.
        std::vector<std::pair<StopId, StopId>> valid_edges;
        RelaxedAdjacencyList relaxed_for_edges = MakeRelaxedAdjacencyList(node.state.adj);
        for (size_t i = 0; i < eval.tour.size(); ++i) {
            StopId candidate_a = eval.tour[i];
            StopId candidate_b = eval.tour[(i + 1) % eval.tour.size()];
            if (
                candidate_a != node.state.boundary.anchor &&
                candidate_b != node.state.boundary.anchor &&
                relaxed_for_edges.GetWeight(candidate_a, candidate_b).has_value()
            ) {
                valid_edges.push_back({candidate_a, candidate_b});
            }
        }

        StopId a{-1}, b{-1};
        if (!valid_edges.empty()) {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::uniform_int_distribution<size_t> dist(0, valid_edges.size() - 1);
            auto [chosen_a, chosen_b] = valid_edges[dist(gen)];
            a = chosen_a;
            b = chosen_b;
        }

        if (a.v == -1) {
            std::cout << "Didn't find edge to branch on\n";
            continue;
        }


        std::cout << "Branching on edge: " << stop_name(node.state, a) << " -> " << stop_name(node.state, b) << "\n";

        // Left branch: forbid the edge
        std::cout << "Creating left branch (forbid)...\n";
        SolutionState left_state = MakeBranchForbidEdge(node.state, a, b);
        SolutionStateEvaluation left_eval = EvaluateState(left_state);
        if (left_eval.lb < eval.lb) {
            std::cout << "\n!!! ERROR: Left branch decreased LB from " << eval.lb << " to " << left_eval.lb << " !!!\n";
            std::cout << "BEFORE (parent):\n";
            PrintTourEvaluation(mapping, node.state, eval);
            std::cout << "\nAFTER (left child, forbid " << stop_name(node.state, a) << " -> " << stop_name(node.state, b) << "):\n";
            PrintTourEvaluation(mapping, left_state, left_eval);
            return;
        }
        if (left_eval.lb < best_ub) {
            pq.push(BnBNode{std::move(left_state), left_eval.lb, node.left_branches + 1, node.right_branches});
        } else {
            std::cout << "Left branch pruned immediately (lb=" << left_eval.lb << ")\n";
        }

        // Right branch: require the edge
        std::cout << "Creating right branch (require)...\n";
        SolutionState right_state = MakeBranchRequireEdge(node.state, a, b);
        SolutionStateEvaluation right_eval = EvaluateState(right_state);
        if (right_eval.lb < eval.lb) {
            std::cout << "\n!!! ERROR: Right branch decreased LB from " << eval.lb << " to " << right_eval.lb << " !!!\n";
            std::cout << "BEFORE (parent):\n";
            PrintTourEvaluation(mapping, node.state, eval);
            std::cout << "\nAFTER (right child, require " << stop_name(node.state, a) << " -> " << stop_name(node.state, b) << "):\n";
            PrintTourEvaluation(mapping, right_state, right_eval);
            return;
        }
        if (right_eval.lb < best_ub) {
            pq.push(BnBNode{std::move(right_state), right_eval.lb, node.left_branches, node.right_branches + 1});
        } else {
            std::cout << "Right branch pruned immediately (lb=" << right_eval.lb << ")\n";
        }
    }

    std::cout << "\n=== Branch and Bound Complete ===\n";
    std::cout << "Nodes explored: " << nodes_explored << "\n";
    std::cout << "Best upper bound: " << best_ub << "\n";
    PrintTourEvaluation(mapping, best_state, best_eval);
}

}  // namespace vats5
