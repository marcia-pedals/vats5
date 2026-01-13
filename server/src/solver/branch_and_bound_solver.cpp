#include "solver/branch_and_bound_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
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
// The graph is symmetrized by taking min(weight(i,j), weight(j,i)).
// Missing edges get a large weight
void OutputConcordeTsp(std::ostream& out, const RelaxedAdjacencyList& relaxed) {
    int n = relaxed.NumStops();
    constexpr int kMissingEdgeWeight = 360 * 3600;

    // Build edge lookup: (origin * n + dest) -> weight
    std::unordered_map<int64_t, int> edge_weights;
    for (int origin = 0; origin < n; ++origin) {
        for (const auto& edge : relaxed.GetEdges(StopId{origin})) {
            int64_t key = static_cast<int64_t>(origin) * n + edge.destination_stop.v;
            edge_weights[key] = edge.weight_seconds;
        }
    }

    // Helper to get weight, returning kMissingEdgeWeight if edge doesn't exist
    auto get_weight = [&](int from, int to) -> int {
        int64_t key = static_cast<int64_t>(from) * n + to;
        auto it = edge_weights.find(key);
        return (it != edge_weights.end()) ? it->second : kMissingEdgeWeight;
    };

    // Output TSP header
    out << "NAME: vats5\n";
    out << "TYPE: TSP\n";
    out << "DIMENSION: " << n << "\n";
    out << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
    out << "EDGE_WEIGHT_FORMAT: UPPER_ROW\n";
    out << "EDGE_WEIGHT_SECTION\n";

    // Output upper triangular matrix (excluding diagonal)
    // For row i, output weights to j for all j > i
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            int weight_ij = get_weight(i, j);
            int weight_ji = get_weight(j, i);
            int symmetric_weight = std::min(weight_ij, weight_ji);
            out << symmetric_weight;
            if (j < n - 1) {
                out << " ";
            }
        }
        if (i < n - 1) {
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

    int n;
    in >> n;

    std::vector<StopId> tour;
    tour.reserve(n);
    for (int i = 0; i < n; ++i) {
        int node;
        in >> node;
        tour.push_back(StopId{node});
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

    // Invoke Concorde
    std::ostringstream cmd;
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

    // Rotate tour so dummy_start is first.
    auto it_start = std::find(tour.begin(), tour.end(), boundary.start);
    if (it_start != tour.end()) {
        std::rotate(tour.begin(), it_start, tour.end());

        // If the anchor comes after the start, then the tour is "backwards" so
        // reverse it and then rotate the start back to the front.
        if (tour[1] == boundary.anchor) {
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
        StopId cur_stop = tour[i];

        if (i == 0) {
            continue;
        }
        StopId prev_stop = tour[i - 1];

        if (prev_stop == boundary.start || cur_stop == boundary.anchor) {
            continue;
        }

        // TODO: Shouldn't need to redo dijkstra's here I think. Can build this up from stuff we've already computed.
        // Or maybe not.
        std::unordered_set<StopId> dest;
        dest.insert(cur_stop);
        const auto soln = FindMinimalPathSet(adj, prev_stop, dest).at(cur_stop);
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
    std::vector<int> tour_backwards_weights;
    int lb;
    int ub;
};

SolutionStateEvaluation EvaluateState(const SolutionState& state) {
    RelaxedAdjacencyList relaxed = CompleteShortestRelaxedPaths(MakeRelaxedAdjacencyList(state.adj), state.boundary);
    ConcordeSolution solution = SolveTspWithConcorde(relaxed, state.boundary);
    ActualPathResult result = ComputeActualPath(solution.tour, state.adj, state.boundary);
    std::cout << "lb: " << solution.optimal_value << ", ub: " << result.duration_seconds << "\n";

    std::vector<int> tour_forwards_weights;
    std::vector<int> tour_backwards_weights;

    for (int i = 0; i < solution.tour.size() - 1; ++i) {
        std::optional<int> fw = relaxed.GetWeight(solution.tour[i], solution.tour[i + 1]);
        std::optional<int> bw = relaxed.GetWeight(solution.tour[i + 1], solution.tour[i]);
        assert(fw.has_value());
        assert(bw.has_value());
        tour_forwards_weights.push_back(*fw);
        tour_backwards_weights.push_back(*bw);
    }

    return SolutionStateEvaluation{
        .tour = solution.tour,
        .tour_forwards_weights = tour_forwards_weights,
        .tour_backwards_weights = tour_backwards_weights,
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
    int max_bw_width = 0;
    for (size_t i = 0; i < eval.tour.size() - 1; ++i) {
        StopId a = eval.tour[i];
        StopId b = eval.tour[i + 1];
        size_t len = stop_name(a).size() + 4 + stop_name(b).size();  // 4 for " -> "
        max_prefix_len = std::max(max_prefix_len, len);
        max_fw_width = std::max(max_fw_width, static_cast<int>(std::to_string(eval.tour_forwards_weights[i]).size()));
        max_bw_width = std::max(max_bw_width, static_cast<int>(std::to_string(eval.tour_backwards_weights[i]).size()));
    }
    int max_weight_width = std::max(max_fw_width, max_bw_width);

    int sum_of_mins = 0;
    for (size_t i = 0; i < eval.tour.size() - 1; ++i) {
        StopId a = eval.tour[i];
        StopId b = eval.tour[i + 1];
        std::string prefix = stop_name(a) + " -> " + stop_name(b);
        int fw = eval.tour_forwards_weights[i];
        int bw = eval.tour_backwards_weights[i];
        int min_weight = std::min(fw, bw);
        sum_of_mins += min_weight;
        std::cout << std::left << std::setw(max_prefix_len) << prefix << ": "
            << std::right << std::setw(max_weight_width) << fw << " | "
            << std::setw(max_weight_width) << bw << " = "
            << min_weight << "\n";
    }
    std::cout << "Sum of mins: " << sum_of_mins << "\n";
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

    // anchor -> start and end -> anchor are added later to avoid being included
    // in path completion.
    // Hacky self-steps to keep these through all the graph transformations that
    // sometimes drop vertices without edges.
    minimal.adjacent[boundary.anchor].push_back({make_zero_flex_path(boundary.anchor, boundary.anchor)});

    // start -> *
    for (StopId stop : existing_stops) {
        minimal.adjacent[boundary.start].push_back({make_zero_flex_path(boundary.start, stop)});
    }

    // * -> end
    for (StopId stop : existing_stops) {
        minimal.adjacent[stop].push_back({make_zero_flex_path(stop, boundary.end)});
    }

   std::cerr << "Converting to steps list..." << std::endl;
    StepsAdjacencyList minimal_steps = AdjacentPathsToStepsList(minimal);

    RemappedAdjacencyList remapped = RemapStopIds(minimal_steps);
    BoundaryIds remapped_boundary = RemapBoundary(remapped.mapping, boundary);

    // Helper to get stop name from remapped id, with special case for dummy nodes.
    auto stop_name = [&](StopId remapped_id) -> std::string {
        StopId original_id = remapped.mapping.new_to_original[remapped_id.v];
        if (original_id == boundary.start) {
            return "START";
        }
        if (original_id == boundary.end) {
            return "END";
        }
        if (original_id == boundary.anchor) {
            return "ANCHOR";
        }
        return mapping.stop_id_to_stop_name.at(original_id);
    };

    SolutionContext solution_context{mapping};
    SolutionState solution_initial_state{
        remapped.adj,
        remapped_boundary,
        remapped.mapping,
    };

    std::cout << "Evaluating initial state...\n";
    SolutionStateEvaluation eval = EvaluateState(solution_initial_state);
    PrintTourEvaluation(mapping, solution_initial_state, eval);

    for (int i = 0; i < eval.tour.size() - 1; ++i) {
        StopId a = eval.tour[i];
        StopId b = eval.tour[i + 1];

        if (b == remapped_boundary.anchor) {
            continue;
        }

        std::cout << "\n\nRequiring " << stop_name(a) << " -> " << stop_name(b) << "\n";

        SolutionState solution_right_state = MakeBranchRequireEdge(solution_initial_state, a, b);
        auto eval2 = EvaluateState(solution_right_state);
        PrintTourEvaluation(mapping, solution_right_state, eval2);

        if (i > 5) {
            return;
        }
    }

    for (int i = 0; i < eval.tour.size() - 1; ++i) {
        StopId a = eval.tour[i];
        StopId b = eval.tour[i + 1];

        if (b == remapped_boundary.anchor) {
            continue;
        }

        std::cout << "Forbidding " << stop_name(a) << " -> " << stop_name(b) << "\n";

        SolutionState solution_left_state = MakeBranchForbidEdge(solution_initial_state, a, b);
        EvaluateState(solution_left_state);
    }
}

}  // namespace vats5
