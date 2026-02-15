#include <CLI/CLI.hpp>
#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include "solver/branch_and_bound.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"

using namespace vats5;

struct UnionFind {
  std::vector<int> parent, rank;

  UnionFind(int n) : parent(n), rank(n, 0) {
    std::iota(parent.begin(), parent.end(), 0);
  }

  int Find(int x) {
    if (parent[x] != x) parent[x] = Find(parent[x]);
    return parent[x];
  }

  bool Unite(int x, int y) {
    int px = Find(x), py = Find(y);
    if (px == py) return false;
    if (rank[px] < rank[py]) std::swap(px, py);
    parent[py] = px;
    if (rank[px] == rank[py]) rank[px]++;
    return true;
  }
};

// Builds an MST over the required stops (excluding start/end) using min
// duration as edge weight, and returns the leaves (degree-1 nodes).
std::vector<StopId> MstLeaves(const ProblemState& state) {
  // Collect required stops, excluding START and END.
  std::vector<StopId> stops(
      state.required_stops.begin(), state.required_stops.end()
  );
  std::erase_if(stops, [&](StopId s) {
    return s == state.boundary.start || s == state.boundary.end;
  });
  std::sort(stops.begin(), stops.end());
  int n = static_cast<int>(stops.size());

  // Build weighted undirected edges: for each pair of required stops, the
  // weight is the min duration step between them in either direction.
  struct Edge {
    int u, v;  // indices into stops vector
    int weight;
  };
  std::vector<Edge> edges;

  for (int i = 0; i < n; i++) {
    for (int k = i + 1; k < n; k++) {
      int min_duration = INT_MAX;

      auto paths_fw = state.completed.PathsBetween(stops[i], stops[k]);
      for (const Path& p : paths_fw) {
        min_duration = std::min(min_duration, p.DurationSeconds());
      }

      auto paths_bw = state.completed.PathsBetween(stops[k], stops[i]);
      for (const Path& p : paths_bw) {
        min_duration = std::min(min_duration, p.DurationSeconds());
      }

      if (min_duration < INT_MAX) {
        edges.push_back({i, k, min_duration});
      }
    }
  }

  // Kruskal's MST: sort edges by weight, greedily add via union-find.
  std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
    return a.weight < b.weight;
  });

  UnionFind uf(n);
  std::vector<int> degree(n, 0);

  for (const Edge& e : edges) {
    if (uf.Unite(e.u, e.v)) {
      degree[e.u]++;
      degree[e.v]++;
    }
  }

  // Collect leaves (degree 1 in the MST).
  std::vector<StopId> leaves;
  for (int i = 0; i < n; i++) {
    if (degree[i] == 1) {
      leaves.push_back(stops[i]);
    }
  }
  return leaves;
}

struct BestPathResult {
  std::vector<Path> paths;
  int duration;
};

// Recursively expands a combined stop into its original constituent stops.
void ExpandStop(
    StopId stop,
    const std::unordered_map<StopId, PlainEdge>& original_edges,
    std::vector<StopId>& out) {
  auto it = original_edges.find(stop);
  if (it == original_edges.end()) {
    out.push_back(stop);
    return;
  }
  ExpandStop(it->second.a, original_edges, out);
  ExpandStop(it->second.b, original_edges, out);
}

std::optional<BestPathResult> FindBestPathBranchAndBound(
  std::vector<StopId> leaves,
  const ProblemState& state
) {
  std::unordered_set<StopId> leaves_set;
  for (StopId leaf : leaves) {
    leaves_set.insert(leaf);
  }
  leaves_set.insert(state.boundary.start);
  leaves_set.insert(state.boundary.end);
  ProblemState state_on_leaves = MakeProblemState(
    MakeAdjacencyList(ReduceToMinimalSystemPaths(state.minimal, leaves_set).AllMergedSteps()),
    state.boundary,
    leaves_set,
    state.stop_names,
    state.step_partition_names,
    state.original_edges
  );

  auto bb_result = BranchAndBoundSolve(state_on_leaves, &std::cout);
  if (bb_result.best_paths.empty()) {
    return std::nullopt;
  }

  // Extract unique stop sequences from BB paths, expand combined stops back
  // to original stop IDs, then expand through the original state's completed
  // graph to recover all intermediate stops.
  std::set<std::vector<StopId>> seen_sequences;
  int best_duration = INT_MAX;
  std::vector<Path> best_paths;

  for (const Path& bb_path : bb_result.best_paths) {
    // Extract raw stop sequence (may contain combined stop IDs).
    std::vector<StopId> raw;
    raw.push_back(bb_path.steps.front().origin.stop);
    for (const Step& step : bb_path.steps) {
      raw.push_back(step.destination.stop);
    }

    // Expand combined stops to original stop IDs.
    std::vector<StopId> sequence;
    for (StopId s : raw) {
      ExpandStop(s, bb_result.original_edges, sequence);
    }
    if (!seen_sequences.insert(sequence).second) {
      continue;
    }

    std::vector<Path> expanded =
        ComputeMinimalFeasiblePathsAlong(sequence, state.completed);
    for (const Path& p : expanded) {
      if (p.DurationSeconds() < best_duration) {
        best_duration = p.DurationSeconds();
        best_paths.clear();
        best_paths.push_back(p);
      } else if (p.DurationSeconds() == best_duration) {
        best_paths.push_back(p);
      }
    }
  }

  if (best_paths.empty()) {
    return std::nullopt;
  }
  return BestPathResult{std::move(best_paths), best_duration};
}

// Tries all permutations of `leaves` as intermediate stops between start and
// end, and returns the permutation yielding the shortest feasible path.
std::optional<BestPathResult> FindBestPermutationPath(
    std::vector<StopId> leaves,
    const ProblemState& state) {
  std::sort(leaves.begin(), leaves.end());

  int best_duration = INT_MAX;
  BestPathResult best;

  do {
    std::vector<StopId> sequence;
    sequence.push_back(state.boundary.start);
    sequence.insert(sequence.end(), leaves.begin(), leaves.end());
    sequence.push_back(state.boundary.end);

    std::vector<Path> paths =
        ComputeMinimalFeasiblePathsAlong(sequence, state.completed);

    auto min_it = std::min_element(
        paths.begin(), paths.end(),
        [](const Path& a, const Path& b) {
          return a.DurationSeconds() < b.DurationSeconds();
        });
    if (min_it != paths.end() && min_it->DurationSeconds() < best_duration) {
      best_duration = min_it->DurationSeconds();
      std::erase_if(paths, [&](const Path& p) {
        return p.DurationSeconds() != best_duration;
      });
      best.paths = std::move(paths);
      best.duration = best_duration;
    }
  } while (std::next_permutation(leaves.begin(), leaves.end()));

  if (best.paths.empty()) {
    return std::nullopt;
  }
  return best;
}

// For each required stop not on the path, computes its shortest "distance" to
// the path. Distance from required stop x to path stop p: find the
// min-duration path from x to p in state.completed, then count how many
// required stops are on that path (including both endpoints). The distance
// from x to the overall path is the minimum across all path stops p.
std::unordered_map<StopId, int> RequiredStopDistances(
    const Path& path,
    const ProblemState& state) {
  std::unordered_set<StopId> visited;
  visited.insert(path.steps.front().origin.stop);
  path.VisitIntermediateStops([&](StopId s) { visited.insert(s); });
  visited.insert(path.steps.back().destination.stop);

  std::unordered_map<StopId, int> distances;
  for (StopId x : state.required_stops) {
    if (visited.contains(x)) {
      continue;
    }
    int min_dist = INT_MAX;
    for (StopId p : visited) {
      auto paths_xp = state.completed.PathsBetween(x, p);
      const Path* shortest = nullptr;
      for (const Path& candidate : paths_xp) {
        if (!shortest ||
            candidate.DurationSeconds() < shortest->DurationSeconds()) {
          shortest = &candidate;
        }
      }
      if (shortest) {
        // Count required stops on the connecting path, including endpoints.
        int count = 0;
        if (state.required_stops.contains(x)) count++;
        shortest->VisitIntermediateStops([&](StopId s) {
          if (state.required_stops.contains(s)) {
            count++;
          }
        });
        min_dist = std::min(min_dist, count);
      }
    }
    distances[x] = min_dist;
  }
  return distances;
}

int main(int argc, char* argv[]) {
  CLI::App app{"Iterative expansion tool"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to problem state JSON")
      ->required();

  CLI11_PARSE(app, argc, argv);

  // Load problem state.
  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << "\n";
    return 1;
  }

  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState state = j.get<ProblemState>();

  std::vector<StopId> leaves = MstLeaves(state);

  for (int iteration = 0; ; iteration++) {
    std::cout << "=== Iteration " << iteration << ": branch and bound on "
              << leaves.size() << " leaves ===\n";
    auto best = FindBestPathBranchAndBound(leaves, state);

    if (!best) {
      std::cout << "No feasible path found.\n";
      return 1;
    }

    // Pick the path with the lowest total distance to unvisited required stops.
    const Path* best_path = &best->paths[0];
    auto distances = RequiredStopDistances(*best_path, state);
    int total_dist = 0;
    for (const auto& [_, d] : distances) total_dist += d;
    for (size_t i = 1; i < best->paths.size(); i++) {
      auto d = RequiredStopDistances(best->paths[i], state);
      int td = 0;
      for (const auto& [_, v] : d) td += v;
      if (td < total_dist) {
        best_path = &best->paths[i];
        distances = std::move(d);
        total_dist = td;
      }
    }

    std::cout << "\nBest duration: " << TimeSinceServiceStart{best->duration}.ToString() << "\n";

    std::cout << "Path (" << best_path->steps.size() << " steps):\n";
    for (const Step& step : best_path->steps) {
      std::cout << "  " << state.StopName(step.origin.stop) << " ("
                << step.origin.time.ToString() << ") -> "
                << state.StopName(step.destination.stop) << " ("
                << step.destination.time.ToString() << ")\n";
    }

    // Collect unvisited required stops, sorted by distance (descending).
    std::vector<std::pair<int, StopId>> unvisited;
    for (const auto& [s, d] : distances) {
      unvisited.emplace_back(d, s);
    }

    if (unvisited.empty()) {
      std::cout << "\nAll required stops are visited.\n";
      break;
    }

    std::sort(unvisited.begin(), unvisited.end());
    std::cout << "\nRequired stops NOT visited (" << unvisited.size() << "):\n";
    for (const auto& [d, s] : unvisited) {
      std::cout << "  " << state.StopName(s) << " (distance: " << d << ")\n";
    }

    // Add the farthest unvisited stop to leaves for the next iteration.
    StopId farthest = unvisited.back().second;
    std::cout << "\nAdding farthest stop: " << state.StopName(farthest) << "\n\n";
    leaves.push_back(farthest);
  }

  return 0;
}
