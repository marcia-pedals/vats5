#include <CLI/CLI.hpp>
#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <numeric>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

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
  std::vector<StopId> sequence;
  int duration;
};

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
      best.sequence = std::move(sequence);
      best.duration = best_duration;
    }
  } while (std::next_permutation(leaves.begin(), leaves.end()));

  if (best.paths.empty()) {
    return std::nullopt;
  }
  return best;
}

// Returns required stops not visited by `path`.
std::vector<StopId> MissingRequiredStops(
    const Path& path,
    const std::unordered_set<StopId>& required_stops) {
  std::unordered_set<StopId> visited;
  visited.insert(path.steps.front().origin.stop);
  path.VisitIntermediateStops([&](StopId s) { visited.insert(s); });

  std::vector<StopId> missing;
  for (StopId s : required_stops) {
    if (!visited.contains(s)) {
      missing.push_back(s);
    }
  }
  return missing;
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

  std::cout << "Evaluating permutations of " << leaves.size() << " MST leaves:\n";
  auto best = FindBestPermutationPath(leaves, state);

  if (!best) {
    std::cout << "No feasible path found for any permutation.\n";
    return 1;
  }

  // Pick the path with the fewest missing required stops.
  const Path* best_path = &best->paths[0];
  std::vector<StopId> missing =
      MissingRequiredStops(*best_path, state.required_stops);
  for (size_t i = 1; i < best->paths.size(); i++) {
    auto m = MissingRequiredStops(best->paths[i], state.required_stops);
    if (m.size() < missing.size()) {
      best_path = &best->paths[i];
      missing = std::move(m);
    }
  }

  std::cout << "\nBest duration: " << TimeSinceServiceStart{best->duration}.ToString() << "\n";
  std::cout << "Stop sequence: ";
  for (size_t i = 0; i < best->sequence.size(); i++) {
    if (i > 0) std::cout << " -> ";
    std::cout << state.StopName(best->sequence[i]);
  }
  std::cout << "\n";

  std::cout << "Path (" << best_path->steps.size() << " steps):\n";
  for (const Step& step : best_path->steps) {
    std::cout << "  " << state.StopName(step.origin.stop) << " ("
              << step.origin.time.ToString() << ") -> "
              << state.StopName(step.destination.stop) << " ("
              << step.destination.time.ToString() << ")\n";
  }
  if (missing.empty()) {
    std::cout << "\nAll required stops are visited.\n";
  } else {
    std::cout << "\nRequired stops NOT visited (" << missing.size() << "):\n";
    for (StopId s : missing) {
      std::cout << "  " << state.StopName(s) << "\n";
    }
  }

  return 0;
}
