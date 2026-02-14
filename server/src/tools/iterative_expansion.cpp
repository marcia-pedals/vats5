#include <CLI/CLI.hpp>
#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <numeric>
#include <string>
#include <vector>

#include "solver/tarel_graph.h"

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

  // Collect required stops into a sorted vector with index mapping.
  std::vector<StopId> stops(
      state.required_stops.begin(), state.required_stops.end()
  );
  std::sort(stops.begin(), stops.end());
  int n = static_cast<int>(stops.size());

  std::cout << "Required stops (" << n << "):\n";
  for (const StopId& s : stops) {
    std::cout << "  " << state.StopName(s) << " (" << s.v << ")\n";
  }
  std::cout << "\n";

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
  std::vector<Edge> mst_edges;
  std::vector<int> degree(n, 0);

  for (const Edge& e : edges) {
    if (uf.Unite(e.u, e.v)) {
      mst_edges.push_back(e);
      degree[e.u]++;
      degree[e.v]++;
    }
  }

  std::cout << "MST edges (" << mst_edges.size() << "):\n";
  for (const Edge& e : mst_edges) {
    std::cout << "  " << state.StopName(stops[e.u]) << " -- "
              << state.StopName(stops[e.v]) << " (" << e.weight << "s)\n";
  }
  std::cout << "\n";

  // Find and print leaves (degree 1 in the MST).
  std::cout << "MST leaves:\n";
  for (int i = 0; i < n; i++) {
    if (degree[i] == 1) {
      std::cout << "  " << state.StopName(stops[i]) << " (" << stops[i].v
                << ")\n";
    }
  }

  return 0;
}
