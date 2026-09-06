#include "solver/gtsp_concorde.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "solver/concorde.h"
#include "solver/concorde_shim.h"
#include "solver/concorde_workdir.h"

namespace vats5 {
namespace {

// ---------------------------------------------------------------------------
// Encoding overview
//
// Noon-Bean: the vertices of every cluster are arranged in a cycle with
// zero-cost arcs v -> next(v). Every inter-cluster arc u -> w is redirected to
// start at prev(u) and gets +M. A tour then enters a cluster at some vertex e,
// walks the whole cycle e, next(e), ..., prev(e) for free, and leaves from
// prev(e) using an arc that originally belonged to e, i.e. it "visits" e.
//
// Doubling: ATSP vertex i becomes in(i) = 2i and out(i) = 2i + 1 joined by a
// zero edge; arc a -> b becomes the edge out(a) -- in(b) with the arc's weight
// plus a constant K. A symmetric tour that uses every zero edge is an ATSP
// tour.
//
// Why the constants can be small. For N ATSP vertices and C clusters a tour of
// the intended shape ("proper") costs N*K + C*M + r, where r is its GTSP cost.
// Any other Hamiltonian cycle of the doubled graph either uses more than C
// inter-cluster arcs (costing at least one extra M) or traverses some cluster
// cycle "the wrong way round", which costs one extra K per such cluster and in
// exchange lets the tour leave that cluster as if from next(e) instead of e.
// Because GTSP edge weights are time differences, that is worth exactly
// time(next(e)) - time(e) when next(e) is a later vertex of the same stop
// (it amounts to waiting there, which a proper tour can do at no extra
// cost). So cycles are ordered by stop and time, and a dummy vertex separates
// stops, breaks any forward gap longer than kMaxCycleGapSeconds, and sits at
// the wrap-around, so no usable jump ever changes stop or goes back in time.
// Such a tour then costs at least K - kMaxCycleGapSeconds > 0 more than the
// best proper tour. Hence M > (max GTSP tour cost) and K > kMaxCycleGapSeconds suffice,
// and total tour costs stay far below Concorde's ~2^30 exact-arithmetic
// limit. Dummies have no arcs of their own (only their cluster's cycle arcs
// touch them), so a tour can neither "visit" a dummy nor leave a cluster
// through a dummy's arcs.
// ---------------------------------------------------------------------------

// Longest forward time jump allowed between a cluster-cycle vertex and its
// successor; longer gaps get a dummy vertex in between.
constexpr int kMaxCycleGapSeconds = 600;

// Per-edge offset K in the doubled graph. Must exceed kMaxCycleGapSeconds.
constexpr int kDoublingOffset = kMaxCycleGapSeconds + 1;

// Below this many ATSP vertices the doubled graph has fewer than 10 nodes,
// which Concorde's library entry point does not handle; use brute force.
constexpr int kBruteForceThreshold = 5;

struct AtspVertex {
  // GTSP vertex, or -1 for a dummy.
  int gtsp_vertex;
  // Dense cluster id.
  int cluster;
  // Neighbors in the cluster cycle; self for a single-vertex cluster.
  int prev;
  int next;
};

struct AtspArc {
  int from;
  int to;
  int weight;
};

struct NoonBean {
  std::vector<AtspVertex> vertices;
  // Sorted by (from, to), no duplicates.
  std::vector<AtspArc> arcs;
  std::vector<int> cluster_size;
  // ATSP vertex of every GTSP vertex.
  std::vector<int> atsp_of_gtsp;
  int num_dummies = 0;
  // Noon-Bean offset M added to every inter-cluster arc.
  int big_m = 0;
  // Span of the real vertex times, an upper bound on any GTSP tour cost.
  int span_seconds = 0;
  int start = 0;
  int end = 0;
};

NoonBean BuildNoonBean(
    const Gtsp& gtsp, const std::vector<int>& cluster_of_vertex
) {
  int n = static_cast<int>(gtsp.vertices.size());
  if (static_cast<int>(cluster_of_vertex.size()) != n) {
    throw std::invalid_argument("cluster_of_vertex size != number of vertices");
  }

  std::unordered_map<int, int> dense_cluster;
  std::vector<int> cluster(n);
  for (int v = 0; v < n; ++v) {
    cluster[v] = dense_cluster
                     .try_emplace(
                         cluster_of_vertex[v],
                         static_cast<int>(dense_cluster.size())
                     )
                     .first->second;
  }
  int num_clusters = static_cast<int>(dense_cluster.size());

  std::vector<std::vector<int>> members(num_clusters);
  for (int v = 0; v < n; ++v) {
    members[cluster[v]].push_back(v);
  }
  if (members[cluster[Gtsp::kStart]].size() != 1 ||
      members[cluster[Gtsp::kEnd]].size() != 1) {
    throw std::invalid_argument(
        "START and END must be the only members of their clusters"
    );
  }

  int min_time = INT_MAX;
  int max_time = INT_MIN;
  for (int v = 2; v < n; ++v) {
    min_time = std::min(min_time, gtsp.vertices[v].time.seconds);
    max_time = std::max(max_time, gtsp.vertices[v].time.seconds);
  }
  NoonBean nb;
  nb.span_seconds = (n > 2) ? max_time - min_time : 0;
  nb.big_m = nb.span_seconds + 1;
  nb.cluster_size.assign(num_clusters, 0);

  std::vector<int> atsp_of_gtsp(n, -1);
  auto add_vertex = [&](int gtsp_vertex, int c) {
    int id = static_cast<int>(nb.vertices.size());
    nb.vertices.push_back({gtsp_vertex, c, id, id});
    nb.cluster_size[c] += 1;
    if (gtsp_vertex >= 0) {
      atsp_of_gtsp[gtsp_vertex] = id;
    } else {
      nb.num_dummies += 1;
    }
    return id;
  };

  for (int c = 0; c < num_clusters; ++c) {
    std::vector<int>& vs = members[c];
    // Cycle order: by stop, then time. A cluster may contain several stops;
    // a forward jump must never change stop, only wait at one.
    std::sort(vs.begin(), vs.end(), [&](int a, int b) {
      return std::make_tuple(
                 gtsp.vertices[a].stop.v, gtsp.vertices[a].time.seconds, a
             ) < std::make_tuple(
                     gtsp.vertices[b].stop.v, gtsp.vertices[b].time.seconds, b
                 );
    });
    if (vs.size() == 1) {
      add_vertex(vs[0], c);
      continue;
    }
    std::vector<int> cycle;
    for (size_t i = 0; i < vs.size(); ++i) {
      if (i > 0) {
        const GtspVertex& prev = gtsp.vertices[vs[i - 1]];
        const GtspVertex& cur = gtsp.vertices[vs[i]];
        if (cur.stop != prev.stop ||
            cur.time.seconds - prev.time.seconds > kMaxCycleGapSeconds) {
          cycle.push_back(add_vertex(-1, c));
        }
      }
      cycle.push_back(add_vertex(vs[i], c));
    }
    // Wrap-around dummy: the step from the latest vertex back to the earliest
    // must not be a usable jump.
    cycle.push_back(add_vertex(-1, c));
    for (size_t i = 0; i < cycle.size(); ++i) {
      int v = cycle[i];
      nb.vertices[v].next = cycle[(i + 1) % cycle.size()];
      nb.vertices[v].prev = cycle[(i + cycle.size() - 1) % cycle.size()];
    }
  }
  nb.start = atsp_of_gtsp[Gtsp::kStart];
  nb.end = atsp_of_gtsp[Gtsp::kEnd];
  nb.atsp_of_gtsp = atsp_of_gtsp;

  // Cluster cycle arcs.
  for (int v = 0; v < static_cast<int>(nb.vertices.size()); ++v) {
    if (nb.vertices[v].next != v) {
      nb.arcs.push_back({v, nb.vertices[v].next, 0});
    }
  }
  // Inter-cluster arcs, redirected to start at the predecessor of their
  // origin. Arcs inside a cluster can never be part of a GTSP tour.
  for (const GtspEdge& e : gtsp.edges) {
    if (cluster[e.from] == cluster[e.to]) {
      continue;
    }
    if (e.weight_seconds < 0) {
      throw std::invalid_argument("negative GTSP edge weight");
    }
    nb.arcs.push_back(
        {nb.vertices[atsp_of_gtsp[e.from]].prev,
         atsp_of_gtsp[e.to],
         e.weight_seconds + nb.big_m}
    );
  }
  // Dummies have no arcs of their own: they are only ever traversed as part
  // of their cluster's cycle.

  std::sort(nb.arcs.begin(), nb.arcs.end(), [](const AtspArc& a, const AtspArc& b) {
    return std::make_tuple(a.from, a.to, a.weight) <
           std::make_tuple(b.from, b.to, b.weight);
  });
  nb.arcs.erase(
      std::unique(
          nb.arcs.begin(),
          nb.arcs.end(),
          [](const AtspArc& a, const AtspArc& b) {
            return a.from == b.from && a.to == b.to;
          }
      ),
      nb.arcs.end()
  );
  return nb;
}

std::unordered_map<int64_t, int> EdgeWeightMap(const Gtsp& gtsp) {
  std::unordered_map<int64_t, int> edge_weight;
  for (const GtspEdge& e : gtsp.edges) {
    edge_weight.try_emplace(
        (static_cast<int64_t>(e.from) << 32) | static_cast<uint32_t>(e.to),
        e.weight_seconds
    );
  }
  return edge_weight;
}

std::optional<int> LookupEdge(
    const std::unordered_map<int64_t, int>& edge_weight, int from, int to
) {
  auto it = edge_weight.find(
      (static_cast<int64_t>(from) << 32) | static_cast<uint32_t>(to)
  );
  if (it == edge_weight.end()) {
    return std::nullopt;
  }
  return it->second;
}

// The ATSP tour (cyclic vertex sequence starting at START) that encodes a
// GTSP tour: each visited vertex followed by the rest of its cluster cycle.
// Throws std::invalid_argument if the GTSP tour is not a valid tour.
std::vector<int> EncodeNoonBeanTour(
    const NoonBean& nb, const Gtsp& gtsp, const std::vector<int>& gtsp_tour
) {
  if (gtsp_tour.empty() || gtsp_tour.front() != Gtsp::kStart ||
      gtsp_tour.back() != Gtsp::kEnd) {
    throw std::invalid_argument("initial tour must run from START to END");
  }
  std::unordered_map<int64_t, int> edge_weight = EdgeWeightMap(gtsp);
  std::vector<bool> cluster_seen(nb.cluster_size.size(), false);
  std::vector<int> atsp_tour;
  atsp_tour.reserve(nb.vertices.size());
  for (size_t k = 0; k < gtsp_tour.size(); ++k) {
    int v = gtsp_tour[k];
    if (v < 0 || v >= static_cast<int>(gtsp.vertices.size())) {
      throw std::invalid_argument("initial tour has an invalid vertex");
    }
    if (k + 1 < gtsp_tour.size() &&
        !LookupEdge(edge_weight, v, gtsp_tour[k + 1])) {
      throw std::invalid_argument(
          "initial tour uses a non-edge " + std::to_string(v) + " -> " +
          std::to_string(gtsp_tour[k + 1])
      );
    }
    int entry = nb.atsp_of_gtsp[v];
    int c = nb.vertices[entry].cluster;
    if (cluster_seen[c]) {
      throw std::invalid_argument("initial tour visits a cluster twice");
    }
    cluster_seen[c] = true;
    int cur = entry;
    for (int j = 0; j < nb.cluster_size[c]; ++j) {
      atsp_tour.push_back(cur);
      cur = nb.vertices[cur].next;
    }
  }
  if (atsp_tour.size() != nb.vertices.size()) {
    throw std::invalid_argument("initial tour does not visit every cluster");
  }
  return atsp_tour;
}

// The GTSP tour encoded by an ATSP tour of the Noon-Bean graph (a cyclic
// vertex sequence). Throws InvalidTourStructure if the ATSP tour does not
// have the proper shape.
GtspSolution DecodeNoonBeanTour(
    const NoonBean& nb, const Gtsp& gtsp, const std::vector<int>& atsp_tour
) {
  int num_vertices = static_cast<int>(nb.vertices.size());
  if (static_cast<int>(atsp_tour.size()) != num_vertices) {
    throw InvalidTourStructure("ATSP tour has the wrong length");
  }
  auto start_it = std::find(atsp_tour.begin(), atsp_tour.end(), nb.start);
  if (start_it == atsp_tour.end()) {
    throw InvalidTourStructure("ATSP tour does not contain START");
  }
  std::vector<int> tour(start_it, atsp_tour.end());
  tour.insert(tour.end(), atsp_tour.begin(), start_it);

  auto format_tour = [&]() {
    std::ostringstream s;
    s << "ATSP tour:";
    for (int v : tour) {
      s << " " << v << "(c" << nb.vertices[v].cluster << ")";
    }
    return s.str();
  };

  std::vector<int> gtsp_tour;
  int i = 0;
  while (i < num_vertices) {
    int entry = tour[i];
    if (nb.vertices[entry].gtsp_vertex < 0) {
      throw InvalidTourStructure(
          "Cluster entered at a dummy vertex at position " +
          std::to_string(i) + ". " + format_tour()
      );
    }
    int size = nb.cluster_size[nb.vertices[entry].cluster];
    int cur = entry;
    for (int j = 0; j < size; ++j) {
      if (i + j >= num_vertices || tour[i + j] != cur) {
        throw InvalidTourStructure(
            "Cluster cycle not traversed in order at position " +
            std::to_string(i + j) + ". " + format_tour()
        );
      }
      cur = nb.vertices[cur].next;
    }
    gtsp_tour.push_back(nb.vertices[entry].gtsp_vertex);
    i += size;
  }
  if (gtsp_tour.back() != Gtsp::kEnd) {
    throw InvalidTourStructure("Tour does not end at END. " + format_tour());
  }

  std::unordered_map<int64_t, int> edge_weight = EdgeWeightMap(gtsp);
  int cost = 0;
  for (size_t k = 0; k + 1 < gtsp_tour.size(); ++k) {
    std::optional<int> w =
        LookupEdge(edge_weight, gtsp_tour[k], gtsp_tour[k + 1]);
    if (!w) {
      throw InvalidTourStructure(
          "Decoded tour uses a non-edge " + std::to_string(gtsp_tour[k]) +
          " -> " + std::to_string(gtsp_tour[k + 1]) + ". " + format_tour()
      );
    }
    cost += *w;
  }
  return GtspSolution{.tour = std::move(gtsp_tour), .cost_seconds = cost};
}

// Optimal ATSP tour (cyclic vertex sequence starting at START) by
// enumeration, for tiny instances. Returns nullopt if there is no tour.
std::optional<std::vector<int>> BruteForceAtspTour(const NoonBean& nb) {
  int n = static_cast<int>(nb.vertices.size());
  std::unordered_map<int64_t, int> arc_weight;
  for (const AtspArc& a : nb.arcs) {
    arc_weight[(static_cast<int64_t>(a.from) << 32) |
               static_cast<uint32_t>(a.to)] = a.weight;
  }
  auto weight = [&](int a, int b) -> std::optional<int> {
    auto it = arc_weight.find(
        (static_cast<int64_t>(a) << 32) | static_cast<uint32_t>(b)
    );
    if (it == arc_weight.end()) {
      return std::nullopt;
    }
    return it->second;
  };

  std::vector<int> rest;
  for (int v = 0; v < n; ++v) {
    if (v != nb.start) {
      rest.push_back(v);
    }
  }
  std::optional<std::vector<int>> best;
  int64_t best_cost = 0;
  do {
    int64_t cost = 0;
    bool valid = true;
    int prev = nb.start;
    for (int v : rest) {
      auto w = weight(prev, v);
      if (!w) {
        valid = false;
        break;
      }
      cost += *w;
      prev = v;
    }
    if (valid) {
      auto w = weight(prev, nb.start);
      if (!w) {
        valid = false;
      } else {
        cost += *w;
      }
    }
    if (valid && (!best || cost < best_cost)) {
      best = std::vector<int>{nb.start};
      best->insert(best->end(), rest.begin(), rest.end());
      best_cost = cost;
    }
  } while (std::next_permutation(rest.begin(), rest.end()));
  return best;
}

// Validates the doubled tour (alternating in/out pairs of the same ATSP
// vertex) and extracts the ATSP vertex sequence, oriented so that each
// vertex is traversed in -> out.
std::vector<int> ExtractAtspTour(const std::vector<int>& doubled_tour) {
  int doubled_n = static_cast<int>(doubled_tour.size());
  int n = doubled_n / 2;
  auto orig = [](int v) { return v / 2; };
  auto is_in = [](int v) { return v % 2 == 0; };

  int start_idx = -1;
  for (int i = 0; i < doubled_n; ++i) {
    if (orig(doubled_tour[i]) == orig(doubled_tour[(i + 1) % doubled_n])) {
      start_idx = i;
      break;
    }
  }
  if (start_idx == -1) {
    throw InvalidTourStructure("Doubled tour has no in/out pair");
  }
  bool reversed = !is_in(doubled_tour[start_idx]);
  std::vector<int> tour;
  tour.reserve(n);
  for (int i = 0; i < n; ++i) {
    int v1 = doubled_tour[(start_idx + 2 * i) % doubled_n];
    int v2 = doubled_tour[(start_idx + 2 * i + 1) % doubled_n];
    if (orig(v1) != orig(v2) || is_in(v1) == is_in(v2)) {
      throw InvalidTourStructure(
          "Doubled tour is not made of in/out pairs at position " +
          std::to_string(i)
      );
    }
    tour.push_back(orig(v1));
  }
  if (reversed) {
    std::reverse(tour.begin(), tour.end());
  }
  return tour;
}

std::optional<GtspSolution> SolveWithConcordeImpl(
    const Gtsp& gtsp,
    const NoonBean& nb,
    const std::optional<GtspSolution>& initial_tour,
    std::ostream* tsp_log,
    int seed
) {
  int n = static_cast<int>(nb.vertices.size());
  int doubled_n = 2 * n;
  int num_clusters = static_cast<int>(nb.cluster_size.size());

  // Every proper tour costs n*K + C*M + r with r <= span, so this bounds the
  // optimum. Artificial (unlisted) edges must cost more than that, and all
  // of it must stay well inside int and Concorde's exact arithmetic.
  int64_t proper_bound = static_cast<int64_t>(n) * kDoublingOffset +
                         static_cast<int64_t>(num_clusters) * nb.big_m +
                         nb.span_seconds;
  if (16 * proper_bound > INT_MAX) {
    throw EdgeWeightOverflow(
        "Encoded tour cost bound " + std::to_string(proper_bound) +
        " is too large for Concorde's int edge lengths"
    );
  }
  int default_len = static_cast<int>(8 * proper_bound);

  std::vector<int> elist;
  std::vector<int> elen;
  elist.reserve(2 * (n + nb.arcs.size()));
  elen.reserve(n + nb.arcs.size());
  for (int v = 0; v < n; ++v) {
    elist.push_back(2 * v);
    elist.push_back(2 * v + 1);
    elen.push_back(0);
  }
  for (const AtspArc& a : nb.arcs) {
    elist.push_back(2 * a.from + 1);
    elist.push_back(2 * a.to);
    elen.push_back(a.weight + kDoublingOffset);
  }
  int ecount = static_cast<int>(elen.size());

  if (tsp_log) {
    *tsp_log << "GTSP->Concorde encoding: " << gtsp.vertices.size()
             << " GTSP vertices, " << gtsp.edges.size() << " GTSP edges, "
             << num_clusters << " clusters, " << nb.num_dummies
             << " dummies, " << n << " ATSP vertices, " << nb.arcs.size()
             << " arcs; doubled: " << doubled_n << " nodes, " << ecount
             << " edges; M=" << nb.big_m << " K=" << kDoublingOffset
             << " default_len=" << default_len << "\n"
             << std::flush;
  }

  std::vector<int> in_tour;
  if (initial_tour) {
    for (int a : EncodeNoonBeanTour(nb, gtsp, initial_tour->tour)) {
      in_tour.push_back(2 * a);
      in_tour.push_back(2 * a + 1);
    }
    if (tsp_log) {
      *tsp_log << "Initial tour given: GTSP cost " << initial_tour->cost_seconds
               << "\n"
               << std::flush;
    }
  }

  WorkDir work_dir;
  std::string log_path = work_dir.path() + "/log";
  std::cout.flush();
  std::cerr.flush();

  int success = 0;
  int found_tour = 0;
  double optval = 0.0;
  std::vector<int> doubled_tour(doubled_n);
  int rval = vats5_concorde_solve_sparse(
      doubled_n,
      ecount,
      elist.data(),
      elen.data(),
      default_len,
      initial_tour ? in_tour.data() : nullptr,
      /*upper_bound=*/nullptr,
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
  // Either the LP was infeasible or the best tour needs an unlisted edge:
  // no tour through the listed edges exists.
  if (!found_tour) {
    return std::nullopt;
  }

  GtspSolution solution =
      DecodeNoonBeanTour(nb, gtsp, ExtractAtspTour(doubled_tour));
  int64_t expected = static_cast<int64_t>(n) * kDoublingOffset +
                     static_cast<int64_t>(num_clusters) * nb.big_m +
                     solution.cost_seconds;
  int64_t reported = static_cast<int64_t>(std::llround(optval));
  if (reported != expected) {
    throw InvalidTourStructure(
        "Concorde tour value " + std::to_string(reported) +
        " != decoded value " + std::to_string(expected)
    );
  }
  return solution;
}

}  // namespace

std::vector<int> GtspClustersFromRequired(
    const Gtsp& gtsp, const RequiredStops& required
) {
  std::vector<int> clusters;
  clusters.reserve(gtsp.vertices.size());
  for (const GtspVertex& v : gtsp.vertices) {
    if (!required.Contains(v.stop)) {
      throw std::invalid_argument(
          "GTSP vertex stop " + std::to_string(v.stop.v) + " is not required"
      );
    }
    clusters.push_back(required.Representative(v.stop).v);
  }
  return clusters;
}

std::optional<GtspSolution> GtspTourAlongStops(
    const Gtsp& gtsp, const std::vector<StopId>& stops
) {
  int n = static_cast<int>(gtsp.vertices.size());
  if (stops.size() < 2 || stops.front() != gtsp.vertices[Gtsp::kStart].stop ||
      stops.back() != gtsp.vertices[Gtsp::kEnd].stop) {
    throw std::invalid_argument("stops must run from the START stop to END");
  }

  // Edges grouped by origin vertex.
  std::vector<int> edge_start(n + 1, 0);
  for (const GtspEdge& e : gtsp.edges) {
    edge_start[e.from + 1]++;
  }
  for (int v = 0; v < n; ++v) {
    edge_start[v + 1] += edge_start[v];
  }
  std::vector<int> edge_index(gtsp.edges.size());
  {
    std::vector<int> fill(edge_start.begin(), edge_start.end() - 1);
    for (size_t i = 0; i < gtsp.edges.size(); ++i) {
      edge_index[fill[gtsp.edges[i].from]++] = static_cast<int>(i);
    }
  }

  // Because path costs telescope, a tour's cost is the time of its last
  // vertex minus the time of its first, plus the START edge's weight. What
  // can follow a vertex depends only on the vertex, so along a fixed order
  // the best tour through a given vertex is the one reaching it with the
  // largest (first-vertex time - START edge weight); best_first[v] holds
  // that value for v's layer and parent[v] the predecessor achieving it.
  constexpr int kUnreachable = INT_MIN;
  std::vector<int> best_first(n, kUnreachable);
  std::vector<int> parent(n, -1);
  std::vector<int> layer{Gtsp::kStart};
  best_first[Gtsp::kStart] = 0;
  for (size_t i = 1; i + 1 < stops.size(); ++i) {
    std::vector<int> next_layer;
    for (int u : layer) {
      for (int k = edge_start[u]; k < edge_start[u + 1]; ++k) {
        const GtspEdge& e = gtsp.edges[edge_index[k]];
        if (gtsp.vertices[e.to].stop != stops[i]) {
          continue;
        }
        int first = (u == Gtsp::kStart)
                        ? gtsp.vertices[e.to].time.seconds - e.weight_seconds
                        : best_first[u];
        if (best_first[e.to] == kUnreachable) {
          next_layer.push_back(e.to);
        }
        if (first > best_first[e.to]) {
          best_first[e.to] = first;
          parent[e.to] = u;
        }
      }
    }
    if (next_layer.empty()) {
      return std::nullopt;
    }
    layer = std::move(next_layer);
  }

  // The last layer: pick the vertex with the cheapest tour through it, then
  // walk the parents back.
  std::unordered_map<int64_t, int> edge_weight = EdgeWeightMap(gtsp);
  std::optional<int> best_cost;
  int best_last = -1;
  for (int u : layer) {
    if (u == Gtsp::kStart || !LookupEdge(edge_weight, u, Gtsp::kEnd)) {
      continue;
    }
    int cost = gtsp.vertices[u].time.seconds - best_first[u];
    if (!best_cost || cost < *best_cost) {
      best_cost = cost;
      best_last = u;
    }
  }
  if (!best_cost) {
    return std::nullopt;
  }
  std::vector<int> tour{Gtsp::kEnd};
  for (int v = best_last; v != -1; v = parent[v]) {
    tour.push_back(v);
  }
  std::reverse(tour.begin(), tour.end());
  int cost = 0;
  for (size_t k = 0; k + 1 < tour.size(); ++k) {
    cost += *LookupEdge(edge_weight, tour[k], tour[k + 1]);
  }
  if (cost != *best_cost) {
    throw InvalidTourStructure("GtspTourAlongStops cost mismatch");
  }
  return GtspSolution{.tour = std::move(tour), .cost_seconds = cost};
}

std::optional<GtspSolution> SolveGtspWithConcorde(
    const Gtsp& gtsp,
    const std::vector<int>& cluster_of_vertex,
    const std::optional<GtspSolution>& initial_tour,
    std::ostream* tsp_log
) {
  NoonBean nb = BuildNoonBean(gtsp, cluster_of_vertex);

  if (static_cast<int>(nb.vertices.size()) < kBruteForceThreshold) {
    std::optional<std::vector<int>> atsp_tour = BruteForceAtspTour(nb);
    if (!atsp_tour) {
      return std::nullopt;
    }
    return DecodeNoonBeanTour(nb, gtsp, *atsp_tour);
  }

  constexpr int kMaxRetries = 5;
  constexpr int kBaseSeed = 43;
  for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
    try {
      return SolveWithConcordeImpl(
          gtsp, nb, initial_tour, tsp_log, kBaseSeed + attempt - 1
      );
    } catch (const InvalidTourStructure&) {
      throw;
    } catch (const EdgeWeightOverflow&) {
      throw;
    } catch (const ConcordeFailure&) {
      if (attempt == kMaxRetries) {
        throw;
      }
    }
  }
  __builtin_unreachable();
}

}  // namespace vats5
