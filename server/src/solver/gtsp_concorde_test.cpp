#include "solver/gtsp_concorde.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <algorithm>
#include <limits>
#include <optional>
#include <sstream>
#include <vector>

#include "solver/data.h"
#include "solver/gtsp.h"
#include "solver/steps_adjacency_list.h"
#include "solver/held_karp_dp.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {
namespace {

// Vertices 2..: (stop, time). Edges as given, plus the START/END edges.
Gtsp MakeGtsp(
    const std::vector<GtspVertex>& vertices, std::vector<GtspEdge> edges
) {
  Gtsp gtsp;
  gtsp.vertices.push_back({StopId{100}, TimeSinceServiceStart{0}});
  gtsp.vertices.push_back({StopId{101}, TimeSinceServiceStart{0}});
  gtsp.vertices.insert(gtsp.vertices.end(), vertices.begin(), vertices.end());
  gtsp.edges = std::move(edges);
  for (int v = 2; v < static_cast<int>(gtsp.vertices.size()); ++v) {
    gtsp.edges.push_back({Gtsp::kStart, v, 0});
    gtsp.edges.push_back({v, Gtsp::kEnd, 0});
  }
  gtsp.edges.push_back({Gtsp::kEnd, Gtsp::kStart, 0});
  return gtsp;
}

std::vector<int> ClustersByStop(const Gtsp& gtsp) {
  std::vector<int> clusters;
  for (const GtspVertex& v : gtsp.vertices) {
    clusters.push_back(v.stop.v);
  }
  return clusters;
}

// Brute force over the GTSP: every ordering of the non-boundary clusters and
// every choice of vertex per cluster.
std::optional<int> BruteForceGtsp(
    const Gtsp& gtsp, const std::vector<int>& clusters
) {
  std::vector<int> cluster_ids;
  for (size_t v = 2; v < gtsp.vertices.size(); ++v) {
    if (std::find(cluster_ids.begin(), cluster_ids.end(), clusters[v]) ==
        cluster_ids.end()) {
      cluster_ids.push_back(clusters[v]);
    }
  }
  std::sort(cluster_ids.begin(), cluster_ids.end());

  auto weight = [&](int a, int b) -> std::optional<int> {
    for (const GtspEdge& e : gtsp.edges) {
      if (e.from == a && e.to == b) {
        return e.weight_seconds;
      }
    }
    return std::nullopt;
  };

  std::optional<int> best;
  // Recursive choice of one vertex per cluster in the current order.
  std::function<void(size_t, int, int)> choose = [&](size_t i, int prev,
                                                     int cost) {
    if (i == cluster_ids.size()) {
      auto w = weight(prev, Gtsp::kEnd);
      if (w && (!best || cost + *w < *best)) {
        best = cost + *w;
      }
      return;
    }
    for (size_t v = 2; v < gtsp.vertices.size(); ++v) {
      if (clusters[v] != cluster_ids[i]) continue;
      auto w = weight(prev, static_cast<int>(v));
      if (!w) continue;
      choose(i + 1, static_cast<int>(v), cost + *w);
    }
  };
  do {
    choose(0, Gtsp::kStart, 0);
  } while (std::next_permutation(cluster_ids.begin(), cluster_ids.end()));
  return best;
}

void CheckTour(
    const Gtsp& gtsp,
    const std::vector<int>& clusters,
    const GtspSolution& solution
) {
  ASSERT_GE(solution.tour.size(), 2u);
  EXPECT_EQ(solution.tour.front(), Gtsp::kStart);
  EXPECT_EQ(solution.tour.back(), Gtsp::kEnd);
  std::vector<int> seen;
  int cost = 0;
  for (size_t k = 0; k + 1 < solution.tour.size(); ++k) {
    int a = solution.tour[k];
    int b = solution.tour[k + 1];
    auto it = std::find_if(
        gtsp.edges.begin(),
        gtsp.edges.end(),
        [&](const GtspEdge& e) { return e.from == a && e.to == b; }
    );
    ASSERT_NE(it, gtsp.edges.end()) << "missing edge " << a << " -> " << b;
    cost += it->weight_seconds;
    seen.push_back(clusters[a]);
  }
  seen.push_back(clusters[Gtsp::kEnd]);
  std::sort(seen.begin(), seen.end());
  std::vector<int> all(clusters);
  std::sort(all.begin(), all.end());
  all.erase(std::unique(all.begin(), all.end()), all.end());
  EXPECT_EQ(seen, all) << "tour must visit every cluster exactly once";
  EXPECT_EQ(cost, solution.cost_seconds);
}

TEST(GtspConcordeTest, TwoClustersPicksCheapestVertices) {
  // Stop 1 at times 100 and 200, stop 2 at 150 and 400.
  Gtsp gtsp = MakeGtsp(
      {{StopId{1}, TimeSinceServiceStart{100}},
       {StopId{1}, TimeSinceServiceStart{200}},
       {StopId{2}, TimeSinceServiceStart{150}},
       {StopId{2}, TimeSinceServiceStart{400}}},
      {{2, 4, 50}, {2, 5, 300}, {3, 5, 200}, {4, 3, 50}}
  );
  std::vector<int> clusters = ClustersByStop(gtsp);
  std::optional<GtspSolution> solution = SolveGtspWithConcorde(gtsp, clusters);
  ASSERT_TRUE(solution.has_value());
  CheckTour(gtsp, clusters, *solution);
  EXPECT_EQ(solution->cost_seconds, 50);
}

TEST(GtspConcordeTest, InfeasibleWhenClustersDisconnected) {
  Gtsp gtsp = MakeGtsp(
      {{StopId{1}, TimeSinceServiceStart{100}},
       {StopId{2}, TimeSinceServiceStart{150}},
       {StopId{3}, TimeSinceServiceStart{150}}},
      {{2, 3, 50}}
  );
  std::vector<int> clusters = ClustersByStop(gtsp);
  EXPECT_FALSE(SolveGtspWithConcorde(gtsp, clusters).has_value());
}

// Random scheduled-step graphs on a few stops, with the stops randomly
// grouped into clusters, large enough to go through Concorde rather than the
// brute-force path. (The encoding is exact for GTSPs from BuildGtsp, not for
// arbitrary GTSPs with time-difference weights: see gtsp_concorde.cpp.)
RC_GTEST_PROP(GtspConcordeTest, MatchesBruteForceOnRandomSteps, ()) {
  int num_stops = *rc::gen::inRange(2, 5);
  int num_clusters = *rc::gen::inRange(1, num_stops + 1);
  int num_steps = *rc::gen::inRange(1, 8);
  std::vector<Step> steps;
  for (int i = 0; i < num_steps; ++i) {
    int origin = *rc::gen::inRange(0, num_stops);
    int destination = (origin + *rc::gen::inRange(1, num_stops)) % num_stops;
    int departure = *rc::gen::inRange(0, 3000);
    int duration = *rc::gen::inRange(0, 1500);
    steps.push_back(Step::PrimitiveScheduled(
        StopId{origin},
        StopId{destination},
        TimeSinceServiceStart{departure},
        TimeSinceServiceStart{departure + duration},
        TripId{i}
    ));
  }
  ProblemBoundary boundary{
      .start = StopId{num_stops}, .end = StopId{num_stops + 1}
  };
  // Boundary stops must exist in the adjacency list; flex steps to and from
  // them are ignored by BuildGtsp anyway.
  for (int stop = 0; stop < num_stops; ++stop) {
    steps.push_back(Step::PrimitiveFlex(
        boundary.start, StopId{stop}, 0, TripId::NOOP
    ));
    steps.push_back(Step::PrimitiveFlex(
        StopId{stop}, boundary.end, 0, TripId::NOOP
    ));
  }
  std::optional<TimeSinceServiceStart> start_time;
  if (*rc::gen::inRange(0, 2) == 1) {
    start_time = TimeSinceServiceStart{*rc::gen::inRange(0, 3000)};
  }
  Gtsp gtsp = BuildGtsp(MakeAdjacencyList(steps), boundary, start_time);

  std::vector<int> cluster_of_stop;
  for (int stop = 0; stop < num_stops; ++stop) {
    cluster_of_stop.push_back(*rc::gen::inRange(0, num_clusters));
  }
  std::vector<int> clusters;
  for (const GtspVertex& v : gtsp.vertices) {
    if (v.stop == boundary.start) {
      clusters.push_back(num_clusters);
    } else if (v.stop == boundary.end) {
      clusters.push_back(num_clusters + 1);
    } else {
      clusters.push_back(cluster_of_stop[v.stop.v]);
    }
  }

  std::optional<int> expected = BruteForceGtsp(gtsp, clusters);
  std::optional<GtspSolution> solution = SolveGtspWithConcorde(gtsp, clusters);
  RC_ASSERT(solution.has_value() == expected.has_value());
  if (solution) {
    CheckTour(gtsp, clusters, *solution);
    RC_ASSERT(solution->cost_seconds == *expected);

    // Scheduling the optimal stop order along the GTSP gives the optimum
    // again, and so does solving with it as the starting tour.
    std::vector<StopId> stop_order;
    for (int v : solution->tour) {
      stop_order.push_back(gtsp.vertices[v].stop);
    }
    std::optional<GtspSolution> along = GtspTourAlongStops(gtsp, stop_order);
    RC_ASSERT(along.has_value());
    CheckTour(gtsp, clusters, *along);
    RC_ASSERT(along->cost_seconds == *expected);

    std::optional<GtspSolution> warm =
        SolveGtspWithConcorde(gtsp, clusters, along);
    RC_ASSERT(warm.has_value());
    CheckTour(gtsp, clusters, *warm);
    RC_ASSERT(warm->cost_seconds == *expected);
  }
}

// On problem states without flex steps the scheduled-steps GTSP over the
// completed graph is exactly the problem Held-Karp solves.
RC_GTEST_PROP(GtspConcordeTest, MatchesHeldKarpWithoutFlexSteps, ()) {
  ProblemState state =
      *GenProblemState(rc::gen::just(CycleIsFlex::kNo), std::nullopt);

  StepsAdjacencyList completed =
      MakeAdjacencyList(state.ComputeCompletedGraph().AllMergedSteps());
  Gtsp gtsp = BuildGtsp(completed, state.boundary);
  std::vector<int> clusters = GtspClustersFromRequired(gtsp, state.required);

  std::optional<GtspSolution> solution = SolveGtspWithConcorde(gtsp, clusters);
  HeldKarpDPResult hk = HeldKarpDPSolve(state, 0);

  if (hk.best_val == std::numeric_limits<int>::max()) {
    RC_ASSERT(!solution.has_value());
  } else {
    RC_ASSERT(solution.has_value());
    CheckTour(gtsp, clusters, *solution);
    RC_ASSERT(solution->cost_seconds == hk.best_val);
  }
}

}  // namespace
}  // namespace vats5
