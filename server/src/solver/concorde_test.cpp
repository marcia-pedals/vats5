#include "solver/concorde.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <unordered_set>

namespace vats5 {

// Test with a simple 4-node ATSP instance.
// Graph:
//   0 -> 1: 10
//   0 -> 2: 15
//   0 -> 3: 20
//   1 -> 0: 12
//   1 -> 2: 8
//   1 -> 3: 25
//   2 -> 0: 18
//   2 -> 1: 9
//   2 -> 3: 5
//   3 -> 0: 22
//   3 -> 1: 14
//   3 -> 2: 6
//
// The optimal tour should visit all 4 nodes exactly once.
// Let's compute: 0->1->2->3->0 = 10+8+5+22 = 45
//               0->2->3->1->0 = 15+5+14+12 = 46
//               0->1->3->2->0 = 10+25+6+18 = 59
//               0->3->2->1->0 = 20+6+9+12 = 47
//               0->2->1->3->0 = 15+9+25+22 = 71
//               0->3->1->2->0 = 20+14+8+18 = 60
// Best is 0->1->2->3->0 = 45
TEST(ConcordeTest, Simple4NodeATSP) {
  std::vector<WeightedEdge> edges = {
      {StopId{0}, StopId{1}, 10},
      {StopId{0}, StopId{2}, 15},
      {StopId{0}, StopId{3}, 20},
      {StopId{1}, StopId{0}, 12},
      {StopId{1}, StopId{2}, 8},
      {StopId{1}, StopId{3}, 25},
      {StopId{2}, StopId{0}, 18},
      {StopId{2}, StopId{1}, 9},
      {StopId{2}, StopId{3}, 5},
      {StopId{3}, StopId{0}, 22},
      {StopId{3}, StopId{1}, 14},
      {StopId{3}, StopId{2}, 6},
  };

  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(edges);

  std::optional<ConcordeSolution> solution = SolveTspWithConcorde(relaxed);
  ASSERT_TRUE(solution.has_value());

  // Verify tour visits all 4 nodes exactly once
  ASSERT_EQ(solution->tour.size(), 4);
  std::unordered_set<int> visited;
  for (const auto& stop : solution->tour) {
    visited.insert(stop.v);
  }
  EXPECT_EQ(visited.size(), 4);
  EXPECT_TRUE(visited.count(0));
  EXPECT_TRUE(visited.count(1));
  EXPECT_TRUE(visited.count(2));
  EXPECT_TRUE(visited.count(3));

  // Verify optimal value is 45
  EXPECT_EQ(solution->optimal_value, 45);
}

// Test with a 3-node ATSP instance where all edges have the same weight.
// Any tour should be optimal.
TEST(ConcordeTest, Simple3NodeSymmetric) {
  std::vector<WeightedEdge> edges = {
      {StopId{0}, StopId{1}, 10},
      {StopId{0}, StopId{2}, 10},
      {StopId{1}, StopId{0}, 10},
      {StopId{1}, StopId{2}, 10},
      {StopId{2}, StopId{0}, 10},
      {StopId{2}, StopId{1}, 10},
  };

  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(edges);

  std::optional<ConcordeSolution> solution = SolveTspWithConcorde(relaxed);
  ASSERT_TRUE(solution.has_value());

  // Verify tour visits all 3 nodes exactly once
  ASSERT_EQ(solution->tour.size(), 3);
  std::unordered_set<int> visited;
  for (const auto& stop : solution->tour) {
    visited.insert(stop.v);
  }
  EXPECT_EQ(visited.size(), 3);

  // Any tour should have cost 30
  EXPECT_EQ(solution->optimal_value, 30);
}

// Property test: verify that the sum of edge weights along the tour equals
// optimal_value.
RC_GTEST_PROP(ConcordeTest, TourCostMatchesOptimalValue, ()) {
  // Generate a complete directed graph with 2-5 nodes.
  int num_stops = *rc::gen::inRange(2, 6);

  // Generate edge weights for all pairs (complete graph ensures valid tour
  // exists). Weight constraint: must be < 32767 (Concorde limit).
  std::vector<WeightedEdge> edges;
  for (int from = 0; from < num_stops; ++from) {
    for (int to = 0; to < num_stops; ++to) {
      if (from != to) {
        int weight = *rc::gen::inRange(1, 1000);
        edges.push_back(WeightedEdge{StopId{from}, StopId{to}, weight});
      }
    }
  }

  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(edges);

  // Solve with Concorde.
  std::optional<ConcordeSolution> solution = SolveTspWithConcorde(relaxed);
  RC_ASSERT(solution.has_value());

  // Verify tour visits all nodes exactly once.
  RC_ASSERT(static_cast<int>(solution->tour.size()) == num_stops);
  std::unordered_set<int> visited;
  for (const auto& stop : solution->tour) {
    visited.insert(stop.v);
  }
  RC_ASSERT(static_cast<int>(visited.size()) == num_stops);

  // Compute the sum of edge weights along the tour.
  int tour_cost = 0;
  for (int i = 0; i < num_stops; ++i) {
    StopId from = solution->tour[i];
    StopId to = solution->tour[(i + 1) % num_stops];
    tour_cost += relaxed.GetWeight(from, to).value();
  }

  // Verify that tour cost matches optimal value.
  RC_ASSERT(tour_cost == solution->optimal_value);
}

// Test with a 3-node ATSP that has no valid tour.
// Graph:
//   0 -> 1: 10
//   1 -> 2: 10
//   2 -> 1: 10
// Missing edges back to 0, so no Hamiltonian cycle exists.
// Concorde will use forbidden edges to complete the tour, so we return nullopt.
TEST(ConcordeTest, NoValidTour) {
  std::vector<WeightedEdge> edges = {
      {StopId{0}, StopId{1}, 10},
      {StopId{1}, StopId{2}, 10},
      {StopId{2}, StopId{1}, 10},
  };

  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(edges);

  std::optional<ConcordeSolution> solution = SolveTspWithConcorde(relaxed);

  // Since there's no valid tour, we should get nullopt.
  EXPECT_FALSE(solution.has_value());
}

// Property test: a graph with edges only along a permutation has exactly one
// valid tour.
RC_GTEST_PROP(ConcordeTest, UniquePermutationTour, ()) {
  // Generate a random permutation of 5-10 vertices.
  int num_stops = *rc::gen::inRange(5, 11);

  // Generate a random permutation using Fisher-Yates shuffle.
  std::vector<int> perm(num_stops);
  for (int i = 0; i < num_stops; ++i) {
    perm[i] = i;
  }
  for (int i = num_stops - 1; i > 0; --i) {
    int j = *rc::gen::inRange(0, i + 1);
    std::swap(perm[i], perm[j]);
  }

  // Build a graph with edges only along the permutation.
  // Edge from perm[i] to perm[(i+1) % n] with weight 1.
  std::vector<WeightedEdge> edges;
  for (int i = 0; i < num_stops; ++i) {
    int from = perm[i];
    int to = perm[(i + 1) % num_stops];
    edges.push_back(WeightedEdge{StopId{from}, StopId{to}, 1});
  }

  RelaxedAdjacencyList relaxed = MakeRelaxedAdjacencyListFromEdges(edges);

  // Solve with Concorde.
  std::optional<ConcordeSolution> solution = SolveTspWithConcorde(relaxed);
  RC_ASSERT(solution.has_value());

  // Verify tour has the correct size.
  RC_ASSERT(static_cast<int>(solution->tour.size()) == num_stops);

  // The solution tour should be a rotation of the original permutation.
  // Find where perm[0] appears in the solution tour.
  int rotation_offset = -1;
  for (int i = 0; i < num_stops; ++i) {
    if (solution->tour[i].v == perm[0]) {
      rotation_offset = i;
      break;
    }
  }
  RC_ASSERT(rotation_offset != -1);

  // Verify the tour matches the permutation (as a cycle).
  for (int i = 0; i < num_stops; ++i) {
    int expected = perm[i];
    int actual = solution->tour[(rotation_offset + i) % num_stops].v;
    RC_ASSERT(expected == actual);
  }

  // Verify optimal value is num_stops (each edge has weight 1).
  RC_ASSERT(solution->optimal_value == num_stops);
}

}  // namespace vats5
