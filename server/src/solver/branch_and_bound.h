#pragma once

#include <functional>
#include <memory>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

#include "solver/data.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct ConstraintRequireEdge {
  StopId a;
  StopId b;
};

struct ConstraintForbidEdge {
  StopId a;
  StopId b;
};

using ProblemConstraint =
    std::variant<ConstraintRequireEdge, ConstraintForbidEdge>;

struct BranchEdge {
  StopId a;
  StopId b;

  bool operator==(const BranchEdge& other) const = default;

  ConstraintRequireEdge Require() const { return ConstraintRequireEdge{a, b}; }

  ConstraintForbidEdge Forbid() const { return ConstraintForbidEdge{a, b}; }
};

// The minimal amount of information needed to reconstruct the entire state of a
// search node from the initial problem.
struct SearchEdge {
  // Additional constraints added to the parent, in order.
  std::vector<ProblemConstraint> constraints;

  // This edge's parent's edge, as index of the search's `search_edges`. (-1 for
  // children of the root node).
  int parent_edge_index;
};

struct SearchNode {
  // The lb computed on the parent problem. Used for priority queue.
  int parent_lb;

  // The edge deriving this node from its parent, as index of the search's
  // `search_edges`. -1 for the root node.
  int edge_index;

  // State computed from the initial problem and the edges.
  //
  // Stored for active nodes so that we don't have to recompute everything from
  // the initial problem every time.
  //
  // Not stored for finished nodes because it's big and we don't want to keep it
  // around after we're done with it.
  std::unique_ptr<ProblemState> state;

  bool operator<(const SearchNode& other) const {
    if (parent_lb == other.parent_lb) {
      return edge_index > other.edge_index;
    }
    return parent_lb > other.parent_lb;
  }
};

ProblemState ApplyConstraints(
    const ProblemState& state, const std::vector<ProblemConstraint>& constraints
);

struct BranchAndBoundResult {
  int best_ub;
  std::vector<Path> best_paths;
  // original_edges from the state that produced best_paths, needed to expand
  // combined stops back to original stop IDs.
  std::unordered_map<StopId, PlainEdge> original_edges;
};

enum class BnbLogTag {
  kNode,        // iteration start: node taken from queue
  kInfeasible,  // node is infeasible
  kPruned,      // node pruned
  kLowerBound,  // lower bound computed + edges
  kUpperBound,  // upper bound path / new best found
  kPrimitive,   // primitive step sequence
  kQueuePrune,  // nodes pruned from queue
  kTerminated,  // search terminated early
  kConcorde,    // pass-through from TSP solver
};

using BnbLogger =
    std::function<void(int iteration, BnbLogTag tag, std::string_view message)>;

inline BnbLogger OstreamBnbLogger(std::ostream& os) {
  return [&os](int, BnbLogTag, std::string_view msg) { os << msg << "\n"; };
}

BranchAndBoundResult BranchAndBoundSolve(
    const ProblemState& initial_state,
    const BnbLogger& logger,
    int max_iter = -1
);

}  // namespace vats5

template <>
struct std::hash<vats5::BranchEdge> {
  std::size_t operator()(const vats5::BranchEdge& e) const noexcept {
    std::size_t h1 = std::hash<int>{}(e.a.v);
    std::size_t h2 = std::hash<int>{}(e.b.v);
    return h1 ^ (h2 << 1);
  }
};
