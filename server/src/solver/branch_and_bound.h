#pragma once

#include <functional>
#include <memory>
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

using ProblemConstraint = std::variant<ConstraintRequireEdge, ConstraintForbidEdge>;

struct BranchEdge {
  StopId a;
  StopId b;

  bool operator==(const BranchEdge& other) const = default;

  ConstraintRequireEdge Require() const {
    return ConstraintRequireEdge{a, b};
  }

  ConstraintForbidEdge Forbid() const {
    return ConstraintForbidEdge{a, b};
  }
};

// The minimal amount of information needed to reconstruct the entire state of a
// search node from the initial problem.
struct SearchEdge {
  // Additional constraints added to the parent, in order.
  std::vector<ProblemConstraint> constraints;

  // This edge's parent's edge. (so nullptr for children of the root node).
  // Owned by the search.
  SearchEdge* parent_edge;
};

struct SearchNode {
  // The lb computed on the parent problem. Used for priority queue.
  int parent_lb;

  // The edge deriving this node from its parent.
  // nullptr for the root node.
  // Owned by the search.
  SearchEdge* edge;

  // State computed from the initial problem and the edges.
  //
  // Stored for active nodes so that we don't have to recompute everything from
  // the initial problem every time.
  //
  // Not stored for finished nodes because it's big and we don't want to keep it
  // around after we're done with it.
  std::unique_ptr<ProblemState> state;
};

ProblemState ApplyConstraints(
  const ProblemState& state,
  const std::vector<ProblemConstraint>& constraints
);

// Stub function for branch and bound solver.
// Returns 0 for now.
int BranchAndBoundSolve();

}  // namespace vats5

template <>
struct std::hash<vats5::BranchEdge> {
  std::size_t operator()(const vats5::BranchEdge& e) const noexcept {
    std::size_t h1 = std::hash<int>{}(e.a.v);
    std::size_t h2 = std::hash<int>{}(e.b.v);
    return h1 ^ (h2 << 1);
  }
};
