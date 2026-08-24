#pragma once

#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "solver/data.h"
#include "solver/search_event.h"
#include "solver/tarel_graph.h"

namespace vats5 {

struct ConstraintRequireEdge {
  StopId a;
  StopId b;

  std::string Debug(const ProblemState& state) const;
};

struct ConstraintForbidEdge {
  StopId a;
  StopId b;

  std::string Debug(const ProblemState& state) const;
};

using ProblemConstraint =
    std::variant<ConstraintRequireEdge, ConstraintForbidEdge>;

std::string Debug(const ProblemConstraint& c, const ProblemState& state);

struct BranchEdge {
  StopId a;
  StopId b;

  bool operator==(const BranchEdge& other) const = default;

  std::string Debug(const ProblemState& state) const;

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

  // The node's ProblemState is NOT stored: with thousands of active nodes the
  // states dominate memory. It is recomputed from the initial problem by
  // replaying the constraint chain (via `edge_index`) when the node is popped.

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

BranchAndBoundResult BranchAndBoundSolve(
    const ProblemState& initial_state,
    std::ostream* search_log,
    std::optional<std::string> run_dir = std::nullopt,
    int max_iter = -1,
    const SearchEventCallback& on_event = nullptr
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
