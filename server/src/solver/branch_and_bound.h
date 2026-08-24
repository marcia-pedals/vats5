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

// Tour-level succession constraints (see SuccessionConstraints in
// tarel_graph.h). Unlike the edge constraints above, these are not applied to
// the minimal graph by ApplyConstraints; they are carried on the search node
// and enforced when computing the tarel lower bound.
struct ConstraintRequireSuccession {
  StopId a;
  StopId b;

  std::string Debug(const ProblemState& state) const;
};

struct ConstraintForbidSuccession {
  StopId a;
  StopId b;

  std::string Debug(const ProblemState& state) const;
};

using ProblemConstraint = std::variant<
    ConstraintRequireEdge,
    ConstraintForbidEdge,
    ConstraintRequireSuccession,
    ConstraintForbidSuccession>;

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

  // State computed from the initial problem and the edges.
  //
  // Stored for active nodes so that we don't have to recompute everything from
  // the initial problem every time.
  //
  // Not stored for finished nodes because it's big and we don't want to keep it
  // around after we're done with it.
  std::unique_ptr<ProblemState> state;

  // Accumulated tour-level succession constraints (from
  // ConstraintRequireSuccession/ConstraintForbidSuccession edges on the path
  // to this node). Enforced in the tarel lower bound, not in `state`.
  SuccessionConstraints successions;

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

  // Only populated when collect_optimal_paths is set: how many optimal-value
  // paths the search encountered (counting duplicates), and how many distinct
  // expanded stop sequences were among them.
  int64_t optimal_path_count = 0;
  int64_t unique_optimal_path_count = 0;
};

// When collect_optimal_paths is set, the search does not prune nodes whose
// lower bound equals the best upper bound; instead it keeps exploring them
// (only stopping once LB strictly exceeds UB) and collects every feasible path
// it finds whose duration equals the optimal value, reporting total and unique
// counts. Useful for diagnosing symmetric / near-isomorphic search nodes.
// When succession_branching is set, the search branches on required-stop
// successions of the LB tour (require: the tour leg leaving `a` goes to `b`;
// forbid: it doesn't) instead of on individual minimal-graph steps. This makes
// each branch a true partition of the tour space.
BranchAndBoundResult BranchAndBoundSolve(
    const ProblemState& initial_state,
    std::ostream* search_log,
    std::optional<std::string> run_dir = std::nullopt,
    int max_iter = -1,
    const SearchEventCallback& on_event = nullptr,
    bool collect_optimal_paths = false,
    bool succession_branching = false
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
