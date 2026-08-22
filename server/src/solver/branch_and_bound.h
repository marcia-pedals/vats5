#pragma once

#include <functional>
#include <limits>
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

  // State computed from the initial problem and the edges.
  //
  // Stored for active nodes so that we don't have to recompute everything from
  // the initial problem every time.
  //
  // Not stored for finished nodes because it's big and we don't want to keep it
  // around after we're done with it.
  std::unique_ptr<ProblemState> state;

  // How many lazily added required stops (see LazyRequiredStops) this node's
  // state already includes. Stops added after that are injected when the node
  // is popped.
  size_t lazy_version = 0;

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

// Warm-start memory for a solve: a known-feasible incumbent seeding the upper
// bound, and a proven lower bound on this problem's optimum (e.g. the previous
// problem's optimum, when this problem only adds requirements to it). The
// search returns as soon as best_ub <= lb_floor: the floor proves the
// incumbent optimal.
struct SearchSeeds {
  int initial_ub = std::numeric_limits<int>::max();
  std::vector<Path> initial_ub_paths;
  int lb_floor = 0;
};

// Coverage-driven lazy growth of the required set ("branch and cut"): the
// search starts from `initial_state.required` and adds stops mid-search
// instead of the caller re-solving with a bigger required set.
struct LazyRequiredStops {
  // Called when the search is about to accept a new incumbent. `tour` is the
  // candidate's stop sequence in original (unfused) stop ids, START..END,
  // `duration_seconds` its duration, and `node_state` the violating node's
  // problem (constraints applied), usable e.g. to trial candidate additions.
  // Returns the stops to add to the required set -- one group, its
  // representative first -- or empty to accept the candidate. Rejected
  // candidates do not become incumbents; the added stops join every node's
  // required set as the nodes are popped, which is sound because a lower
  // bound computed with fewer required stops is still a lower bound with
  // more.
  std::function<std::vector<StopId>(
      const std::vector<StopId>& tour,
      int duration_seconds,
      const ProblemState& node_state
  )>
      on_candidate;
};

BranchAndBoundResult BranchAndBoundSolve(
    const ProblemState& initial_state,
    std::ostream* search_log,
    std::optional<std::string> run_dir = std::nullopt,
    int max_iter = -1,
    const SearchEventCallback& on_event = nullptr,
    const LazyRequiredStops* lazy = nullptr,
    const SearchSeeds* seeds = nullptr
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
