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

// Restricts the arrival times the tour may use at `stop`. With `keep`, only
// the scheduled minimal steps arriving at `stop` at one of `times` survive;
// every flex step into `stop` (a walk in, or the START->stop step that lets
// the tour start there) is removed too, since a flex arrival happens at an
// arbitrary time. Without `keep`, exactly the scheduled steps arriving at one
// of `times` are removed and flex steps are untouched.
//
// The pair is a valid dichotomy: a tour whose visit to `stop` arrives at one
// of `times` by a scheduled step is in the first branch, and every other tour
// is in the second, including tours that walk in or start at `stop`, because
// the second branch deletes no flex step. Dropping the flex arrivals in the
// first branch is what lets NarrowArrivalTimes pin the tour to the window
// around `times`. A tour may pass through `stop` more than once, so each
// branch is a relaxation of the intended child, which keeps the bounds valid.
struct ConstraintArrivalTimes {
  StopId stop;
  // Sorted ascending.
  std::vector<TimeSinceServiceStart> times;
  bool keep;

  std::string Debug(const ProblemState& state) const;
};

using ProblemConstraint = std::variant<
    ConstraintRequireEdge,
    ConstraintForbidEdge,
    ConstraintArrivalTimes>;

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

struct ArrivalWindowNarrowing {
  ProblemState state;

  // Every step of a tour of duration <= ub departs at or after `earliest` and
  // arrives at or before `latest`.
  TimeSinceServiceStart earliest;
  TimeSinceServiceStart latest;

  int num_steps_removed;
  int num_rounds;
};

// Narrows `state` to the steps a tour of duration at most `ub_seconds` can
// use.
//
// A tour visits some stop x of every required group as an endpoint of its
// completed-graph paths. That visit either arrives at x by a scheduled
// completed path (possibly one from START, i.e. the tour started elsewhere
// and rode to x), or (if an all-flex START->x path exists) the tour starts at
// x and leaves it by a scheduled completed path (a tour that both starts and
// ends at x uses no scheduled step at all and is unaffected by narrowing). So
// the tour is at x at a time within the range of x's scheduled completed-path
// arrival times, widened by its scheduled departure times when the tour can
// start at x, and hence lies entirely within that range widened by ub on both
// sides. Intersecting over all groups gives a window outside of which no
// scheduled minimal step can be used. Removing those steps can shrink the
// ranges, so this repeats until nothing changes. A stop with a flex completed
// path in from a stop other than START (or, when the tour can start there,
// out to a stop other than END) could be visited at any time and does not
// constrain the window, nor does its group; flex paths longer than ub are
// ignored since no tour within ub can use them.
//
// Returns nullopt if the window becomes empty, i.e. no tour of duration <= ub
// exists.
std::optional<ArrivalWindowNarrowing> NarrowArrivalTimes(
    const ProblemState& state, int ub_seconds
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
    int known_lb,
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
