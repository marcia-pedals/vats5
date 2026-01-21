#pragma once

#include <memory>
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

struct BranchRequireEdge {
  StopId a;
  StopId b;
};

struct BranchForbidEdge {
  StopId a;
  StopId b;
};

// The minimal amount of information needed to reconstruct the entire state of a
// search node from the initial problem.
struct SearchEdge {
  // Additional constraints added to the parent, in order.
  std::vector<std::variant<BranchRequireEdge, BranchForbidEdge>> constraints;

  // This edge's parent's edge. (so nullptr for children of the root node).
  // Owned by the search.
  SearchEdge* parent_edge;
};

struct ProblemBoundary {
  StopId start;
  StopId end;
};

struct ProblemState {
  // The graph of minimal steps, i.e. the steps from which all possible tours
  // can be made, with the property that deleting one step will make at least
  // one tour impossible.
  StepsAdjacencyList minimal;

  // The completion of `minimal`, i.e. every possible route between elements of
  // `stops` is a path.
  StepPathsAdjacencyList completed;

  // Which stops in `minimal`/`completed` are the START and END.
  ProblemBoundary boundary;

  // All stops that are required to be visited, including START and END.
  std::unordered_set<StopId> stops;

  // Names of all the stops for display purposes.
  std::unordered_map<StopId, std::string> stop_names;
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

// Stub function for branch and bound solver.
// Returns 0 for now.
int BranchAndBoundSolve();

}  // namespace vats5
