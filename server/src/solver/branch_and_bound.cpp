#include "solver/branch_and_bound.h"
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"

namespace vats5 {


ProblemState ApplyConstraints(
  const ProblemState& state,
  const std::vector<ProblemConstraint>& constraints
) {
  // Mutable copies of all the `ProblemState` fields we'll be mutating.
  std::vector<Step> steps = state.minimal.AllSteps();
  ProblemBoundary boundary = state.boundary;
  std::unordered_set<StopId> required_stops = state.required_stops;
  std::unordered_map<StopId, std::string> stop_names = state.stop_names;
  StopId next_stop_id{state.minimal.NumStops()};

  // Apply constraints in order, by mutating the copies that we just made above.
  for (const ProblemConstraint& constraint : constraints) {
    if (std::holds_alternative<ConstraintForbidEdge>(constraint)) {
      const ConstraintForbidEdge& forbid = std::get<ConstraintForbidEdge>(constraint);
      // Forbid is super simple. We just erase all minimal steps a->b.
      std::erase_if(steps, [&](const Step& s) -> bool {
        return s.origin_stop == forbid.a && s.destination_stop == forbid.b;
      });
    } else if (std::holds_alternative<ConstraintRequireEdge>(constraint)) {
      const ConstraintRequireEdge& require = std::get<ConstraintRequireEdge>(constraint);
      // Require is a bit more complicated. At a high level, we add a new stop
      // "a->b" to the graph representing the act of going to a and then
      // proceeding to b along the required edge. We make "ab" a required stop
      // and drop the requirements to visit the original "a" and "b". But we
      // still keep them around so that they can be used as intermediate stops
      // in other routes.
      //
      // Note: Alternatively, we could actually completely drop "a" and/or "b"
      // from the graph and add combinations of steps that went through them.
      // This would reduce the graph size so it might be good. But I think it
      // might end up being the same amount of computation because we added a
      // bunch more steps to compensate. And it's more complicated and makes
      // interpreting the paths harder.
      StopId ab = next_stop_id;
      next_stop_id.v += 1;
      stop_names[ab] = "(" + stop_names[require.a] + "->" + stop_names[require.b] + ")";
      required_stops.insert(ab);
      required_stops.erase(require.a);
      required_stops.erase(require.b);
      assert(boundary.start != require.b);
      assert(boundary.end != require.a);
      if (boundary.start == require.a) {
        boundary.start = ab;
      }
      if (boundary.end == require.b) {
        boundary.end = ab;
      }

      // Collect some steps that we'll need for constructing the steps to and from "ab".
      // Steps from a to b.
      std::vector<Step> a_to_b;
      // Steps from * to a, grouped by *.
      std::unordered_map<StopId, std::vector<Step>> star_to_a;
      // Steps from b to *, not grouped by anything.
      std::vector<Step> b_to_star;
      for (const Step& s : steps) {
        if (s.origin_stop == require.a && s.destination_stop == require.b) {
          a_to_b.push_back(s);
        }
        if (s.destination_stop == require.a) {
          star_to_a[s.origin_stop].push_back(s);
        }
        if (s.origin_stop == require.b) {
          b_to_star.push_back(s);
        }
      }

      // The steps to "ab" are the steps "x->a" merged with the steps "a->b".
      for (const auto& [x, x_to_a] : star_to_a) {
        std::vector<Step> x_to_ab = PairwiseMergedSteps(x_to_a, a_to_b);
        for (Step s : x_to_ab) {
          s.destination_stop = ab;
          steps.push_back(s);
        }
      }

      // The steps from "ab" are the steps "b->x".
      for (Step s : b_to_star) {
        s.origin_stop = ab;
        steps.push_back(s);
      }
    } else {
      assert(false);
    }
  }

  // Build the new problem state from the stuff we've been mutating.
  return MakeProblemState(
    MakeAdjacencyList(steps), std::move(boundary), std::move(required_stops), std::move(stop_names)
  );
}

int BranchAndBoundSolve() {
  return 0;
}

}  // namespace vats5
