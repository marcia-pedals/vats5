#include "solver/branch_and_bound.h"
#include <sys/stat.h>
#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
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
        return s.origin.stop == forbid.a && s.destination.stop == forbid.b;
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
      assert(!(boundary.start == require.a && boundary.end == require.b));
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
        if (s.origin.stop == require.a && s.destination.stop == require.b) {
          a_to_b.push_back(s);
        }
        if (s.destination.stop == require.a) {
          star_to_a[s.origin.stop].push_back(s);
        }
        if (s.origin.stop == require.b) {
          b_to_star.push_back(s);
        }
      }

      // The steps to "ab" are the steps "x->a" merged with the steps "a->b".
      for (const auto& [x, x_to_a] : star_to_a) {
        std::vector<Step> x_to_ab = PairwiseMergedSteps(x_to_a, a_to_b);
        for (Step s : x_to_ab) {
          s.destination.stop = ab;
          steps.push_back(s);
        }
      }

      // The steps from "ab" are the steps "b->x".
      for (Step s : b_to_star) {
        s.origin.stop = ab;
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

int BranchAndBoundSolve(
  const ProblemState& initial_state,
  std::ostream* search_log,
  int max_iter
) {
  std::vector<SearchEdge> search_edges;
  std::vector<SearchNode> q;
  q.push_back(SearchNode{
    .parent_lb = 0,
    .edge_index = -1,
    .state = std::make_unique<ProblemState>(initial_state),
  });

  auto PushQ = [&search_edges, &q](const ProblemState& state, int new_lb, SearchEdge new_edge) {
    int new_edge_index = search_edges.size();
    search_edges.push_back(new_edge);
    // TODO: Figure out if passing ApplyConstraints to std::make_unique does the smart thing or not.
    std::unique_ptr<ProblemState> new_state = std::make_unique<ProblemState>(ApplyConstraints(state, new_edge.constraints));
    q.push_back(std::move(SearchNode{new_lb, new_edge_index, std::move(new_state)}));
    std::push_heap(q.begin(), q.end());
  };

  int iter_num = 0;
  int best_ub = std::numeric_limits<int>::max();

  while (!q.empty()) {
    if (max_iter > 0 && iter_num >= max_iter) {
      throw std::runtime_error("Exceeded max_iter");
    }
    iter_num += 1;

    std::pop_heap(q.begin(), q.end());
    SearchNode cur_node = std::move(q.back());
    ProblemState& state = *cur_node.state;
    q.pop_back();

    if (search_log != nullptr) {
      *search_log << "Take " << cur_node.parent_lb;
      if (cur_node.edge_index != -1) {
        for (auto c : search_edges[cur_node.edge_index].constraints) {
          if (std::holds_alternative<ConstraintForbidEdge>(c)) {
            auto f = std::get<ConstraintForbidEdge>(c);
            *search_log << " [forbid " << state.StopName(f.a) << " -> " << state.StopName(f.b) << "]";
          } else {
            auto r = std::get<ConstraintRequireEdge>(c);
            *search_log << " [require " << state.StopName(r.a) << " -> " << state.StopName(r.b) << "]";
          }
        }
      }
      *search_log << "\n";
      // TODO: Detailed log level so that we can print these out sometimes.
      // showValue(state, *search_log);
      // *search_log << "\n";
    }

    if (cur_node.parent_lb >= best_ub) {
      if (search_log != nullptr) {
        *search_log << "Search terminated: LB >= UB\n";
      }
      return best_ub;
    }

    // Compute lower bound.
    std::optional<TspTourResult> lb_result_opt = ComputeTarelLowerBound(state);
    if (!lb_result_opt.has_value()) {
      // Infeasible node!
      if (search_log != nullptr) {
        *search_log << "  infeasible\n";
      }
      continue;
    }
    TspTourResult& lb_result = lb_result_opt.value();
    if (lb_result.optimal_value >= best_ub) {
      // Pruned node!
      if (search_log != nullptr) {
        *search_log << "  pruned: LB (" << lb_result.optimal_value << ") >= UB (" << best_ub << ")\n";
      }
      continue;
    }
    if (search_log != nullptr) {
      *search_log << "  lb: " << lb_result.optimal_value << "\n";
    }

    // Make an upper bound by actually following the LB path.
    std::vector<Step> feasible_steps;
    feasible_steps.push_back(Step::PrimitiveFlex(state.boundary.start, state.boundary.start, 0, TripId::NOOP));
    for (int i = 0; i < lb_result.tour_edges.size(); ++i) {
      TarelEdge& edge = lb_result.tour_edges[i];
      std::vector<Step> next_steps;
      for (const Path& p : state.completed.PathsBetween(edge.origin.stop, edge.destination.stop)) {
        next_steps.push_back(p.merged_step);
      }
      feasible_steps = PairwiseMergedSteps(feasible_steps, next_steps);
    }
    // TODO: Reference thing about 00:00:00.
    std::erase_if(feasible_steps, [](const Step& step) {
      return step.origin.time < TimeSinceServiceStart{0};
    });
    auto best_feasible_step_it = std::min_element(feasible_steps.begin(), feasible_steps.end(), [](const Step& a, const Step& b) {
      return a.DurationSeconds() < b.DurationSeconds();
    });
    if (best_feasible_step_it != feasible_steps.end() && best_feasible_step_it->DurationSeconds() < best_ub) {
      best_ub = best_feasible_step_it->DurationSeconds();
      if (search_log != nullptr) {
        *search_log << "  found new ub " << best_ub << " " << best_feasible_step_it->origin.time.ToString() << " " << best_feasible_step_it->destination.time.ToString() << "\n";
        for (int i = 0; i < lb_result.tour_edges.size(); ++i) {
          TarelEdge& edge = lb_result.tour_edges[i];
          *search_log << "    " << state.StopName(edge.origin.stop) << " -> " << state.StopName(edge.destination.stop) << "\n";
        }
      }
    }

    std::vector<Step> primitive_steps;
    for (const TarelEdge& e : lb_result.tour_edges) {
      const auto& paths = state.completed.PathsBetween(e.origin.stop, e.destination.stop);
      assert(paths.size() > 0);
      Path best = paths[0];
      for (const Path& p : paths) {
        if (p.DurationSeconds() < best.DurationSeconds()) {
          best = p;
        }
      }
      for (const Step& s : best.steps) {
        primitive_steps.push_back(s);
      }
    }

    if (primitive_steps.size() <= 2) {
      // TODO: Figure out what to do here.
      if (search_log != nullptr) {
        *search_log << "  pruned: 2 or fewer primitive steps\n";
      }
      continue;
    }

    // Select branch edge by hashing edges on LB path.
    size_t edge_hash = 0;
    for (const Step& s : primitive_steps) {
      edge_hash ^= std::hash<int>{}(s.origin.stop.v) * 31 + std::hash<int>{}(s.destination.stop.v);
    }
    // TODO: Make it possible to select START -> * and * -> END steps.
    // Ok so the problem is that the path is allowed to "start at the start" for zero cost even if e.g. the start is actually like "START->(a->b)" which should incur the "(a->b)" cost.
    // I think that having an ACTUAL_START which is not allowed to be merged would probably fix this. But this would incur an extra vertex cost? Is this the only way to "branch on requiring a certain start"?
    // Alternatively we could track "start cost".
    // Everything in parallel with END of course.
    Step& branch_step = primitive_steps[(edge_hash % (primitive_steps.size() - 2)) + 1];
    // Step& branch_step = primitive_steps[edge_hash % primitive_steps.size()];
    BranchEdge branch_edge_fw{branch_step.origin.stop, branch_step.destination.stop};
    BranchEdge branch_edge_rv{branch_step.destination.stop, branch_step.origin.stop};

    // Make and push search nodes for branches.
    PushQ(state, lb_result.optimal_value, SearchEdge{{branch_edge_fw.Require()}, cur_node.edge_index});
    PushQ(state, lb_result.optimal_value, SearchEdge{{branch_edge_fw.Forbid(), branch_edge_rv.Require()}, cur_node.edge_index});
    PushQ(state, lb_result.optimal_value, SearchEdge{{branch_edge_fw.Forbid(), branch_edge_rv.Forbid()}, cur_node.edge_index});
  }

  return best_ub;
}

}  // namespace vats5
