#include "solver/branch_and_bound.h"

#include <algorithm>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"

namespace vats5 {

ProblemState ApplyConstraints(
    const ProblemState& state, const std::vector<ProblemConstraint>& constraints
) {
  // Mutable copies of all the `ProblemState` fields we'll be mutating.
  std::vector<Step> steps = state.minimal.AllSteps();
  ProblemBoundary boundary = state.boundary;
  std::unordered_set<StopId> required_stops = state.required_stops;
  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos = state.stop_infos;
  std::unordered_map<StopId, PlainEdge> original_edges = state.original_edges;
  StopId next_stop_id{state.minimal.NumStops()};

  // Apply constraints in order, by mutating the copies that we just made above.
  for (const ProblemConstraint& constraint : constraints) {
    if (std::holds_alternative<ConstraintForbidEdge>(constraint)) {
      const ConstraintForbidEdge& forbid =
          std::get<ConstraintForbidEdge>(constraint);
      // Forbid is super simple. We just erase all minimal steps a->b.
      std::erase_if(steps, [&](const Step& s) -> bool {
        return s.origin.stop == forbid.a && s.destination.stop == forbid.b;
      });
    } else if (std::holds_alternative<ConstraintRequireEdge>(constraint)) {
      const ConstraintRequireEdge& require =
          std::get<ConstraintRequireEdge>(constraint);
      // Require is a bit more complicated. At a high level, we add a new stop
      // "ab" to the graph representing the act of going to a and then
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
      stop_infos[ab] = ProblemStateStopInfo{
          GtfsStopId{""},
          "(" + stop_infos[require.a].stop_name + "->" + stop_infos[require.b].stop_name + ")"
      };
      required_stops.insert(ab);
      required_stops.erase(require.a);
      required_stops.erase(require.b);
      assert(!(boundary.start == require.a && boundary.end == require.b));
      if (boundary.start == require.a) {
        boundary.start = ab;
      }
      if (boundary.end == require.b) {
        boundary.end = ab;
      }

      original_edges[ab] = PlainEdge{require.a, require.b};

      // Collect some steps that we'll need for constructing the steps to and
      // from "ab". Steps from a to b.
      std::vector<Step> a_to_b;
      // Steps from * to a, grouped by *.
      std::unordered_map<StopId, std::vector<Step>> star_to_a;
      // Steps from b to *, grouped by *.
      std::unordered_map<StopId, std::vector<Step>> b_to_star;
      for (const Step& s : steps) {
        if (s.origin.stop == require.a && s.destination.stop == require.b) {
          a_to_b.push_back(s);
        }
        if (s.destination.stop == require.a) {
          star_to_a[s.origin.stop].push_back(s);
        }
        if (s.origin.stop == require.b) {
          b_to_star[s.destination.stop].push_back(s);
        }
      }

      // There are 2 things that "ab" could represent:
      // 1. You are at "a" and you're gonna leave via "b".
      // 2. You are at "b" and you've arrived via "a".
      //
      // The choice is arbitrary except for: If "a" is `boundary.start`, then
      // (2) does not work because we make "ab" into the new `boundary.start`,
      // and we allow you to start at `boundary.start` without incurring any
      // cost, so (2) would allow you to arrive at "b" via "a" without incurring
      // any cost. Similarly, if "b" is `boundary.end`, then (1) does not work.
      //
      // Therefore, we arbitrarily choose to do (1) when "a" is `boundary.start`
      // and (2) otherwise.
      if (boundary.start == ab) {
        // The steps to "ab" are the steps "x->a".
        for (const auto& [x, x_to_a] : star_to_a) {
          for (Step s : x_to_a) {
            s.destination.stop = ab;
            steps.push_back(s);
          }
        }
        // The steps from "ab" are the steps "a->b" merged with the steps
        // "b->x".
        for (const auto& [x, b_to_x] : b_to_star) {
          std::vector<Step> ab_to_x = PairwiseMergedSteps(a_to_b, b_to_x);
          for (Step s : ab_to_x) {
            s.origin.stop = ab;
            steps.push_back(s);
          }
        }
      } else {
        // The steps to "ab" are the steps "x->a" merged with the steps "a->b".
        for (const auto& [x, x_to_a] : star_to_a) {
          std::vector<Step> x_to_ab = PairwiseMergedSteps(x_to_a, a_to_b);
          for (Step s : x_to_ab) {
            s.destination.stop = ab;
            steps.push_back(s);
          }
        }
        // The steps from "ab" are the steps "b->x".
        for (const auto& [x, b_to_x] : b_to_star) {
          for (Step s : b_to_x) {
            s.origin.stop = ab;
            steps.push_back(s);
          }
        }
      }

      // TODO TODO: Figure out if this is ok. And explain why it's ok and why
      // it's important.
      std::erase_if(steps, [&](const Step& s) -> bool {
        if (s.destination.stop != require.b) {
          return false;
        }
        StopId erase_from = require.a;
        while (true) {
          if (s.origin.stop == erase_from) {
            return true;
          }
          auto it = original_edges.find(erase_from);
          if (it == original_edges.end()) {
            return false;
          }
          erase_from = it->second.b;
        }
      });
    } else {
      assert(false);
    }
  }

  // Build the new problem state from the stuff we've been mutating.
  return MakeProblemState(
      MakeAdjacencyList(steps),
      std::move(boundary),
      std::move(required_stops),
      std::move(stop_infos),
      state.step_partition_names,
      original_edges
  );
}

BranchAndBoundResult BranchAndBoundSolve(
    const ProblemState& initial_state,
    const BnbLogger& logger,
    int max_iter
) {
  std::vector<SearchEdge> search_edges;
  std::vector<SearchNode> q;
  q.push_back(
      SearchNode{
          .parent_lb = 0,
          .edge_index = -1,
          .state = std::make_unique<ProblemState>(initial_state),
      }
  );

  auto PushQ = [&search_edges, &q](
                   const ProblemState& state, int new_lb, SearchEdge new_edge
               ) {
    int new_edge_index = search_edges.size();
    search_edges.push_back(new_edge);
    // TODO: Figure out if passing ApplyConstraints to std::make_unique does the
    // smart thing or not.
    std::unique_ptr<ProblemState> new_state = std::make_unique<ProblemState>(
        ApplyConstraints(state, new_edge.constraints)
    );
    q.push_back(
        std::move(SearchNode{new_lb, new_edge_index, std::move(new_state)})
    );
    std::push_heap(q.begin(), q.end());
  };

  int iter_num = 0;
  int best_ub = std::numeric_limits<int>::max();
  std::vector<Path> best_paths;
  std::unordered_map<StopId, PlainEdge> best_original_edges;

  while (!q.empty()) {
    if (max_iter > 0 && iter_num >= max_iter) {
      throw std::runtime_error("Exceeded max_iter");
    }
    iter_num += 1;

    std::pop_heap(q.begin(), q.end());
    SearchNode cur_node = std::move(q.back());
    ProblemState& state = *cur_node.state;
    q.pop_back();

    {
      std::ostringstream msg;
      msg << iter_num << " (" << q.size() + 1 << " active nodes) Take "
          << TimeSinceServiceStart{cur_node.parent_lb}.ToString();
      if (cur_node.edge_index != -1) {
        for (auto c : search_edges[cur_node.edge_index].constraints) {
          if (std::holds_alternative<ConstraintForbidEdge>(c)) {
            auto f = std::get<ConstraintForbidEdge>(c);
            msg << " [forbid " << state.StopName(f.a) << " -> "
                << state.StopName(f.b) << "]";
          } else {
            auto r = std::get<ConstraintRequireEdge>(c);
            msg << " [require " << state.StopName(r.a) << " -> "
                << state.StopName(r.b) << "]";
          }
        }
      }
      msg << " {cur " << cur_node.edge_index;
      if (cur_node.edge_index != -1) {
        msg << "; parent "
            << search_edges[cur_node.edge_index].parent_edge_index;
      }
      msg << "}";
      logger(iter_num, BnbLogTag::kNode, msg.str());
    }

    if (cur_node.parent_lb >= best_ub) {
      logger(iter_num, BnbLogTag::kTerminated, "Search terminated: LB >= UB");
      return {best_ub, std::move(best_paths), std::move(best_original_edges)};
    }

    // Compute lower bound.
    TextLogger tsp_log = [&logger, iter_num](std::string_view msg) {
      logger(iter_num, BnbLogTag::kConcorde, msg);
    };
    std::optional<TspTourResult> lb_result_opt = ComputeTarelLowerBound(
        state,
        best_ub < std::numeric_limits<int>::max() ? std::make_optional(best_ub)
                                                  : std::nullopt,
        tsp_log
    );
    if (!lb_result_opt.has_value()) {
      // Infeasible node!
      logger(iter_num, BnbLogTag::kInfeasible, "  infeasible");
      continue;
    }
    TspTourResult& lb_result = lb_result_opt.value();

    if (lb_result.optimal_value >= best_ub) {
      // Pruned node!
      logger(
          iter_num,
          BnbLogTag::kPruned,
          std::format(
              "  pruned: LB ({}) >= UB ({})",
              TimeSinceServiceStart{lb_result.optimal_value}.ToString(),
              TimeSinceServiceStart{best_ub}.ToString()
          )
      );
      continue;
    }
    {
      std::ostringstream msg;
      msg << "  lb: "
          << TimeSinceServiceStart{lb_result.optimal_value}.ToString() << "\n";
      msg << "  lb edges:";
      for (const TarelEdge& edge : lb_result.tour_edges) {
        msg << "\n    " << state.StopName(edge.origin.stop) << " -> "
            << state.StopName(edge.destination.stop)
            << " w=" << TimeSinceServiceStart{edge.weight}.ToString();
      }
      logger(iter_num, BnbLogTag::kLowerBound, msg.str());
    }

    // Make an upper bound by actually following the LB path.
    std::vector<StopId> stop_sequence;
    stop_sequence.push_back(lb_result.tour_edges[0].origin.stop);
    for (const auto& edge : lb_result.tour_edges) {
      stop_sequence.push_back(edge.destination.stop);
    }
    std::vector<Path> feasible_paths =
        ComputeMinimalFeasiblePathsAlong(stop_sequence, state.completed);
    if (feasible_paths.size() > 0) {
      const Path& feasible_path = *std::min_element(
          feasible_paths.begin(), feasible_paths.end(),
          [](const Path& a, const Path& b) {
            return a.DurationSeconds() < b.DurationSeconds();
          });
      {
        std::ostringstream msg;
        msg << "  ub path ("
            << TimeSinceServiceStart{feasible_path.DurationSeconds()}.ToString()
            << "): ";
        for (int i = 0; i < lb_result.tour_edges.size() - 1; ++i) {
          if (i > 0) {
            msg << " -> ";
          }
          TarelEdge& edge = lb_result.tour_edges[i];
          msg << state.StopName(edge.destination.stop);
        }
        logger(iter_num, BnbLogTag::kUpperBound, msg.str());
      }
      if (feasible_path.DurationSeconds() < best_ub) {
        best_ub = feasible_path.DurationSeconds();
        best_paths.clear();
        for (const Path& p : feasible_paths) {
          if (p.DurationSeconds() == best_ub) {
            best_paths.push_back(p);
          }
        }
        best_original_edges = state.original_edges;
        logger(
            iter_num,
            BnbLogTag::kUpperBound,
            std::format(
                "  found new ub {} {} {}",
                TimeSinceServiceStart{best_ub}.ToString(),
                feasible_path.merged_step.origin.time.ToString(),
                feasible_path.merged_step.destination.time.ToString()
            )
        );
        // Prune nodes that can no longer beat the new UB.
        size_t old_size = q.size();
        std::erase_if(q, [best_ub](const SearchNode& node) {
          return node.parent_lb >= best_ub;
        });
        size_t pruned_count = old_size - q.size();
        if (pruned_count > 0) {
          std::make_heap(q.begin(), q.end());
          logger(
              iter_num,
              BnbLogTag::kQueuePrune,
              std::format("  pruned {} nodes from queue", pruned_count)
          );
        }
      }
    }

    std::vector<Step> primitive_steps;
    for (const TarelEdge& e : lb_result.tour_edges) {
      const auto& paths =
          state.completed.PathsBetween(e.origin.stop, e.destination.stop);
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

    if (primitive_steps.size() > 0) {
      std::ostringstream msg;
      msg << "  primitive: ";
      msg << state.StopName(primitive_steps[0].origin.stop);
      for (const Step& step : primitive_steps) {
        msg << "->" << state.StopName(step.destination.stop);
      }
      logger(iter_num, BnbLogTag::kPrimitive, msg.str());
    }

    if (primitive_steps.size() <= 2) {
      // TODO: Figure out what to do here.
      logger(
          iter_num,
          BnbLogTag::kPruned,
          "  pruned: 2 or fewer primitive steps"
      );
      continue;
    }

    // Select branch edge by hashing edges on LB path.
    // size_t edge_hash = 0;
    // for (const Step& s : primitive_steps) {
    //   edge_hash ^= std::hash<int>{}(s.destination.stop.v) * 31 +
    //   std::hash<int>{}(s.origin.stop.v);
    // }
    // Step& branch_step = primitive_steps[edge_hash % primitive_steps.size()];
    // Step& branch_step = primitive_steps[edge_hash % primitive_steps.size()];
    Step& branch_step = primitive_steps[0];
    BranchEdge branch_edge_fw{
        branch_step.origin.stop, branch_step.destination.stop
    };
    BranchEdge branch_edge_rv{
        branch_step.destination.stop, branch_step.origin.stop
    };

    // Make and push search nodes for branches.

    // TODO: Does using `branch_edge_rv` work afetr we have Required
    // `branch_edge_fw` which removes its endpoints from the required stops??
    // PushQ(state, lb_result.optimal_value,
    // SearchEdge{{branch_edge_fw.Require(), branch_edge_rv.Require()},
    // cur_node.edge_index}); PushQ(state, lb_result.optimal_value,
    // SearchEdge{{branch_edge_fw.Require(), branch_edge_rv.Forbid()},
    // cur_node.edge_index});

    // PushQ(state, lb_result.optimal_value,
    // SearchEdge{{branch_edge_fw.Require()}, cur_node.edge_index});
    // PushQ(state, lb_result.optimal_value,
    // SearchEdge{{branch_edge_fw.Forbid(), branch_edge_rv.Require()},
    // cur_node.edge_index}); PushQ(state, lb_result.optimal_value,
    // SearchEdge{{branch_edge_fw.Forbid(), branch_edge_rv.Forbid()},
    // cur_node.edge_index});

    PushQ(
        state,
        std::max(cur_node.parent_lb, lb_result.optimal_value),
        SearchEdge{{branch_edge_fw.Require()}, cur_node.edge_index}
    );
    PushQ(
        state,
        std::max(cur_node.parent_lb, lb_result.optimal_value),
        SearchEdge{{branch_edge_fw.Forbid()}, cur_node.edge_index}
    );
  }

  return {best_ub, std::move(best_paths), std::move(best_original_edges)};
}

}  // namespace vats5
