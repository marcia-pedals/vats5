#include "solver/branch_and_bound.h"

#include <sys/stat.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"

namespace vats5 {

std::string ConstraintRequireEdge::Debug(const ProblemState& state) const {
  return "[require " + state.StopName(a) + " -> " + state.StopName(b) + "]";
}

std::string ConstraintForbidEdge::Debug(const ProblemState& state) const {
  return "[forbid " + state.StopName(a) + " -> " + state.StopName(b) + "]";
}

std::string BranchEdge::Debug(const ProblemState& state) const {
  return state.StopName(a) + " -> " + state.StopName(b);
}

std::string Debug(const ProblemConstraint& c, const ProblemState& state) {
  return std::visit([&](const auto& x) { return x.Debug(state); }, c);
}

ProblemState ApplyConstraints(
    const ProblemState& state, const std::vector<ProblemConstraint>& constraints
) {
  // Mutable copies of all the `ProblemState` fields we'll be mutating.
  std::vector<Step> steps = state.minimal.AllSteps();
  ProblemBoundary boundary = state.boundary;
  RequiredStops required = state.required;
  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos =
      state.stop_infos;
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
          "(" + stop_infos[require.a].stop_name + "->" +
              stop_infos[require.b].stop_name + ")"
      };
      required.representative[ab] = ab;

      required.EraseGroup(require.a);
      required.EraseGroup(require.b);

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
      std::move(required),
      std::move(stop_infos),
      state.step_partition_names,
      std::move(original_edges)
  );
}

namespace {

// Removes required stops that "split" the network: a required stop x is
// implied if there is a pair of stops a, b that any feasible path certainly
// visits (the boundary stops, or members of singleton required groups) such
// that, with x removed from the graph, there is no time-feasible path a -> b
// and no time-feasible path b -> a. Any feasible path visits a and b in some
// order, so it must pass through x either way, and the requirement to visit x
// is redundant.
//
// Reachability is time-feasible: an earliest-arrival search from time 0
// (which considers every departure) rather than plain topological
// reachability, so a stop pair connected only by schedules that cannot
// actually be chained still counts as split.
//
// Removals are processed sequentially, with the justification stops always
// drawn from the still-required set, so each removed stop remains certainly
// visited even when several stops are removed.
//
// Returns the number of required stops removed (whole groups are erased).
int RemoveSplitRequiredStops(ProblemState& state, std::ostream* search_log) {
  const StepsAdjacencyList& adj = state.minimal;

  // Earliest-arrival steps from `from` to `destinations`, never passing
  // through `removed`. A destination d is time-feasibly reachable iff the
  // result's entry for d is visited.
  auto Reachable = [&adj](
                       StopId from,
                       StopId removed,
                       const std::unordered_set<StopId>& destinations
                   ) -> std::vector<Step> {
    return FindShortestPathsAtTime(
        adj,
        TimeSinceServiceStart{0},
        from,
        destinations,
        nullptr,
        nullptr,
        removed
    );
  };
  auto Visited = [](const std::vector<Step>& result, StopId stop) -> bool {
    return result[stop.v].destination.time.seconds !=
           std::numeric_limits<int>::max();
  };

  std::vector<StopId> candidates(
      state.required.AllFlat().begin(), state.required.AllFlat().end()
  );
  std::sort(candidates.begin(), candidates.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });

  int removed_count = 0;
  for (StopId x : candidates) {
    if (!state.required.Contains(x)) {
      continue;  // Group already erased via another member.
    }
    StopId x_rep = state.required.Representative(x);
    // Never unrequire the boundary stops' groups; the boundary is handled
    // specially by the tarel graph construction.
    if (state.required.Representative(state.boundary.start) == x_rep ||
        state.required.Representative(state.boundary.end) == x_rep) {
      continue;
    }

    // Stops that any feasible path certainly visits: the boundary stops and
    // every singleton required group outside x's group. (A multi-stop group
    // only guarantees that some member is visited, so its members can't
    // serve as justification.)
    std::vector<StopId> forced{state.boundary.start, state.boundary.end};
    for (const std::vector<StopId>& group : state.required.Groups()) {
      if (group.size() == 1 && group[0] != state.boundary.start &&
          group[0] != state.boundary.end &&
          state.required.Representative(group[0]) != x_rep) {
        forced.push_back(group[0]);
      }
    }

    std::unordered_set<StopId> forced_set(forced.begin(), forced.end());
    std::vector<std::vector<Step>> reach;
    reach.reserve(forced.size());
    for (StopId a : forced) {
      reach.push_back(Reachable(a, x, forced_set));
    }

    bool split = false;
    for (size_t i = 0; i < forced.size() && !split; ++i) {
      for (size_t j = i + 1; j < forced.size() && !split; ++j) {
        if (!Visited(reach[i], forced[j]) && !Visited(reach[j], forced[i])) {
          split = true;
          if (search_log != nullptr) {
            *search_log << "  unrequire " << state.StopName(x) << " (splits "
                        << state.StopName(forced[i]) << " | "
                        << state.StopName(forced[j]) << ")\n";
          }
        }
      }
    }
    if (split) {
      size_t before = state.required.size();
      state.required.EraseGroup(x);
      removed_count += static_cast<int>(before - state.required.size());
    }
  }
  return removed_count;
}

}  // namespace

BranchAndBoundResult BranchAndBoundSolve(
    const ProblemState& initial_state,
    std::ostream* search_log,
    std::optional<std::string> run_dir,
    int max_iter,
    const SearchEventCallback& on_event
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
  std::vector<int> split_removed_counts;

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
      *search_log << iter_num << " (" << q.size() + 1 << " active nodes) Take "
                  << TimeSinceServiceStart{cur_node.parent_lb};
      if (cur_node.edge_index != -1) {
        for (const auto& c : search_edges[cur_node.edge_index].constraints) {
          *search_log << " " << Debug(c, state);
        }
      }
      *search_log << " {cur " << cur_node.edge_index;
      if (cur_node.edge_index != -1) {
        *search_log << "; parent "
                    << search_edges[cur_node.edge_index].parent_edge_index;
      }
      *search_log << "}\n";
      // TODO: Detailed log level so that we can print these out sometimes.
      // showValue(state, *search_log);
      // *search_log << "\n";
    }

    // if (search_log != nullptr) {
    //   showValue(state, *search_log);
    //   *search_log << "\n";
    // }

    if (cur_node.parent_lb >= best_ub) {
      if (search_log != nullptr) {
        *search_log << "Search terminated: LB >= UB\n";
      }
      return {
          best_ub,
          std::move(best_paths),
          std::move(best_original_edges),
          std::move(split_removed_counts)
      };
    }

    split_removed_counts.push_back(RemoveSplitRequiredStops(state, search_log));

    StepPathsAdjacencyList completed = state.ComputeCompletedGraph();

    // Compute lower bound.
    std::optional<std::ofstream> tsp_log_file;
    if (run_dir.has_value()) {
      std::string iter_dir =
          run_dir.value() + "/iter" + std::to_string(iter_num);
      mkdir(iter_dir.c_str(), 0755);
      tsp_log_file.emplace(iter_dir + "/tsp_log");
    }
    std::optional<TspTourResult> lb_result_opt = ComputeTarelLowerBound(
        state,
        // std::nullopt,
        best_ub < std::numeric_limits<int>::max() ? std::make_optional(best_ub)
                                                  : std::nullopt,
        tsp_log_file.has_value() ? &tsp_log_file.value() : nullptr,
        on_event
    );
    if (!lb_result_opt.has_value()) {
      // Infeasible node!
      if (search_log != nullptr) {
        *search_log << "  infeasible from tarel\n";
      }
      continue;
    }
    TspTourResult& lb_result = lb_result_opt.value();

    if (lb_result.optimal_value >= best_ub) {
      // Pruned node!
      if (search_log != nullptr) {
        *search_log << "  pruned: LB ("
                    << TimeSinceServiceStart{lb_result.optimal_value}
                    << ") >= UB (" << TimeSinceServiceStart{best_ub} << ")\n";
      }
      continue;
    }
    if (search_log != nullptr) {
      *search_log << "  lb: " << TimeSinceServiceStart{lb_result.optimal_value}
                  << "\n";
      *search_log << "  lb edges:\n";
      for (const TarelEdge& edge : lb_result.tour_edges) {
        *search_log << "    " << state.StopName(edge.origin.stop) << " -> "
                    << state.StopName(edge.destination.stop)
                    << " w=" << TimeSinceServiceStart{edge.weight} << "\n";
      }
    }

    // Make an upper bound by actually following the LB path.
    std::vector<StopId> stop_sequence;
    stop_sequence.push_back(lb_result.tour_edges[0].origin.stop);
    for (const auto& edge : lb_result.tour_edges) {
      stop_sequence.push_back(edge.destination.stop);
    }
    std::vector<Path> feasible_paths =
        ComputeMinimalFeasiblePathsAlong(stop_sequence, completed);
    if (feasible_paths.size() > 0) {
      const Path& feasible_path = *std::min_element(
          feasible_paths.begin(),
          feasible_paths.end(),
          [](const Path& a, const Path& b) {
            return a.DurationSeconds() < b.DurationSeconds();
          }
      );
      if (search_log != nullptr) {
        *search_log << "  ub path ("
                    << TimeSinceServiceStart{feasible_path.DurationSeconds()}
                    << "): ";
        for (int i = 0; i < lb_result.tour_edges.size() - 1; ++i) {
          if (i > 0) {
            *search_log << " -> ";
          }
          TarelEdge& edge = lb_result.tour_edges[i];
          *search_log << state.StopName(edge.destination.stop);
        }
        *search_log << "\n";
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
        if (search_log != nullptr) {
          *search_log << "  found new ub " << TimeSinceServiceStart{best_ub}
                      << " " << feasible_path.merged_step.origin.time << " "
                      << feasible_path.merged_step.destination.time << "\n";
        }
        // Prune nodes that can no longer beat the new UB.
        size_t old_size = q.size();
        std::erase_if(q, [best_ub](const SearchNode& node) {
          return node.parent_lb >= best_ub;
        });
        size_t pruned_count = old_size - q.size();
        if (pruned_count > 0) {
          std::make_heap(q.begin(), q.end());
          if (search_log != nullptr) {
            *search_log << "  pruned " << pruned_count << " nodes from queue\n";
          }
        }
      }
    }

    std::vector<Step> primitive_steps;
    for (const TarelEdge& e : lb_result.tour_edges) {
      const auto& paths =
          completed.PathsBetween(e.origin.stop, e.destination.stop);
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

    if (search_log != nullptr && primitive_steps.size() > 0) {
      *search_log << "  primitive: ";
      *search_log << state.StopName(primitive_steps[0].origin.stop);
      for (const Step& step : primitive_steps) {
        *search_log << "->" << state.StopName(step.destination.stop);
      }
      *search_log << "\n";
    }

    if (primitive_steps.size() <= 1) {
      // TODO: Figure out what to do here.
      if (search_log != nullptr) {
        *search_log << "  pruned: 1 or fewer primitive steps\n";
      }
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

  return {
      best_ub,
      std::move(best_paths),
      std::move(best_original_edges),
      std::move(split_removed_counts)
  };
}

}  // namespace vats5
