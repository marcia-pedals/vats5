#include "solver/branch_and_bound2.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <nlohmann/detail/conversions/to_chars.hpp>
#include <unordered_map>
#include <unordered_set>

#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"

namespace vats5 {

using PartitionStartSteps = std::unordered_map<
    StopId,
    std::unordered_map<StepPartitionId, std::vector<Step>>>;

PartitionStartSteps ComputePartitionStartSteps(
    const StepPathsAdjacencyList& completed
) {
  PartitionStartSteps partition_start_steps;
  for (const Step& step : completed.AllMergedSteps()) {
    if (step.is_flex) {
      partition_start_steps[step.destination.stop][step.destination.partition]
          .push_back(
              Step::PrimitiveFlex(
                  step.destination.stop,
                  step.destination.stop,
                  0,
                  step.destination.trip,
                  step.destination.partition
              )
          );
    } else {
      partition_start_steps[step.destination.stop][step.destination.partition]
          .push_back(
              Step::PrimitiveScheduled(
                  step.destination.stop,
                  step.destination.stop,
                  step.destination.time,
                  step.destination.time,
                  step.destination.trip,
                  step.destination.partition
              )
          );
    }
  }
  for (auto& [_, stop_steps] : partition_start_steps) {
    for (auto& [_, steps] : stop_steps) {
      SortSteps(steps);
      MakeMinimalCover(steps);
    }
  }
  return partition_start_steps;
}

using StepsBetween =
    std::unordered_map<std::pair<StopId, TarelState>, std::vector<Step>>;

StepsBetween ComputeStepsBetween(const StepPathsAdjacencyList& completed) {
  StepsBetween steps_between;
  for (const Step& step : completed.AllMergedSteps()) {
    TarelState tarel_dest{step.destination.stop, step.destination.partition};
    steps_between[{step.origin.stop, tarel_dest}].push_back(step);
  }
  return steps_between;
}

struct Search2State {
  ProblemState problem;
  StepPathsAdjacencyList completed;
};

struct Search2Node {
  int lb;
  int node_index;
  std::unique_ptr<Search2State> state;
  std::optional<std::vector<TarelState>> initial_path;

  bool operator<(const Search2Node& other) const {
    if (lb == other.lb) {
      return node_index > other.node_index;
    }
    return lb > other.lb;
  }
};

struct Search2Constraint {
  bool require;
  TarelEdge edge;
  int step_count;
  int step_rep;
};

struct Search2History {
  std::vector<Search2Constraint> constraints;
};

Search2State ForbidSteps(
    const Search2State& state,
    const TarelEdge& edge,
    const std::vector<Step>& steps
) {
  assert(steps.size() > 0);

  // Copy parts of `state` that we're gonna be mutating.
  StepPathsAdjacencyList completed = state.completed;

  // Find a reference to the path group containing the `steps` that we want to
  // remove.
  std::vector<std::vector<Path>>& adjacent_to_origin =
      completed.adjacent[edge.origin.stop];
  auto path_group_it = std::ranges::find_if(
      adjacent_to_origin, [&](const std::vector<Path>& paths) {
        return (
            paths.size() > 0 &&
            paths[0].merged_step.origin.stop == edge.origin.stop &&
            paths[0].merged_step.destination.stop == edge.destination.stop
        );
      }
  );
  assert(path_group_it != adjacent_to_origin.end());
  std::vector<Path>& paths = *path_group_it;

  // Remove `steps`.
  // Because everything is sorted consistently, we can do a double-pointer.
  int next_to_remove_i = 0;
  std::erase_if(paths, [&](const Path& path) {
    if (next_to_remove_i < steps.size() &&
        steps[next_to_remove_i] == path.merged_step) {
      next_to_remove_i += 1;
      return true;
    }
    return false;
  });

  // We expect to have removed each step exactly once.
  assert(next_to_remove_i == steps.size());

  return Search2State{
      .problem = state.problem,
      .completed = std::move(completed),
  };
}

// TODOOOO: Preserve the Path-iness instead of collapsing things into
// single-step paths.
struct RequireResult {
  Search2State state;
  TarelState new_vertex;
};

RequireResult RequireSteps(
    const Search2State& state,
    const TarelEdge& edge,
    const std::vector<Step>& steps
) {
  assert(steps.size() > 0);
  StopId a = edge.origin.stop;
  StopId b = edge.destination.stop;
  StepPartitionId b_part = steps[0].destination.partition;
  for (const Step& step : steps) {
    assert(step.origin.stop == a);
    assert(step.destination.stop == b);
    assert(step.destination.partition == b_part);
  }

  // Copy parts of `state` that we're gonna be mutating.
  ProblemState problem = state.problem;
  StepPathsAdjacencyList completed = state.completed;

  // TODO: More robust way of producing new StopId.
  StopId ab{static_cast<int>(problem.stop_infos.size())};
  problem.stop_infos[ab] = ProblemStateStopInfo{
      GtfsStopId{""},
      "(" + problem.stop_infos[edge.origin.stop].stop_name + "->" +
          problem.stop_infos[edge.destination.stop].stop_name + ")"
  };
  problem.required.representative[ab] = ab;
  problem.required.EraseGroup(a);
  problem.required.EraseGroup(b);

  assert(!(problem.boundary.start == a && problem.boundary.end == b));
  if (problem.boundary.start == a) {
    problem.boundary.start = ab;
  }
  if (problem.boundary.end == b) {
    problem.boundary.end = ab;
  }

  // Collect some steps that we'll need for constructing the steps to and
  // from "ab".
  // Steps from x to a, grouped by x.
  std::unordered_map<StopId, std::vector<Step>> x_to_as;
  // Steps from b to x, grouped by x.
  std::unordered_map<StopId, std::vector<Step>> b_to_xs;
  for (const Step& s : completed.AllMergedSteps()) {
    if ((s.origin.stop == a && s.destination.stop == b) ||
        (s.origin.stop == b && s.destination.stop == a)) {
      continue;
    }

    if (s.destination.stop == a) {
      x_to_as[s.origin.stop].push_back(s);
    }
    if (s.origin.stop == b) {
      b_to_xs[s.destination.stop].push_back(s);
    }
  }

  // Eliminate a and b from `completed`.
  std::erase_if(completed.adjacent, [&](const auto& kv) {
    return kv.first == a || kv.first == b;
  });
  for (auto& [_, path_groups] : completed.adjacent) {
    std::erase_if(path_groups, [&](const std::vector<Path>& path_group) {
      return (
          path_group.size() == 0 ||
          path_group[0].merged_step.destination.stop == a ||
          path_group[0].merged_step.destination.stop == b
      );
    });
  }

  // ab should only have 1 partition. Below, we assert that it does and
  // initialize this to whatever that partition is.
  StepPartitionId ab_part;

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
  if (problem.boundary.start == ab) {
    // Do (1).

    // Because "a" is START, it should only have one partition. We record what
    // partition it is here, and assert that there is only one.
    std::optional<StepPartitionId> a_part;

    // The steps to ab are: The steps to a.
    for (const auto& [x, x_to_a] : x_to_as) {
      std::vector<Path> path_group_to_ab;
      for (Step x_to_a_step : x_to_a) {
        x_to_a_step.destination.stop = ab;
        path_group_to_ab.push_back(Path{x_to_a_step, {x_to_a_step}});

        if (a_part.has_value()) {
          assert(x_to_a_step.destination.partition == *a_part);
        } else {
          a_part = x_to_a_step.destination.partition;
        }
      }
      completed.adjacent[x].push_back(std::move(path_group_to_ab));
    }
    assert(a_part.has_value());
    ab_part = *a_part;

    // The steps from ab are: The steps a->b merged with the steps from b.
    for (const auto& [x, b_to_x] : b_to_xs) {
      std::vector<Path> path_group_to_x;
      std::vector<Step> merged_steps = PairwiseMergedSteps(steps, b_to_x);
      for (Step merged_step : merged_steps) {
        merged_step.origin.stop = ab;

        // If the step goes to END, we don't want the normal behavior where we
        // take the partition of the latest non-flex step, because it doesn't
        // make sense to have a partition on END.
        //
        // TODO: Figure out if this makes sense and is reasonable, of if it's a
        // crazy hack.
        if (merged_step.destination.stop == problem.boundary.end) {
          merged_step.destination.partition = StepPartitionId::NONE;
        }

        path_group_to_x.push_back(Path{merged_step, {merged_step}});
      }
      completed.adjacent[ab].push_back(std::move(path_group_to_x));
    }

  } else {
    // Do (2).

    ab_part = b_part;

    // The steps to ab are: The steps to a merged with the steps a->b.
    for (const auto& [x, x_to_a] : x_to_as) {
      std::vector<Path> path_group_to_ab;
      std::vector<Step> merged_steps = PairwiseMergedSteps(x_to_a, steps);
      for (Step merged_step : merged_steps) {
        merged_step.destination.stop = ab;
        assert(merged_step.destination.partition == ab_part);

        // If the step goes to END, we don't want the normal behavior where we
        // take the partition of the latest non-flex step, because it doesn't
        // make sense to have a partition on END.
        //
        // TODO: Figure out if this makes sense and is reasonable, of if it's a
        // crazy hack.
        if (merged_step.destination.stop == problem.boundary.end) {
          merged_step.destination.partition = StepPartitionId::NONE;
        }

        path_group_to_ab.push_back(Path{merged_step, {merged_step}});
      }
      completed.adjacent[x].push_back(std::move(path_group_to_ab));
    }

    // The steps from ab are: The steps from b.
    for (const auto& [x, b_to_x] : b_to_xs) {
      std::vector<Path> path_group_to_x;
      for (Step b_to_x_step : b_to_x) {
        b_to_x_step.origin.stop = ab;
        path_group_to_x.push_back(Path{b_to_x_step, {b_to_x_step}});
      }
      completed.adjacent[ab].push_back(std::move(path_group_to_x));
    }
  }

  // int best_in_ab = std::numeric_limits<int>::max();
  // int best_out_ab = std::numeric_limits<int>::max();
  // for (const Step& step : completed.AllMergedSteps()) {
  //   assert(step.origin.stop != a);
  //   assert(step.origin.stop != b);
  //   assert(step.destination.stop != a);
  //   assert(step.destination.stop != b);

  //   if (step.destination.stop == ab && step.DurationSeconds() < best_in_ab) {
  //     best_in_ab = step.DurationSeconds();
  //   }
  //   if (step.origin.stop == ab && step.DurationSeconds() < best_out_ab) {
  //     best_out_ab = step.DurationSeconds();
  //   }
  // }
  // std::cout << "Best in ab: " << best_in_ab << "\n";
  // std::cout << "Best out ab: " << best_out_ab << "\n";

  return RequireResult{
      .state =
          Search2State{
              .problem = std::move(problem),
              .completed = std::move(completed),
          },
      .new_vertex = TarelState(ab, ab_part),
  };
}

struct ReqTreeNode;

struct ReqTreeEdge {
  TarelState a;
  TarelState b;
  int step_rep;
  std::unique_ptr<ReqTreeNode> node;
};

struct ReqTreeNode {
  std::string title;
  int count = 0;
  std::vector<ReqTreeEdge> children = {};
  std::map<std::pair<TarelState, TarelState>, int> forbid_count = {};
};

void IncrementReqTree(
    const ProblemState& state,
    ReqTreeNode& node,
    std::span<const Search2Constraint> constraints
) {
  node.count += 1;

  auto head_constraint = std::find_if(
      constraints.begin(),
      constraints.end(),
      [](const Search2Constraint& constraint) { return constraint.require; }
  );
  if (head_constraint == constraints.end()) {
    if (!constraints.empty()) {
      const Search2Constraint& latest_fbd = *(constraints.end() - 1);
      assert(!latest_fbd.require);
      node.forbid_count[{
          latest_fbd.edge.origin, latest_fbd.edge.destination
      }] += 1;
    }
    return;
  }

  auto it = std::find_if(
      node.children.begin(), node.children.end(), [&](const ReqTreeEdge& edge) {
        return (
            edge.a == head_constraint->edge.origin &&
            edge.b == head_constraint->edge.destination &&
            edge.step_rep == head_constraint->step_rep
        );
      }
  );
  if (it == node.children.end()) {
    std::string title;
    if (true) {
      // If we're simply adding one more step along the previous constraint,
      // omit that to simplify the title.
      // TODO: More robust way of identifying this condition.
      title = "-> " + head_constraint->edge.destination.Debug(state);
    } else {
      title = head_constraint->edge.origin.Debug(state) + " -> " +
              head_constraint->edge.destination.Debug(state);
    }

    node.children.push_back({
        .a = head_constraint->edge.origin,
        .b = head_constraint->edge.destination,
        .step_rep = head_constraint->step_rep,
        .node = std::make_unique<ReqTreeNode>(title),
    });
    it = node.children.end() - 1;
  }
  IncrementReqTree(
      state,
      *it->node,
      constraints.subspan(head_constraint - constraints.begin() + 1)
  );

  // Maintain descending sort by count: the child at `it` may need to move left.
  auto dest =
      std::find_if(node.children.begin(), it, [&](const ReqTreeEdge& edge) {
        return edge.node->count < it->node->count;
      });
  if (dest != it) {
    std::rotate(dest, it, it + 1);
  }
}

void PrintReqTree(const ReqTreeNode& node, int depth = 0) {
  if (node.count <= 1) {
    return;
  }
  std::string indent(2 * depth, ' ');

  int total_forbid_count = 0;
  for (const auto& [_, count] : node.forbid_count) {
    total_forbid_count += count;
  }

  std::cout << indent << node.title << ": " << node.count << " ("
            << total_forbid_count << " forbids / " << node.forbid_count.size()
            << " edges)\n";
  for (const ReqTreeEdge& edge : node.children) {
    PrintReqTree(*edge.node, depth + 1);
  }
}

BranchAndBound2Result BranchAndBound2Solve(
    const ProblemState& initial_state, const SearchEventCallback& on_event
) {
  int next_node_index = 0;
  std::vector<Search2History> history;
  std::vector<Search2Node> q;
  q.push_back(
      {.lb = 0,
       .node_index = next_node_index,
       .state = std::make_unique<Search2State>(
           initial_state, initial_state.ComputeCompletedGraph()
       )}
  );
  history.push_back({.constraints = {}});
  next_node_index += 1;

  int iter_count = 0;
  int best_ub = std::numeric_limits<int>::max();
  ReqTreeNode req_tree_root({
      .title = "root",
  });
  while (!q.empty()) {
    std::pop_heap(q.begin(), q.end());
    Search2Node node = std::move(q.back());
    q.pop_back();

    iter_count += 1;
    std::cout << "Iter " << iter_count << ": take "
              << TimeSinceServiceStart{node.lb} << " (" << (q.size() + 1)
              << " active)\n";

    if (node.lb >= best_ub) {
      std::cout << "  search terminated: lb >= ub\n";
      break;
    }

    IncrementReqTree(
        node.state->problem, req_tree_root, history[node.node_index].constraints
    );
    PrintReqTree(req_tree_root);

    // for (const Search2Constraint& c : history[node.node_index].constraints) {
    //   std::cout << "  ";
    //   if (c.require) {
    //     std::cout << "REQ";
    //   } else {
    //     std::cout << "FBD";
    //   }
    //   std::cout << " " << c.edge.Debug(node.state->problem) << " - "
    //             << c.step_count << "\n";
    // }

    std::optional<TspTourResult> lb_result = ComputeTarelLowerBound(
        node.state->problem,
        node.state->completed,
        best_ub < std::numeric_limits<int>::max() ? std::optional(best_ub)
                                                  : std::nullopt,
        // &std::cout,
        nullptr,  // TODO: out file
        on_event
    );
    if (!lb_result.has_value()) {
      std::cout << "  infeasible from tarel"
                << " (best " << TimeSinceServiceStart{best_ub} << ")\n";
      continue;
    }
    std::cout << "  delta-lb: "
              << TimeSinceServiceStart{lb_result->optimal_value - node.lb}
              << "\n";

    {
      // Make an upper bound by actually following the LB path.
      std::vector<StopId> stop_sequence;
      stop_sequence.push_back(lb_result->tour_edges[0].origin.stop);
      for (const auto& edge : lb_result->tour_edges) {
        stop_sequence.push_back(edge.destination.stop);
      }
      std::vector<Path> feasible_paths = ComputeMinimalFeasiblePathsAlong(
          stop_sequence, node.state->completed
      );
      if (feasible_paths.size() > 0) {
        const Path& feasible_path = *std::min_element(
            feasible_paths.begin(),
            feasible_paths.end(),
            [](const Path& a, const Path& b) {
              return a.DurationSeconds() < b.DurationSeconds();
            }
        );
        if (feasible_path.DurationSeconds() < best_ub) {
          best_ub = feasible_path.DurationSeconds();
        }
        std::cout << "  feasible path: "
                  << TimeSinceServiceStart{feasible_path.DurationSeconds()}
                  << " (best " << TimeSinceServiceStart{best_ub} << ")\n";
      }
    }

    PartitionStartSteps partition_start_steps =
        ComputePartitionStartSteps(node.state->completed);
    StepsBetween steps_between = ComputeStepsBetween(node.state->completed);

    // We try to find the branch constraint that maximizes the min lb of the
    // children. We can't do it exactly so we do some approximation.
    //
    // The tradeoff here is that if we take more steps, the child where we
    // forbid the steps gets bigger but the child where we require at least one
    // of the steps gets smaller.
    //
    // Forbidding steps increases the bound of all routes that use those steps
    // by the smallest error of the remaining steps on that route. e.g. if we
    // forbid all the zero-error steps from (x,p)->(y,q), then we increase the
    // bound of the routes by the amount of the smallest nonzero error
    // (x,p)->(y,q).
    //
    // Requiring steps increases the bound of ??? by ???. But like I hope that
    // if we require a very small number of steps then we'll be able to quickly
    // disprove the branch because this is extremely constraining and the
    // constraint propagates outwards quickly.
    //
    // So our approximation for now is to find the edge on lb_result.tour_edges
    // that has the smallest number of zero-error steps, breaking ties by the
    // smallest nonzero error.
    struct EdgeErrorSummary {
      int zero_error_count;
      int smallest_nonzero_error;
      TarelEdge edge;
      std::vector<Step> zero_error_steps;

      bool BetterThan(const EdgeErrorSummary& other) {
        if (zero_error_count == other.zero_error_count) {
          return smallest_nonzero_error > other.smallest_nonzero_error;
        }
        return zero_error_count < other.zero_error_count;
      }
    };
    EdgeErrorSummary best_constraint{
        .zero_error_count = std::numeric_limits<int>::max(),
        .smallest_nonzero_error = 0,
    };

    // std::cout << "\n";
    for (const TarelEdge& e : lb_result->tour_edges) {
      // std::cout << "Investigating " << e.Debug(node.state->problem) << "\n";
      std::vector<StepProvenance> prov;
      std::vector<Step> e_steps = PairwiseMergedSteps(
          partition_start_steps[e.origin.stop][e.origin.partition],
          steps_between[{e.origin.stop, e.destination}],
          &prov
      );

      // std::cout
      //     << "  partition starts: "
      //     << partition_start_steps[e.origin.stop][e.origin.partition].size()
      //     << "\n"
      //     << "  steps between: "
      //     << steps_between[{e.origin.stop, e.destination}].size() << "\n"
      //     << "  merged: " << e_steps.size() << "\n";

      std::vector<Step> zero_error_steps;
      std::map<int, int> error_histogram;
      for (int e_step_i = 0; e_step_i < e_steps.size(); ++e_step_i) {
        int error = e_steps[e_step_i].DurationSeconds() - e.weight;
        assert(error >= 0);
        error_histogram[error] += 1;
        if (error == 0) {
          zero_error_steps.push_back(
              steps_between[{e.origin.stop, e.destination}]
                           [prov[e_step_i].bc_index]
          );
        }
      }

      // for (const auto& [error, count] : error_histogram) {
      //   std::cout << "  " << TimeSinceServiceStart{error} << ": " << count
      //             << "\n";
      // }

      int smallest_nonzero_error = std::numeric_limits<int>::max();
      if (error_histogram.size() > 1) {
        auto second_it = error_histogram.begin();
        ++second_it;
        smallest_nonzero_error = second_it->first;
      }

      assert(error_histogram[0] > 0);
      EdgeErrorSummary error_summary{
          .zero_error_count = error_histogram[0],
          .smallest_nonzero_error = smallest_nonzero_error,
          .edge = e,
          .zero_error_steps = std::move(zero_error_steps),
      };

      if (error_summary.BetterThan(best_constraint)) {
        best_constraint = error_summary;
      }
    }

    std::vector<TarelState> lb_tarel_path;
    assert(lb_result->tour_edges.size() > 0);
    lb_tarel_path.push_back(lb_result->tour_edges[0].origin);
    for (const TarelEdge& edge : lb_result->tour_edges) {
      lb_tarel_path.push_back(edge.destination);
    }

    // std::cout << "Best constraint "
    //           << best_constraint.edge.Debug(node.state->problem) << "\n";
    // std::cout << "  zero_error_count: " << best_constraint.zero_error_count
    //           << "\n";
    // std::cout << "  smallest_nonzero_error: "
    //           <<
    //           TimeSinceServiceStart{best_constraint.smallest_nonzero_error}
    //           << "\n";

    q.push_back(
        Search2Node{
            .lb = std::max(node.lb, lb_result->optimal_value),
            .node_index = next_node_index,
            .state = std::make_unique<Search2State>(ForbidSteps(
                *node.state,
                best_constraint.edge,
                best_constraint.zero_error_steps
            )),
            .initial_path = lb_tarel_path,
        }
    );
    std::push_heap(q.begin(), q.end());
    Search2History hist_with_forbid = history[node.node_index];
    hist_with_forbid.constraints.push_back({
        .require = false,
        .edge = best_constraint.edge,
        .step_count = static_cast<int>(best_constraint.zero_error_steps.size()),
        .step_rep = std::ranges::min_element(
                        best_constraint.zero_error_steps,
                        {},
                        [](const Step& step) { return step.origin.time; }
        )->origin.time.seconds,
    });
    history.push_back(hist_with_forbid);
    next_node_index += 1;

    RequireResult require_result = RequireSteps(
        *node.state, best_constraint.edge, best_constraint.zero_error_steps
    );
    std::unique_ptr<Search2State> require_state =
        std::make_unique<Search2State>(std::move(require_result.state));

    std::vector<TarelState> lb_tarel_path_reqd = lb_tarel_path;
    auto a_it = std::find_if(
        lb_tarel_path_reqd.begin(),
        lb_tarel_path_reqd.end(),
        [&](const TarelState& state) {
          return state.stop == best_constraint.edge.origin.stop;
        }
    );
    assert(a_it < lb_tarel_path_reqd.end() - 1);
    assert((a_it + 1)->stop == best_constraint.edge.destination.stop);
    *a_it = require_result.new_vertex;
    lb_tarel_path_reqd.erase(a_it + 1);

    // {
    //   StopId ab =
    //   StopId{static_cast<int>(require_state->problem.stop_infos.size()) - 1};
    //   auto require_state_edges = MakeTarelEdges(require_state->completed);

    //   std::vector<TarelEdge> new_tarel_tour = lb_result->tour_edges;
    //   auto ab_it = std::find_if(new_tarel_tour.begin(), new_tarel_tour.end(),
    //   [&](const TarelEdge& edge) {
    //     return edge.origin == best_constraint.edge.origin && edge.destination
    //     == best_constraint.edge.destination;
    //   });
    //   assert(ab_it != new_tarel_tour.end());

    //   int new_tarel_weight = 0;
    //   for (auto it = new_tarel_tour.begin(); it < new_tarel_tour.end(); ++it)
    //   {
    //     if (it == ab_it) {
    //       // Intentionally skip.
    //     } else if (ab_it > new_tarel_tour.begin() && it == ab_it - 1) {
    //       auto new_edge_it = std::find_if(require_state_edges.begin(),
    //       require_state_edges.end(), [&](const TarelEdge& edge) {
    //         // I think that there is only one ab partition by construction,
    //         but I'm not sure.
    //         // TODO: Verify.
    //         return edge.origin == it->origin && edge.destination.stop == ab;
    //       });
    //       assert(new_edge_it != require_state_edges.end());
    //       new_tarel_weight += new_edge_it->weight;
    //     } else if (ab_it + 1 < new_tarel_tour.end() && it == ab_it + 1) {
    //       auto new_edge_it = std::find_if(require_state_edges.begin(),
    //       require_state_edges.end(), [&](const TarelEdge& edge) {
    //         // I think that there is only one ab partition by construction,
    //         but I'm not sure.
    //         // TODO: Verify.
    //         return edge.origin.stop == ab && edge.destination ==
    //         it->destination;
    //       });
    //       assert(new_edge_it != require_state_edges.end());
    //       new_tarel_weight += new_edge_it->weight;
    //     } else {
    //       new_tarel_weight += it->weight;
    //     }
    //   }

    //   std::cout << "  new REQ tour tarel weight delta: " <<
    //   TimeSinceServiceStart{new_tarel_weight - lb_result->optimal_value} <<
    //   "\n";
    // }

    q.push_back(
        Search2Node{
            .lb = std::max(node.lb, lb_result->optimal_value),
            .node_index = next_node_index,
            .state = std::move(require_state),
            .initial_path = lb_tarel_path_reqd,
        }
    );
    std::push_heap(q.begin(), q.end());
    Search2History hist_with_require = history[node.node_index];
    hist_with_require.constraints.push_back({
        .require = true,
        .edge = best_constraint.edge,
        .step_count = static_cast<int>(best_constraint.zero_error_steps.size()),
        .step_rep = std::ranges::min_element(
                        best_constraint.zero_error_steps,
                        {},
                        [](const Step& step) { return step.origin.time; }
        )->origin.time.seconds,
    });
    history.push_back(hist_with_require);
    next_node_index += 1;
  }

  return BranchAndBound2Result{.best_ub = best_ub};
}

}  // namespace vats5
