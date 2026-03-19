#include "solver/branch_and_bound2.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <nlohmann/detail/conversions/to_chars.hpp>
#include <unordered_map>

#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"

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
Search2State RequireSteps(
    const Search2State& state,
    const TarelEdge& edge,
    const std::vector<Step>& steps
) {
  assert(steps.size() > 0);
  StopId a = edge.origin.stop;
  StopId b = edge.destination.stop;
  for (const Step& step : steps) {
    assert(step.origin.stop == a && step.destination.stop == b);
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

    // The steps to ab are: The steps to a.
    for (const auto& [x, x_to_a] : x_to_as) {
      std::vector<Path> path_group_to_ab;
      for (Step x_to_a_step : x_to_a) {
        x_to_a_step.destination.stop = ab;
        path_group_to_ab.push_back(Path{x_to_a_step, {x_to_a_step}});
      }
      completed.adjacent[x].push_back(std::move(path_group_to_ab));
    }

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

    // The steps to ab are: The steps to a merged with the steps a->b.
    for (const auto& [x, x_to_a] : x_to_as) {
      std::vector<Path> path_group_to_ab;
      std::vector<Step> merged_steps = PairwiseMergedSteps(x_to_a, steps);
      for (Step merged_step : merged_steps) {
        merged_step.destination.stop = ab;

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

  return Search2State{
      .problem = std::move(problem),
      .completed = std::move(completed),
  };
}

BranchAndBound2Result BranchAndBound2Solve(const ProblemState& initial_state) {
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
  while (!q.empty()) {
    std::pop_heap(q.begin(), q.end());
    Search2Node node = std::move(q.back());
    q.pop_back();

    iter_count += 1;
    std::cout << "Iter " << iter_count << ": take "
              << TimeSinceServiceStart{node.lb} << " (" << (q.size() + 1)
              << " active)\n";

    for (const Search2Constraint& c : history[node.node_index].constraints) {
      std::cout << "  ";
      if (c.require) {
        std::cout << "REQ";
      } else {
        std::cout << "FBD";
      }
      std::cout << " " << c.edge.Debug(node.state->problem) << " - "
                << c.step_count << "\n";
    }

    std::optional<TspTourResult> lb_result = ComputeTarelLowerBound(
        node.state->problem,
        node.state->completed,
        std::nullopt,  // TODO: UB
        nullptr,       // TODO: out file
        nullptr        // TODO: search event callback
    );
    if (!lb_result.has_value()) {
      std::cout << "  infeasible from tarel\n";
      continue;
    }
    std::cout << "  delta-lb: "
              << TimeSinceServiceStart{lb_result->optimal_value - node.lb}
              << "\n";

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

    std::cout << "\n";
    for (const TarelEdge& e : lb_result->tour_edges) {
      std::cout << "Investigating " << e.Debug(node.state->problem) << "\n";
      std::vector<StepProvenance> prov;
      std::vector<Step> e_steps = PairwiseMergedSteps(
          partition_start_steps[e.origin.stop][e.origin.partition],
          steps_between[{e.origin.stop, e.destination}],
          &prov
      );

      std::cout
          << "  partition starts: "
          << partition_start_steps[e.origin.stop][e.origin.partition].size()
          << "\n"
          << "  steps between: "
          << steps_between[{e.origin.stop, e.destination}].size() << "\n"
          << "  merged: " << e_steps.size() << "\n";

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

      for (const auto& [error, count] : error_histogram) {
        std::cout << "  " << TimeSinceServiceStart{error} << ": " << count
                  << "\n";
      }

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
        }
    );
    std::push_heap(q.begin(), q.end());
    Search2History hist_with_forbid = history[node.node_index];
    hist_with_forbid.constraints.push_back({
        .require = false,
        .edge = best_constraint.edge,
        .step_count = static_cast<int>(best_constraint.zero_error_steps.size()),
    });
    history.push_back(hist_with_forbid);
    next_node_index += 1;

    q.push_back(
        Search2Node{
            .lb = std::max(node.lb, lb_result->optimal_value),
            .node_index = next_node_index,
            .state = std::make_unique<Search2State>(RequireSteps(
                *node.state,
                best_constraint.edge,
                best_constraint.zero_error_steps
            )),
        }
    );
    std::push_heap(q.begin(), q.end());
    Search2History hist_with_require = history[node.node_index];
    hist_with_require.constraints.push_back({
        .require = true,
        .edge = best_constraint.edge,
        .step_count = static_cast<int>(best_constraint.zero_error_steps.size()),
    });
    history.push_back(hist_with_require);
    next_node_index += 1;
  }

  return BranchAndBound2Result{.best_ub = best_ub};
}

}  // namespace vats5
