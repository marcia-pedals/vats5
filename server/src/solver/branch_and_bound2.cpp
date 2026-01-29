#include "solver/branch_and_bound2.h"
#include <unordered_map>
#include "solver/data.h"
#include "solver/step_merge.h"

namespace vats5 {


ProblemState ApplyConstraint(
  const ProblemState& state,
  const ElaborateConstraint& constraint
) {
  assert(constraint.required_path.size() >= 1);

  // Mutable copies of all the `ProblemState` fields we'll be mutating.
  std::vector<Step> steps = state.minimal.AllSteps();
  ProblemBoundary boundary = state.boundary;
  std::unordered_set<StopId> required_stops = state.required_stops;
  std::unordered_map<StopId, std::string> stop_names = state.stop_names;
  StopId next_stop_id{state.minimal.NumStops()};

  // Add a new stop "(required_path[0]->...->required_path[n])" representing the
  // act of going to "required_path[0]", following it all the way, and ending up
  // at "required_path[n]". Make this a new required stop, and remove all of
  // "required_path[i]" from the required stops.
  StopId required_path_stop;
  if (constraint.required_path.size() == 1) {
    // Special case: If the required path only has 1 stop, then we don't need to
    // do anything to make a new "required path stop".
    required_path_stop = constraint.required_path[0];
  } else {
    // General case: The required path has >1 stops.
    assert(constraint.required_path.size() > 1);
    required_path_stop = next_stop_id;
    next_stop_id.v += 1;
    {
      std::stringstream ss;
      ss << "(";
      for (int i = 0; i < constraint.required_path.size(); ++i) {
        if (i > 0) {
          ss << "->";
        }
        ss << state.StopName(constraint.required_path[i]);
      }
      ss << ")";
      stop_names[required_path_stop] = std::move(ss.str());
    }
    required_stops.insert(required_path_stop);
    for (StopId s : constraint.required_path) {
      required_stops.erase(s);
    }

    std::unordered_map<StopId, int> required_path_index;
    for (int i = 0; i < constraint.required_path.size(); ++i) {
      required_path_index[constraint.required_path[i]] = i;
    }

    // Collect some steps that we'll need for constructing the steps to and from
    // "required_path_stop".
    // `along_path[i]` are the steps from `required_path[i]` to `required_path[i+1]`.
    std::vector<std::vector<Step>> along_path(constraint.required_path.size() - 1);
    // Steps from * to `required_path[0]`, grouped by *.
    std::unordered_map<StopId, std::vector<Step>> star_to_start;
    // Steps from `required_path[n]` to *, not grouped by anything.
    std::vector<Step> end_to_star;
    for (const Step& step : steps) {
      // Collect into `along_path`.
      {
        auto origin_required_path_index_it = required_path_index.find(step.origin.stop);
        if (origin_required_path_index_it != required_path_index.end()) {
          int index = origin_required_path_index_it->second;
          if (
            index < constraint.required_path.size() - 1 &&
            step.destination.stop == constraint.required_path[index + 1]
          ) {
            along_path[index].push_back(step);
          }
        }
      }

      // Collect into `star_to_start`.
      if (step.destination.stop == *(constraint.required_path.begin())) {
        star_to_start[step.origin.stop].push_back(step);
      }

      // Collect into `end_to_star`.
      if (step.origin.stop == *(constraint.required_path.end() - 1)) {
        end_to_star.push_back(step);
      }
    }

    // Sorted and minimal should have been preserved in all the things we've
    // collected, but assert it to make sure.
    for (const std::vector<Step>& along_path_element : along_path) {
      assert(CheckSortedAndMinimal(along_path_element));
    }
    for (const auto& [star, star_to_start_steps] : star_to_start) {
      assert(CheckSortedAndMinimal(star_to_start_steps));
    }

    std::vector<Step> merged_along_path = along_path[0];
    for (int i = 1; i < along_path.size(); ++i) {
      merged_along_path = PairwiseMergedSteps(merged_along_path, along_path[i]);
    }

    // The steps to "(required_path[0]->...->required_path[n])" are the steps
    // "x->required_path[0]" merged with the required path.
    for (const auto& [x, x_to_start] : star_to_start) {
      for (Step step : PairwiseMergedSteps(x_to_start, merged_along_path)) {
        step.destination.stop = required_path_stop;
        steps.push_back(step);
      }
    }

    // The steps from "(required_path[0]->...->required_path[n])" are the steps
    // "required_path[n]->x".
    for (Step step : end_to_star) {
      step.origin.stop = required_path_stop;
      steps.push_back(step);
    }
  }

  // Ok now forbid steps from the required path to the "forbid next".
  if (constraint.forbid_next.has_value()) {
    StopId forbid_next = constraint.forbid_next.value();
    if (constraint.required_path.size() == 1) {
      // Special case: If required path only has 1 stop, just forbid steps from
      // there to "forbid next".
      // TODO: Hmm think, maybe I do also need required_path[0]' thingy for this.
      StopId required_stop = constraint.required_path[0];
      std::erase_if(steps, [required_stop, forbid_next](const Step& s) {
        return s.origin.stop == required_stop && s.destination.stop == forbid_next;
      });
    } else {
      // General case: Required path has >1 stops.
      assert(constraint.required_path.size() > 1);

      // For each "required_path[i]", add a new stop "required_path[i]'"
      // representing that we are not merely repeating part of the required path
      // after having visited it.
      std::vector<StopId> required_path_prime;
      std::unordered_map<StopId, StopId> to_prime;
      required_path_prime.reserve(constraint.required_path.size());
      for (StopId s : constraint.required_path) {
        StopId s_prime = next_stop_id;
        next_stop_id.v += 1;
        to_prime[s] = s_prime;
        stop_names[s_prime] = stop_names[s] + "'";
      }

      auto PrimeIfInRequiredPath = [&to_prime](StopId s) -> StopId {
        auto it = to_prime.find(s);
        return it == to_prime.end() ? s : it->second;
      };

      // Add all incoming and outgoing steps to the required path prime.
      for (const Step& step : steps) {
        if (step.origin.stop == required_path_stop) {
          continue;
        }

        Step step_prime = step;
        step_prime.origin.stop = PrimeIfInRequiredPath(step_prime.origin.stop);
        step_prime.destination.stop = PrimeIfInRequiredPath(step_prime.destination.stop);
        if (step_prime != step) {
          steps.push_back(step_prime);
        }
      }

      // Forbid steps from the required path to "forbid next".
      std::erase_if(steps, [&](const Step& step) {
        return (
          (step.origin.stop == required_path_stop || step.origin.stop == *(constraint.required_path.end() - 1)) &&
          step.destination.stop == forbid_next
        );
      });
    }
  }

  // Build the new problem state from the stuff we've been mutating.
  return MakeProblemState(
    MakeAdjacencyList(steps),
    std::move(boundary),
    std::move(required_stops),
    std::move(stop_names),
    state.step_partition_names
  );
}

int DummyFunction() {
  return 42;
}

}  // namespace vats5
