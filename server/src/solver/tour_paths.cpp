#include "solver/tour_paths.h"

#include <algorithm>
#include <optional>

#include "solver/step_merge.h"

namespace vats5 {

std::vector<Path> ComputeMinDurationFeasiblePaths(
    const std::vector<StopId>& stop_sequence,
    const StepPathsAdjacencyList& completed
) {
  if (stop_sequence.size() < 2) {
    return {};
  }

  StopId start = stop_sequence[0];

  std::vector<Step> feasible_steps = {ZeroEdge(start, start)};
  auto ExtendFeasibleSteps = [&](StopId a, StopId b) {
    feasible_steps =
        PairwiseMergedSteps(feasible_steps, completed.MergedStepsBetween(a, b));
  };
  for (size_t i = 0; i + 1 < stop_sequence.size(); ++i) {
    ExtendFeasibleSteps(stop_sequence[i], stop_sequence[i + 1]);
  }

  // TODO: Reference thing about 00:00:00.
  std::erase_if(feasible_steps, [](const Step& step) {
    return step.origin.time < TimeSinceServiceStart{0};
  });

  auto min_step_it = std::min_element(
      feasible_steps.begin(),
      feasible_steps.end(),
      [](const Step& a, const Step& b) {
        return a.DurationSeconds() < b.DurationSeconds();
      }
  );
  if (min_step_it == feasible_steps.end()) {
    return {};
  }
  int min_duration = min_step_it->DurationSeconds();

  std::vector<Path> result;
  for (const Step& merged_step : feasible_steps) {
    if (merged_step.DurationSeconds() > min_duration) {
      continue;
    }

    Path path{.merged_step = merged_step, .steps = {}};

    // Build up the individual steps along the tour.
    Step cur_step = ZeroEdge(start, start);
    cur_step.origin.time = merged_step.origin.time;
    cur_step.destination.time = merged_step.origin.time;
    auto ExtendPath = [&](StopId a, StopId b) -> bool {
      std::optional<Step> next_step =
          SelectBestNextStep(cur_step, completed.MergedStepsBetween(a, b));
      if (!next_step.has_value()) {
        return false;
      }
      path.steps.push_back(*next_step);
      cur_step = *next_step;
      return true;
    };
    for (size_t i = 0; i + 1 < stop_sequence.size(); ++i) {
      if (!ExtendPath(stop_sequence[i], stop_sequence[i + 1])) {
        break;
      }
    }

    result.push_back(path);
  }

  return result;
}

}  // namespace vats5
