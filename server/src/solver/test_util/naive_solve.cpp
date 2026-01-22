#include "solver/test_util/naive_solve.h"

#include <algorithm>
#include <limits>
#include <span>
#include <vector>

#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/tarel_graph.h"

namespace vats5 {

int NaiveSolve(const ProblemState& state) {
  std::vector<StopId> middle_stops;
  for (StopId stop : state.required_stops) {
    if (stop != state.boundary.start && stop != state.boundary.end) {
      middle_stops.push_back(stop);
    }
  }
  std::sort(middle_stops.begin(), middle_stops.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });

  auto get_steps = [&](StopId from, StopId to) -> std::vector<Step> {
    std::span<const Path> paths = state.completed.PathsBetween(from, to);
    std::vector<Step> steps;
    for (const Path& path : paths) {
      steps.push_back(path.merged_step);
    }
    return steps;
  };

  int min_duration = std::numeric_limits<int>::max();
  do {
    std::vector<StopId> perm;
    perm.push_back(state.boundary.start);
    for (StopId stop : middle_stops) {
      perm.push_back(stop);
    }
    perm.push_back(state.boundary.end);

    std::vector<Step> accumulated = get_steps(perm[0], perm[1]);
    for (size_t i = 1; i < perm.size(); ++i) {
      std::vector<Step> edge_steps = get_steps(perm[i], perm[(i + 1) % perm.size()]);
      accumulated = PairwiseMergedSteps(accumulated, edge_steps);
    }

    for (const Step& step : accumulated) {
      min_duration = std::min(min_duration, step.DurationSeconds());
    }
  } while (std::next_permutation(middle_stops.begin(), middle_stops.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  }));

  return min_duration;
}

}  // namespace vats5
