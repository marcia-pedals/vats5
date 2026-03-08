#include "solver/test_util/naive_solve.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"

namespace vats5 {

void EnumeratePermutations(
    const ProblemState& state, const SolutionSpaceCallback& callback
) {
  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();

  // Make the initial generating permutation, which is
  // START -> ... all non-boundary stops in order ... -> END.
  std::vector<StopId> gen_perm;
  gen_perm.push_back(state.boundary.start);
  for (StopId stop : state.required.AllFlat()) {
    if (stop != state.boundary.start && stop != state.boundary.end) {
      gen_perm.push_back(stop);
    }
  }
  gen_perm.push_back(state.boundary.end);
  std::sort(gen_perm.begin() + 1, gen_perm.end() - 1);

  // Now loop over all generating permutations using std::next_permutation to
  // cycle through permutations of all interior elements.
  do {
    // Close the tour by appending START, so the last edge is END -> START.
    std::vector<StopId> tour_sequence = gen_perm;
    tour_sequence.push_back(state.boundary.start);

    std::vector<Path> paths =
        ComputeMinimalFeasiblePathsAlong(tour_sequence, completed);

    for (Path& path : paths) {
      callback(
          SolutionSpaceElement{
              .generating_permutation = gen_perm,
              .path = std::move(path),
          }
      );
    }
  } while (std::next_permutation(gen_perm.begin() + 1, gen_perm.end() - 1));
}

void EnumerateSolutionSpace(
    const ProblemState& state, const SolutionSpaceCallback& callback
) {
  std::vector<std::vector<StopId>> groups = state.required.Groups();

  // Enumerate all combinations: for each group, try each member.
  std::vector<int> choices(groups.size(), 0);

  while (true) {
    // Build a RequiredStops with one chosen member per group (no groups).
    RequiredStops modified_required;
    modified_required.representative[state.boundary.start] =
        state.boundary.start;
    modified_required.representative[state.boundary.end] = state.boundary.end;
    for (size_t g = 0; g < groups.size(); ++g) {
      StopId chosen = groups[g][choices[g]];
      modified_required.representative[chosen] = chosen;
    }

    ProblemState modified = state.WithRequired(modified_required);

    EnumeratePermutations(modified, callback);

    // Advance to next combination.
    bool advanced = false;
    for (int i = static_cast<int>(groups.size()) - 1; i >= 0; --i) {
      choices[i]++;
      if (choices[i] < static_cast<int>(groups[i].size())) {
        advanced = true;
        break;
      }
      choices[i] = 0;
    }
    if (!advanced) break;
  }
}

int BruteForceSolveOptimalDuration(const ProblemState& state) {
  int best = std::numeric_limits<int>::max();
  EnumerateSolutionSpace(state, [&](const SolutionSpaceElement& elem) {
    best = std::min(best, elem.path.DurationSeconds());
  });
  return best;
}

}  // namespace vats5
