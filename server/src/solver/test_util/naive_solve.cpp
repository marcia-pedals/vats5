#include "solver/test_util/naive_solve.h"

#include <algorithm>
#include <limits>
#include <span>
#include <vector>

#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/tarel_graph.h"

namespace vats5 {

void EnumeratePermutations(
    const ProblemState& state, const SolutionSpaceCallback& callback
) {
  // Make the initial generating permutation, which is
  // START -> ... all non-boundary stops in order ... -> END.
  std::vector<StopId> gen_perm;
  gen_perm.push_back(state.boundary.start);
  for (StopId stop : state.required_stops) {
    if (stop != state.boundary.start && stop != state.boundary.end) {
      gen_perm.push_back(stop);
    }
  }
  gen_perm.push_back(state.boundary.end);
  std::sort(gen_perm.begin() + 1, gen_perm.end() - 1);

  // Now loop over all generating permutations using std::next_permutation to
  // cycle through permutations of all interior elements.
  do {
    std::vector<Step> accumulated_steps;
    std::vector<std::vector<StopId>> accumulated_actual_paths;
    for (size_t i = 0; i < gen_perm.size(); ++i) {
      std::span<const Path> edge_paths = state.completed.PathsBetween(
          gen_perm[i], gen_perm[(i + 1) % gen_perm.size()]
      );
      std::vector<Step> edge_steps;
      edge_steps.reserve(edge_paths.size());
      for (const Path& p : edge_paths) {
        edge_steps.push_back(p.merged_step);
      }

      if (i == 0) {
        accumulated_steps = edge_steps;
        for (const Path& p : edge_paths) {
          accumulated_actual_paths.push_back({});
          for (const Step& s : p.steps) {
            accumulated_actual_paths.back().push_back(s.origin.stop);
          }
          assert(accumulated_actual_paths.back().size() > 0);
          assert(accumulated_actual_paths.back()[0] == state.boundary.start);
        }
      } else {
        std::vector<StepProvenance> new_provenance;
        std::vector<Step> new_accumulated_steps =
            PairwiseMergedSteps(accumulated_steps, edge_steps, &new_provenance);
        std::vector<std::vector<StopId>> new_accumulated_actual_paths;
        for (const StepProvenance& step_provenance : new_provenance) {
          assert(step_provenance.ab_index < accumulated_actual_paths.size());
          assert(step_provenance.bc_index < edge_paths.size());
          new_accumulated_actual_paths.push_back(
              accumulated_actual_paths[step_provenance.ab_index]
          );
          for (const Step& s : edge_paths[step_provenance.bc_index].steps) {
            new_accumulated_actual_paths.back().push_back(s.origin.stop);
          }
          assert(
              new_accumulated_actual_paths.back()[0] == state.boundary.start
          );
        }
        accumulated_steps = std::move(new_accumulated_steps);
        accumulated_actual_paths = std::move(new_accumulated_actual_paths);
      }
      assert(accumulated_steps.size() == accumulated_actual_paths.size());
    }
    for (int i = 0; i < accumulated_steps.size(); ++i) {
      if (accumulated_steps[i].origin.time < TimeSinceServiceStart{0}) {
        // See function doc comment.
        continue;
      }
      callback(
          SolutionSpaceElement{
              .generating_permutation = gen_perm,
              .actual_path = accumulated_actual_paths[i],
              .merged_step = accumulated_steps[i],
          }
      );
    }
  } while (std::next_permutation(gen_perm.begin() + 1, gen_perm.end() - 1));
}

void EnumerateSolutionSpace(
    const ProblemState& state, const SolutionSpaceCallback& callback
) {
  // Build groups from alternate_stop: for every required non-boundary stop,
  // look up its rep (self if not in alternate_stop).
  std::unordered_map<StopId, std::vector<StopId>> groups_by_rep;
  for (StopId stop : state.required_stops) {
    if (stop == state.boundary.start || stop == state.boundary.end) continue;
    auto it = state.alternate_stop.find(stop);
    StopId rep = (it != state.alternate_stop.end()) ? it->second : stop;
    groups_by_rep[rep].push_back(stop);
  }
  for (auto& [rep, members] : groups_by_rep) {
    std::sort(members.begin(), members.end(), [](StopId a, StopId b) {
      return a.v < b.v;
    });
  }

  // Collect groups sorted by first member for determinism.
  std::vector<std::vector<StopId>> groups;
  for (auto& [rep, members] : groups_by_rep) {
    groups.push_back(std::move(members));
  }
  std::sort(groups.begin(), groups.end(), [](const auto& a, const auto& b) {
    return a[0].v < b[0].v;
  });

  // Enumerate all combinations: for each group, try each member.
  std::vector<int> choices(groups.size(), 0);

  while (true) {
    ProblemState modified = state;
    modified.required_stops = {state.boundary.start, state.boundary.end};
    for (size_t g = 0; g < groups.size(); ++g) {
      modified.required_stops.insert(groups[g][choices[g]]);
    }

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
    best = std::min(best, elem.merged_step.DurationSeconds());
  });
  return best;
}

}  // namespace vats5
