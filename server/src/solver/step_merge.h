#pragma once

#include <limits>
#include <unordered_map>
#include <vector>

#include "solver/data.h"

namespace vats5 {

// Provenance information for a merged step: which input steps it came from.
struct StepProvenance {
  size_t ab_index;
  size_t bc_index;
};

// Check that this is sorted in the sense that: Flex step(s) come first, and all
// non-flex steps are sorted by origin_time ascending.
//
// Check that this is minimal in the sense that: No step dominates any other
// step. "Dominates" means that you can leave at-or-after and still arrive
// at-or-before the dominated step.
//
// Some consequences of these properties are:
// - There is at most one flex step.
// - The steps are sorted by destination time ascending.
bool CheckSortedAndMinimal(const std::vector<Step>& steps);

// Sorts steps in-place: flex steps first (ordered by duration ascending),
// then non-flex steps by origin_time ascending, then by destination_time
// descending.
void SortSteps(std::vector<Step>& steps);

// Updates steps in-place to be a minimal cover.
// A minimal cover (1) is a subset of the original steps, (2) satisfies
// CheckSortedAndMinimal, and (3) for any original step, there is a step in the
// minimal cover with origin_time no earlier and destination_time no later.
// Precondition: steps is sorted in accordance with `SortSteps`.
//
// If parallel is non-null, it is compacted in the same way as steps,
// preserving correspondence between steps and associated data.
template<typename T = int>
void MakeMinimalCover(std::vector<Step>& steps, std::vector<T>* parallel = nullptr) {
  if (steps.size() <= 1) {
    return;
  }

  const StopId deletion_marker = StopId{std::numeric_limits<int>::min()};

  // Forward sweep through all the flex steps to delete all but the first
  // (shortest) one and to record the duration of the first (shortest) one.
  int flex_step_duration = std::numeric_limits<int>::max();
  for (size_t i = 0; i < steps.size() && steps[i].is_flex; ++i) {
    if (i == 0) {
      flex_step_duration = steps[i].FlexDurationSeconds();
    } else {
      steps[i].origin.stop = deletion_marker;
    }
  }

  // Backwards sweep through the non-flex steps: a step is dominated if there's
  // a later-departing step that arrives earlier OR if the flex step is
  // equal-or-shorter.
  int earliest_destination_time = std::numeric_limits<int>::max();
  for (int i = static_cast<int>(steps.size()) - 1; i >= 0 && !steps[i].is_flex;
       i--) {
    if (steps[i].destination.time.seconds >= earliest_destination_time ||
        steps[i].DurationSeconds() >= flex_step_duration) {
      // This step is dominated.
      steps[i].origin.stop = deletion_marker;
    } else {
      // This step is not dominated, update earliest destination time
      earliest_destination_time = steps[i].destination.time.seconds;
    }
  }

  // Remove marked steps in-place using two-pointer technique
  size_t write_pos = 0;
  for (size_t read_pos = 0; read_pos < steps.size(); read_pos++) {
    if (steps[read_pos].origin.stop != deletion_marker) {
      if (write_pos != read_pos) {
        steps[write_pos] = std::move(steps[read_pos]);
        if (parallel) {
          (*parallel)[write_pos] = std::move((*parallel)[read_pos]);
        }
      }
      write_pos++;
    }
  }

  steps.resize(write_pos);
  if (parallel) {
    parallel->resize(write_pos);
  }
}

// Return a set of steps from stop A to stop C satisfying CheckSortedAndMinimal
// that are made of a step from ab followed by a step from bc.
//
// ab: All the steps from stop A to stop B, must satisfy CheckSortedAndMinimal.
// bc: All the steps from stop B to stop C, must satisfy CheckSortedAndMinimal.
//
// If provenance is non-null, it is filled with one entry per result step
// indicating which indices in ab and bc were combined to produce it.
std::vector<Step> PairwiseMergedSteps(
    const std::vector<Step>& ab, const std::vector<Step>& bc,
    std::vector<StepProvenance>* provenance = nullptr
);

// Merge two consecutive steps into a single step.
// The destination of `ab` should be the origin of `bc`.
Step MergedStep(Step ab, Step bc);

// Compute the merged step for a path of consecutive steps, with proper origin
// time calculation. The origin time adjustment handles flex paths that
// transition to fixed trips.
Step ConsecutiveMergedSteps(const std::vector<Step>& path);

// Return the best (earliest arriving at destination) step from `candidates` following `cur`, if any exist.
// Preconditions:
// - cur.destination.stop == c.origin.stop for all candidates
// - candidates satisfies CheckSortedAndMinimal.
std::optional<Step> SelectBestNextStep(const Step cur, const std::vector<Step>& candidates);

}  // namespace vats5
