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

// Comparison function for sorting steps: flex steps first (ordered by duration
// ascending), then non-flex steps by origin_time ascending, then by
// destination_time descending.
inline bool SmallerStep(const Step& a, const Step& b) {
  // Flex steps come first
  if (a.is_flex != b.is_flex) {
    return a.is_flex > b.is_flex;  // flex first
  }

  // Among flex steps, sort by duration ascending
  if (a.is_flex && b.is_flex) {
    return a.FlexDurationSeconds() < b.FlexDurationSeconds();
  }

  // Among non-flex steps, sort by origin time then destination time
  if (a.origin.time.seconds != b.origin.time.seconds) {
    return a.origin.time.seconds < b.origin.time.seconds;
  }
  return a.destination.time.seconds > b.destination.time.seconds;
}

// Sorts steps in-place using stable sort: flex steps first (ordered by duration
// ascending), then non-flex steps by origin_time ascending, then by
// destination_time descending.
//
// If parallel is non-null, it is permuted in the same way as steps,
// preserving correspondence between steps and associated data.
template<typename T = int>
void SortSteps(std::vector<Step>& steps, std::vector<T>* parallel = nullptr) {
  if (parallel) {
    // Build index array and sort it
    std::vector<size_t> indices(steps.size());
    for (size_t i = 0; i < steps.size(); ++i) {
      indices[i] = i;
    }
    std::stable_sort(indices.begin(), indices.end(), [&steps](size_t a, size_t b) {
      return SmallerStep(steps[a], steps[b]);
    });

    // Apply permutation to both vectors
    std::vector<Step> sorted_steps(steps.size());
    std::vector<T> sorted_parallel(parallel->size());
    for (size_t i = 0; i < indices.size(); ++i) {
      sorted_steps[i] = std::move(steps[indices[i]]);
      sorted_parallel[i] = std::move((*parallel)[indices[i]]);
    }
    steps = std::move(sorted_steps);
    *parallel = std::move(sorted_parallel);
  } else {
    std::stable_sort(steps.begin(), steps.end(), SmallerStep);
  }
}

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

  // Forward sweep through all the flex steps to find the shortest duration
  // and keep only the last one with that duration.
  int flex_step_duration = std::numeric_limits<int>::max();
  size_t last_shortest_flex = 0;
  for (size_t i = 0; i < steps.size() && steps[i].is_flex; ++i) {
    int duration = steps[i].FlexDurationSeconds();
    if (duration <= flex_step_duration) {
      flex_step_duration = duration;
      last_shortest_flex = i;
    }
  }
  // Mark all flex steps except the last shortest one for deletion.
  for (size_t i = 0; i < steps.size() && steps[i].is_flex; ++i) {
    if (i != last_shortest_flex) {
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
