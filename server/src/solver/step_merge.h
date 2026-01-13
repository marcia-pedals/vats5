#pragma once

#include <unordered_map>
#include <vector>

#include "solver/data.h"

namespace vats5 {

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
void MakeMinimalCover(std::vector<Step>& steps);

// Return a set of steps from stop A to stop C satisfying CheckSortedAndMinimal
// that are made of a step from ab followed by a step from bc.
//
// ab: All the steps from stop A to stop B, must satisfy CheckSortedAndMinimal.
// bc: All the steps from stop B to stop C, must satisfy CheckSortedAndMinimal.
std::vector<Step> PairwiseMergedSteps(
    const std::vector<Step>& ab, const std::vector<Step>& bc
);

// Merge two consecutive steps into a single step.
// The destination of `ab` should be the origin of `bc`.
Step MergedStep(Step ab, Step bc);

// Compute the merged step for a path of consecutive steps, with proper origin
// time calculation. The origin time adjustment handles flex paths that
// transition to fixed trips.
Step ConsecutiveMergedSteps(const std::vector<Step>& path);

}  // namespace vats5
