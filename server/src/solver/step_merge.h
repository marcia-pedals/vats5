#pragma once

#include <vector>
#include <unordered_map>

#include "solver/data.h"

namespace vats5 {

// Checks that steps is sorted by origin_time ascending and also by desintation_time ascending.
//
// This is "minimal" in the sense that: TODO explain.
bool CheckSortedAndMinimal(const std::vector<Step>& steps);

// Sorts steps in-place by origin_time ascending, then by destination_time descending.
void SortByOriginAndDestinationTime(std::vector<Step>& steps);

// Updates steps in-place to be a minimal cover.
// A minimal cover (1) is a subset of the original steps, (2) satisfies CheckSortedAndMinimal, and
// (3) for any original step, there is a step in the minimal cover with origin_time no earlier and
// destination_time no later.
// Precondition: steps is sorted by origin_time ascending, then by destination_time descending.
void MakeMinimalCover(std::vector<Step>& steps);

// Return a minimal set of steps from stop A to stop C that are made of a step from ab followed by a step from bc.
//
// ab: All the steps from stop A to stop B, must be minimal
// bc: All the steps from stop B to stop C, must be minimal.
//
// Minimal is in the sense of CheckSortedAndMinimal above.
std::vector<Step> MergeSteps(const std::vector<Step>& ab, const std::vector<Step>& bc);

}  // namespace vats5