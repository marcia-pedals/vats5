#pragma once

#include <vector>
#include <unordered_map>

#include "solver/data.h"

namespace vats5 {

// Checks that steps is sorted by origin_time ascending and also by desintation_time ascending.
//
// This is "minimal" in the sense that: TODO explain.
bool CheckSortedAndMinimal(const std::vector<Step>& steps);

// Sorts steps in place by origin_time ascending.
void SortByOriginTime(std::vector<Step>& steps);

// Return a dominating set of steps from stop A to stop C that are made of a step from ab followed by a step from bc.
//
// ab: All the steps from stop A to stop B, sorted by origin_time ascending.
// bc: All the steps from stop B to stop C, sorted by origin_time ascending.
//
// See the top-level README.md for explanation of what a dominating set of steps is.
std::vector<Step> MergeSteps(const std::vector<Step>& ab, const std::vector<Step>& bc);

}  // namespace vats5