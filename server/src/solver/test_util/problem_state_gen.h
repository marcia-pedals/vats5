#pragma once

#include <rapidcheck.h>

#include <iostream>

#include "solver/tarel_graph.h"

namespace vats5 {

void showValue(const ProblemState& state, std::ostream& os);

enum class CycleIsFlex { kNo, kYes };

rc::Gen<ProblemState> GenProblemState(CycleIsFlex cycle_is_flex);

}  // namespace vats5
