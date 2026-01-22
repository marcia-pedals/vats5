#pragma once

#include <rapidcheck.h>

#include <iostream>

#include "solver/tarel_graph.h"

namespace vats5 {

void showValue(const ProblemState& state, std::ostream& os);

enum class CycleIsFlex { kNo, kYes };

rc::Gen<ProblemState> GenProblemState(
    rc::Gen<CycleIsFlex> cycle_is_flex_gen = rc::gen::element(CycleIsFlex::kNo, CycleIsFlex::kYes));

}  // namespace vats5
