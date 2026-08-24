#include "solver/held_karp_dp.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include "rapidcheck/Log.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {

RC_GTEST_PROP(HeldKarpDpTest, SolveFindsOptimalValue, ()) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state = *GenProblemState(
      std::nullopt,
      rc::gen::construct<StepPartitionId>(
          rc::gen::inRange(0, num_partitions - 1)
      )
  );

  RC_ASSERT(
      BruteForceSolveOptimalDuration(state) ==
      HeldKarpDPSolve(state, &RC_LOG()).best_val
  );
}

}  // namespace vats5
