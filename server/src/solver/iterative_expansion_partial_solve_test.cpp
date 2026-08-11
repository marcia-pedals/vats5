#include "solver/iterative_expansion_partial_solve.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rapidcheck/Assertions.h"
#include "rapidcheck/Log.h"
#include "solver/data.h"
#include "solver/tarel_graph.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {
namespace {

// The duration the partial problem was solved to, or max if it is infeasible.
int OptimalDuration(const PartialSolution& solution) {
  int result = std::numeric_limits<int>::max();
  for (const PartialSolutionPath& path : solution.paths) {
    result = std::min(result, path.path.DurationSeconds());
  }
  return result;
}

// A problem over stops a..(a + num_stops - 1), each its own required group,
// with `steps` between them plus the boundary. The boundary stops are the two
// after the last one.
ProblemState MakeState(int num_stops, std::vector<Step> steps) {
  std::unordered_set<StopId> stops;
  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos;
  for (int i = 0; i < num_stops; ++i) {
    stops.insert(StopId{i});
    stop_infos[StopId{i}] = ProblemStateStopInfo{
        GtfsStopId{std::string(1, 'a' + i)}, std::string(1, 'a' + i)
    };
  }

  ProblemBoundary boundary{
      .start = StopId{num_stops},
      .end = StopId{num_stops + 1},
  };
  AddBoundary(steps, stops, stop_infos, boundary);

  RequiredStops required;
  for (StopId stop : stops) {
    required.representative[stop] = stop;
  }
  return MakeProblemState(
      MakeAdjacencyList(steps),
      boundary,
      std::move(required),
      stop_infos,
      {},
      {}
  );
}

// A flex step of `duration` seconds, on its own trip.
Step Flex(int origin, int destination, int duration, int trip) {
  return Step::PrimitiveFlex(
      StopId{origin},
      StopId{destination},
      duration,
      TripId{trip},
      StepPartitionId::NONE
  );
}

// The stops of `subset`, as their names, sorted.
std::vector<std::string> TourNames(
    const ProblemState& state, const std::vector<StopId>& tour
) {
  std::vector<std::string> names;
  for (StopId stop : tour) {
    names.push_back(state.StopName(stop));
  }
  return names;
}

// Every stop's whole group, for the group representatives selected by `keep`.
std::unordered_set<StopId> WholeGroups(
    const ProblemState& state, const std::vector<StopId>& keep
) {
  std::unordered_set<StopId> subset;
  for (StopId rep : keep) {
    state.required.VisitGroupStops(rep, [&](StopId stop) {
      subset.insert(stop);
    });
  }
  return subset;
}

// Three stops on a line, cheap forwards and expensive backwards, so a -> b ->
// c is the only good order to visit them in.
ProblemState LineState() {
  return MakeState(
      3,
      {
          Flex(0, 1, 100, 0),
          Flex(1, 2, 100, 1),
          Flex(0, 2, 1000, 2),
          Flex(1, 0, 1000, 3),
          Flex(2, 1, 1000, 4),
          Flex(2, 0, 1000, 5),
      }
  );
}

TEST(PartialSolveBruteForceTest, VisitsTheSubsetInTheCheapestOrder) {
  ProblemState state = LineState();
  PartialSolution solution = PartialSolveBruteForce(
      WholeGroups(state, {StopId{0}, StopId{1}, StopId{2}}), state
  );

  ASSERT_FALSE(solution.paths.empty());
  EXPECT_EQ(OptimalDuration(solution), 200);
  for (const PartialSolutionPath& path : solution.paths) {
    EXPECT_EQ(
        TourNames(state, path.subset_tour),
        (std::vector<std::string>{"START", "a", "b", "c", "END"})
    );
  }
}

// The subset says which stops the tour has to visit, not which ones the paths
// between them may pass through.
TEST(PartialSolveBruteForceTest, RoutesThroughStopsOutsideTheSubset) {
  ProblemState state = LineState();
  PartialSolution solution =
      PartialSolveBruteForce(WholeGroups(state, {StopId{0}, StopId{2}}), state);

  ASSERT_FALSE(solution.paths.empty());
  // a -> c goes via b for 200 rather than taking the direct 1000 edge.
  EXPECT_EQ(OptimalDuration(solution), 200);
  for (const PartialSolutionPath& path : solution.paths) {
    EXPECT_EQ(
        TourNames(state, path.subset_tour),
        (std::vector<std::string>{"START", "a", "c", "END"})
    );
  }
}

TEST(PartialSolveBruteForceTest, EmptySubsetGoesStraightToEnd) {
  ProblemState state = LineState();
  PartialSolution solution = PartialSolveBruteForce({}, state);

  ASSERT_FALSE(solution.paths.empty());
  EXPECT_EQ(OptimalDuration(solution), 0);
}

TEST(PartialSolveBruteForceTest, VisitsOneStopPerGroup) {
  // b and c are the same group, so a tour visits whichever is cheaper -- c,
  // which is a tenth of the distance from a.
  ProblemState state = MakeState(
      3,
      {
          Flex(0, 1, 100, 0),
          Flex(1, 0, 100, 1),
          Flex(0, 2, 10, 2),
          Flex(2, 0, 10, 3),
      }
  );
  state.required.representative[StopId{2}] = StopId{1};

  PartialSolution solution =
      PartialSolveBruteForce(WholeGroups(state, {StopId{0}, StopId{1}}), state);

  ASSERT_FALSE(solution.paths.empty());
  EXPECT_EQ(OptimalDuration(solution), 10);
  for (const PartialSolutionPath& path : solution.paths) {
    std::vector<std::string> names = TourNames(state, path.subset_tour);
    EXPECT_TRUE(std::ranges::contains(names, "c")) << names.size();
    EXPECT_FALSE(std::ranges::contains(names, "b"));
  }
}

TEST(PartialSolveBruteForceTest, InfeasibleSubsetHasNoPaths) {
  // Nothing reaches c, so no tour visits it. A dead end would not do:
  // AddBoundary adds a *->END step from every stop.
  ProblemState state = MakeState(3, {Flex(0, 1, 100, 0), Flex(1, 0, 100, 1)});

  PartialSolution solution = PartialSolveBruteForce(
      WholeGroups(state, {StopId{0}, StopId{1}, StopId{2}}), state
  );

  EXPECT_TRUE(solution.paths.empty());
}

// A subset of `state`'s required stops, holding whole groups, which is what
// both PartialSolve* functions require of it.
std::unordered_set<StopId> GenRequiredSubset(const ProblemState& state) {
  std::vector<StopId> representatives;
  for (StopId rep : state.required.GroupRepresentatives()) {
    if (rep == state.boundary.start || rep == state.boundary.end) {
      continue;
    }
    representatives.push_back(rep);
  }
  // GroupRepresentatives comes out of a hash set, so fix the order before
  // generating against it; otherwise a shrink can pick a different subset.
  std::ranges::sort(representatives, {}, &StopId::v);

  std::vector<StopId> keep;
  for (StopId rep : representatives) {
    if (*rc::gen::arbitrary<bool>()) {
      keep.push_back(rep);
    }
  }
  return WholeGroups(state, keep);
}

// The two solvers solve the same partial problem, so whatever else differs
// between them, the duration they achieve must not.
RC_GTEST_PROP(
    PartialSolveTest, BruteForceAndBranchAndBoundAgreeOnDuration, ()
) {
  ProblemState state = *GenProblemState();
  std::unordered_set<StopId> subset = GenRequiredSubset(state);

  RC_LOG() << "subset size " << subset.size() << "\n";

  PartialSolution brute = PartialSolveBruteForce(subset, state);
  PartialSolution bnb = PartialSolveBranchAndBound(subset, state, nullptr);

  RC_LOG() << "brute " << OptimalDuration(brute) << " over "
           << brute.paths.size() << " paths\n";
  RC_LOG() << "bnb " << OptimalDuration(bnb) << " over " << bnb.paths.size()
           << " paths\n";

  RC_ASSERT(OptimalDuration(brute) == OptimalDuration(bnb));
}

// Whichever solver produced it, a returned path must actually run from START
// to END and take the duration the solution was solved to.
RC_GTEST_PROP(PartialSolveTest, ReturnedPathsAreOptimalStartToEnd, ()) {
  ProblemState state = *GenProblemState();
  std::unordered_set<StopId> subset = GenRequiredSubset(state);

  for (const PartialSolution& solution :
       {PartialSolveBruteForce(subset, state),
        PartialSolveBranchAndBound(subset, state, nullptr)}) {
    int optimal = OptimalDuration(solution);
    for (const PartialSolutionPath& path : solution.paths) {
      RC_ASSERT(path.path.merged_step.origin.stop == state.boundary.start);
      RC_ASSERT(path.path.merged_step.destination.stop == state.boundary.end);
      RC_ASSERT(path.path.DurationSeconds() == optimal);
      RC_ASSERT(path.subset_tour.front() == state.boundary.start);
      RC_ASSERT(path.subset_tour.back() == state.boundary.end);
    }
  }
}

// The tour a path came from must visit every group of the subset, which is
// the whole point of a partial solution.
RC_GTEST_PROP(PartialSolveTest, ReturnedToursVisitEverySubsetGroup, ()) {
  ProblemState state = *GenProblemState();
  std::unordered_set<StopId> subset = GenRequiredSubset(state);

  std::unordered_set<StopId> subset_representatives;
  for (StopId stop : subset) {
    subset_representatives.insert(state.required.Representative(stop));
  }

  for (const PartialSolution& solution :
       {PartialSolveBruteForce(subset, state),
        PartialSolveBranchAndBound(subset, state, nullptr)}) {
    for (const PartialSolutionPath& path : solution.paths) {
      std::unordered_set<StopId> visited;
      path.path.VisitAllStops([&](StopId stop) {
        visited.insert(state.required.Representative(stop));
      });
      for (StopId rep : subset_representatives) {
        RC_ASSERT(visited.contains(rep));
      }
    }
  }
}

}  // namespace
}  // namespace vats5
