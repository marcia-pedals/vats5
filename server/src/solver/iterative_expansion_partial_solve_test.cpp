#include "solver/iterative_expansion_partial_solve.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rapidcheck/Assertions.h"
#include "rapidcheck/Log.h"
#include "solver/concorde.h"
#include "solver/data.h"
#include "solver/tarel_graph.h"
#include "solver/test_util/problem_state_gen.h"
#include "solver/tour_paths.h"

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

// A scheduled step from `origin_time` to `destination_time`, on its own trip.
Step Scheduled(
    int origin, int destination, int origin_time, int destination_time, int trip
) {
  return Step::PrimitiveScheduled(
      StopId{origin},
      StopId{destination},
      TimeSinceServiceStart{origin_time},
      TimeSinceServiceStart{destination_time},
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

// The solvers solve the same partial problem, so whatever else differs
// between them, the duration they achieve must not.
RC_GTEST_PROP(
    PartialSolveTest, BruteForceAndBranchAndBoundAgreeOnDuration, ()
) {
  ProblemState state = *GenProblemState();
  std::unordered_set<StopId> subset = GenRequiredSubset(state);

  RC_LOG() << "subset size " << subset.size() << "\n";

  PartialSolution brute = PartialSolveBruteForce(subset, state);
  // Discard cases where the tarel TSP tour splits a stop's states into
  // separate visits, which branch and bound cannot solve yet (see
  // FarApartAlternateStopGroupThrowsInvalidTourStructure below).
  PartialSolution bnb;
  try {
    bnb = PartialSolveBranchAndBound(
        MakePartialProblemState(subset, state), state, 0, nullptr
    );
  } catch (const InvalidTourStructure&) {
    RC_DISCARD("InvalidTourStructure");
  }
  PartialSolution held_karp =
      PartialSolveHeldKarp(MakePartialProblemState(subset, state), state);

  RC_LOG() << "brute " << OptimalDuration(brute) << " over "
           << brute.paths.size() << " paths\n";
  RC_LOG() << "bnb " << OptimalDuration(bnb) << " over " << bnb.paths.size()
           << " paths\n";
  RC_LOG() << "held-karp " << OptimalDuration(held_karp) << " over "
           << held_karp.paths.size() << " paths\n";

  RC_ASSERT(OptimalDuration(brute) == OptimalDuration(bnb));
  RC_ASSERT(OptimalDuration(brute) == OptimalDuration(held_karp));
}

// Whichever solver produced it, a returned path must actually run from START
// to END and take the duration the solution was solved to.
RC_GTEST_PROP(PartialSolveTest, ReturnedPathsAreOptimalStartToEnd, ()) {
  ProblemState state = *GenProblemState();
  std::unordered_set<StopId> subset = GenRequiredSubset(state);

  PartialSolution bnb;
  try {
    bnb = PartialSolveBranchAndBound(
        MakePartialProblemState(subset, state), state, 0, nullptr
    );
  } catch (const InvalidTourStructure&) {
    RC_DISCARD("InvalidTourStructure");
  }

  for (const PartialSolution& solution :
       {PartialSolveBruteForce(subset, state),
        bnb,
        PartialSolveHeldKarp(MakePartialProblemState(subset, state), state)}) {
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

  PartialSolution bnb;
  try {
    bnb = PartialSolveBranchAndBound(
        MakePartialProblemState(subset, state), state, 0, nullptr
    );
  } catch (const InvalidTourStructure&) {
    RC_DISCARD("InvalidTourStructure");
  }

  for (const PartialSolution& solution :
       {PartialSolveBruteForce(subset, state),
        bnb,
        PartialSolveHeldKarp(MakePartialProblemState(subset, state), state)}) {
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

// An alternate-stop group whose members are further apart than
// kCycleEdgeWeight's reward for traversing the group's states consecutively in
// the tarel TSP graph: the optimal TSP tour splits the group into separate
// visits, entering as one member but leaving with the other member's edges,
// and extracting the tour throws out of the solve. This pins the known
// limitation (https://github.com/marcia-pedals/vats5/issues/116) that the
// property tests above discard when a generated case hits it.
TEST(PartialSolveTest, FarApartAlternateStopGroupThrowsInvalidTourStructure) {
  ProblemState state = MakeState(
      4,
      {
          Scheduled(0, 1, 0, 0, 0),
          Scheduled(1, 0, 0, 0, 1),
          Scheduled(1, 2, 20 * 60, 40 * 60, 2),
          Scheduled(2, 3, 40 * 60, 60 * 60, 3),
          Scheduled(3, 0, 60 * 60, 80 * 60, 4),
      }
  );
  state.required.representative[StopId{2}] = StopId{1};

  std::unordered_set<StopId> subset =
      WholeGroups(state, {StopId{0}, StopId{1}, StopId{3}});
  EXPECT_THROW(
      PartialSolveBranchAndBound(
          MakePartialProblemState(subset, state), state, 0, nullptr
      ),
      InvalidTourStructure
  );
}

// The tour of `stops`, with the boundary around it.
std::vector<StopId> Tour(
    const ProblemState& state, const std::vector<StopId>& stops
) {
  std::vector<StopId> tour{state.boundary.start};
  tour.insert(tour.end(), stops.begin(), stops.end());
  tour.push_back(state.boundary.end);
  return tour;
}

PartialSolution ExtendTour(
    const ProblemState& state, const std::vector<StopId>& tour, StopId new_stop
) {
  MinimalPathSetCache cache(state.minimal);
  return NaivelyExtendPartialSolution(
      tour, TourPathSets::Compute(tour, cache), new_stop, cache
  );
}

TEST(NaivelyExtendPartialSolutionTest, InsertsTheNewStopWhereItIsCheapest) {
  ProblemState state = LineState();
  // b belongs between a and c: either end of the tour would need one of the
  // expensive backwards edges.
  PartialSolution extended =
      ExtendTour(state, Tour(state, {StopId{0}, StopId{2}}), StopId{1});

  ASSERT_FALSE(extended.paths.empty());
  EXPECT_EQ(OptimalDuration(extended), 200);
  for (const PartialSolutionPath& path : extended.paths) {
    EXPECT_EQ(
        TourNames(state, path.subset_tour),
        (std::vector<std::string>{"START", "a", "b", "c", "END"})
    );
  }
}

TEST(NaivelyExtendPartialSolutionTest, KeepsTheOrderOfTheTourItExtends) {
  ProblemState state = LineState();
  // The tour is in the expensive order, and inserting a stop can't fix that:
  // wherever b goes, one of c -> a's 1000 or c -> b -> a's 2000 is paid.
  PartialSolution extended =
      ExtendTour(state, Tour(state, {StopId{2}, StopId{0}}), StopId{1});

  ASSERT_FALSE(extended.paths.empty());
  EXPECT_EQ(OptimalDuration(extended), 1100);
  for (const PartialSolutionPath& path : extended.paths) {
    std::vector<std::string> names = TourNames(state, path.subset_tour);
    EXPECT_LT(
        std::ranges::find(names, "c") - names.begin(),
        std::ranges::find(names, "a") - names.begin()
    );
  }
}

TEST(NaivelyExtendPartialSolutionTest, ExtendsTheEmptyTour) {
  ProblemState state = LineState();
  // Only one position to insert into, and no prefix or suffix around it.
  PartialSolution extended = ExtendTour(state, Tour(state, {}), StopId{1});

  ASSERT_FALSE(extended.paths.empty());
  EXPECT_EQ(OptimalDuration(extended), 0);
  for (const PartialSolutionPath& path : extended.paths) {
    EXPECT_EQ(
        TourNames(state, path.subset_tour),
        (std::vector<std::string>{"START", "b", "END"})
    );
  }
}

TEST(NaivelyExtendPartialSolutionTest, UnreachableNewStopHasNoPaths) {
  // Nothing but the boundary connects to c, so no position works: inserting it
  // before a has no c -> a and inserting it after has no a -> c.
  ProblemState state = MakeState(3, {Flex(0, 1, 100, 0), Flex(1, 0, 100, 1)});
  PartialSolution extended =
      ExtendTour(state, Tour(state, {StopId{0}}), StopId{2});

  EXPECT_TRUE(extended.paths.empty());
}

// The optimum over the insertion positions, each computed from scratch rather
// than from the tour's cached prefixes and suffixes.
int ReferenceExtendedDuration(
    const ProblemState& state, const std::vector<StopId>& tour, StopId new_stop
) {
  int result = std::numeric_limits<int>::max();
  for (size_t position = 1; position < tour.size(); ++position) {
    std::vector<StopId> extended_tour = tour;
    extended_tour.insert(extended_tour.begin() + position, new_stop);
    for (const Path& path :
         ComputeMinimalFeasiblePathsAlong(extended_tour, state.minimal)) {
      result = std::min(result, path.DurationSeconds());
    }
  }
  return result;
}

// A tour of `state` to extend, together with the stop to insert into it. The
// tour is in stop order, which the extension has to preserve whether or not it
// is a good one.
struct GeneratedExtension {
  std::vector<StopId> tour;
  StopId new_stop;
};
GeneratedExtension GenExtension(const ProblemState& state) {
  std::vector<StopId> representatives;
  for (StopId rep : state.required.GroupRepresentatives()) {
    if (rep == state.boundary.start || rep == state.boundary.end) {
      continue;
    }
    representatives.push_back(rep);
  }
  // GroupRepresentatives comes out of a hash set, so fix the order before
  // generating against it; otherwise a shrink can pick a different tour.
  std::ranges::sort(representatives, {}, &StopId::v);

  size_t new_stop_index =
      *rc::gen::inRange<size_t>(0, representatives.size()).as("new stop index");
  std::vector<StopId> stops;
  for (size_t i = 0; i < representatives.size(); ++i) {
    if (i != new_stop_index && *rc::gen::arbitrary<bool>()) {
      stops.push_back(representatives[i]);
    }
  }
  return {
      .tour = Tour(state, stops),
      .new_stop = representatives[new_stop_index],
  };
}

// Splicing into the tour's cached prefixes and suffixes must find the same
// optimum as rebuilding each candidate tour's paths from scratch.
RC_GTEST_PROP(
    NaivelyExtendPartialSolutionTest, AgreesWithExtendingFromScratch, ()
) {
  ProblemState state = *GenProblemState();
  GeneratedExtension extension = GenExtension(state);

  PartialSolution extended =
      ExtendTour(state, extension.tour, extension.new_stop);

  RC_LOG() << "tour of " << extension.tour.size() << " stops, "
           << extended.paths.size() << " extended paths\n";

  RC_ASSERT(
      OptimalDuration(extended) ==
      ReferenceExtendedDuration(state, extension.tour, extension.new_stop)
  );
}

// Every returned path must come from the tour it was given with the new stop
// spliced in somewhere in the middle, and must achieve the optimum.
RC_GTEST_PROP(
    NaivelyExtendPartialSolutionTest, ReturnedPathsFollowTheExtendedTour, ()
) {
  ProblemState state = *GenProblemState();
  GeneratedExtension extension = GenExtension(state);

  PartialSolution extended =
      ExtendTour(state, extension.tour, extension.new_stop);

  int optimal = OptimalDuration(extended);
  for (const PartialSolutionPath& path : extended.paths) {
    RC_ASSERT(path.path.DurationSeconds() == optimal);

    // The extended tour is the original with the new stop inserted, and never
    // in the first or last position, which belong to the boundary.
    std::vector<StopId> without_new_stop = path.subset_tour;
    auto new_stop_it = std::ranges::find(without_new_stop, extension.new_stop);
    RC_ASSERT(new_stop_it != without_new_stop.end());
    RC_ASSERT(new_stop_it != without_new_stop.begin());
    RC_ASSERT(new_stop_it != without_new_stop.end() - 1);
    without_new_stop.erase(new_stop_it);
    RC_ASSERT(without_new_stop == extension.tour);

    // And the path visits the whole of it, in order.
    std::vector<StopId> visited;
    path.path.VisitAllStops([&](StopId stop) { visited.push_back(stop); });
    auto tour_it = path.subset_tour.begin();
    for (StopId stop : visited) {
      if (tour_it != path.subset_tour.end() && stop == *tour_it) {
        ++tour_it;
      }
    }
    RC_ASSERT(tour_it == path.subset_tour.end());
  }
}

}  // namespace
}  // namespace vats5
