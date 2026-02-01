#include "solver/branch_and_bound2.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>
#include <algorithm>
#include <sstream>
#include <vector>
#include "rapidcheck/Assertions.h"
#include "rapidcheck/Log.h"
#include "solver/tarel_graph.h"
#include "solver/test_util/naive_solve.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {

TEST(BranchAndBound2Test, DummyTest) {
  EXPECT_EQ(DummyFunction(), 42);
}

RC_GTEST_PROP(BranchAndBound2Test, BranchPreservesSolutionSpace, ()) {
  int num_partitions = *rc::gen::inRange(1, 20);
  ProblemState state_orig = *GenProblemState(std::nullopt, rc::gen::construct<StepPartitionId>(rc::gen::inRange(0, num_partitions - 1)));

  // Helper to find optimal solution and log it.
  auto FindAndLogOptimal = [](const ProblemState& state, const std::string& label) -> int {
    std::vector<SolutionSpaceElement> solutions = EnumerateSolutionSpace(state);
    if (solutions.empty()) {
      RC_LOG() << label << ": no solution\n";
      return std::numeric_limits<int>::max();
    }

    auto best = std::min_element(solutions.begin(), solutions.end(),
      [](const SolutionSpaceElement& a, const SolutionSpaceElement& b) {
        return a.merged_step.DurationSeconds() < b.merged_step.DurationSeconds();
      });

    RC_LOG() << label << ": " << best->merged_step.DurationSeconds() << " [";
    for (size_t i = 0; i < best->actual_path.size(); ++i) {
      if (i > 0) RC_LOG() << " -> ";
      RC_LOG() << state.StopName(best->actual_path[i]);
    }
    RC_LOG() << "]\n";

    return best->merged_step.DurationSeconds();
  };

  // Get the required stops excluding START and END.
  std::vector<StopId> available_stops;
  for (StopId stop : state_orig.required_stops) {
    if (stop != state_orig.boundary.start && stop != state_orig.boundary.end) {
      available_stops.push_back(stop);
    }
  }

  RC_PRE(available_stops.size() >= 1);

  // Generate a random path of distinct stops.
  int n = static_cast<int>(available_stops.size());
  int path_length = *rc::gen::inRange(1, n + 1);
  std::vector<StopId> required_path = *rc::gen::unique<std::vector<StopId>>(path_length, rc::gen::elementOf(available_stops));

  RC_LOG() << "required_path: ";
  for (int i = 0; i < required_path.size(); ++i) {
    if (i > 0) RC_LOG() << " -> ";
    RC_LOG() << state_orig.StopName(required_path[i]);
  }
  RC_LOG() << "\n\n";

  // Build the constraints that partition the solution space:
  // - {required_path=[s1], forbid_next=s2}
  // - {required_path=[s1, s2], forbid_next=s3}
  // - ...
  // - {required_path=[s1, ..., s(n-1)], forbid_next=sn}
  // - {required_path=[s1, ..., sn], forbid_next=nullopt}
  std::vector<ElaborateConstraint> constraints;
  for (int i = 1; i <= required_path.size(); ++i) {
    ElaborateConstraint constraint;
    constraint.required_path = std::vector<StopId>(required_path.begin(), required_path.begin() + i);
    if (i < required_path.size()) {
      constraint.forbid_next = required_path[i];
    } else {
      constraint.forbid_next = std::nullopt;
    }
    constraints.push_back(constraint);
  }

  // Compute optimal for original state.
  int opt_orig = FindAndLogOptimal(state_orig, "original");

  // Apply each constraint and compute optimal for each branch.
  int min_branch_opt = std::numeric_limits<int>::max();
  for (int i = 0; i < constraints.size(); ++i) {
    const ElaborateConstraint& constraint = constraints[i];
    ProblemState state_branch = ApplyConstraint(state_orig, constraint);

    std::stringstream label;
    label << "branch " << i << " (path length " << constraint.required_path.size();
    if (constraint.forbid_next.has_value()) {
      label << ", forbid " << state_orig.StopName(constraint.forbid_next.value());
    }
    label << ")";

    int opt_branch = FindAndLogOptimal(state_branch, label.str());
    min_branch_opt = std::min(min_branch_opt, opt_branch);
  }

  RC_ASSERT(opt_orig == min_branch_opt);
}

// bool TourShouldBeInConstrainedSpace(const ElaborateConstraint& constraint, std::vector<StopId> tour) {
//   auto it = tour.begin();
//   while ((it = std::search(it, tour.end(), constraint.required_path.begin(), constraint.required_path.end())) != tour.end()) {

//   }
// }

// bool TourInConstrainedSpace(const ProblemState& base, const ElaborateConstraint& constraint, std::vector<StopId> tour) {
//   auto it = tour.begin();
//   while ((it = std::search(it, tour.end(), constraint.required_path.begin(), constraint.required_path.end())) != tour.end()) {

//   }

//   ProblemState state = ApplyConstraint(base, constraint);
// }

struct StepSpec {
  std::string origin;
  std::string destination;
  int start_time;
  int end_time;
};

struct FindSubpathsTestData {
  ProblemState state;
  std::vector<TarelEdge> tour;
  std::unordered_map<std::string, StopId> stop_ids;

  StopId GetStopId(const std::string& name) const {
    return stop_ids.at(name);
  }

  std::string GetStopName(StopId id) const {
    for (const auto& [name, stop_id] : stop_ids) {
      if (stop_id == id) return name;
    }
    return "?";
  }
};

// Check that subpaths match expected paths. Each expected path is a list of steps.
void ExpectSubpaths(
    const FindSubpathsTestData& data,
    const std::vector<Path>& subpaths,
    const std::vector<std::vector<StepSpec>>& expected) {
  ASSERT_EQ(subpaths.size(), expected.size())
      << "Expected " << expected.size() << " subpaths, got " << subpaths.size();

  // For each expected path, find a matching subpath.
  std::vector<bool> matched(subpaths.size(), false);
  for (const auto& expected_steps : expected) {
    bool found = false;
    for (size_t i = 0; i < subpaths.size(); ++i) {
      if (matched[i]) continue;
      const Path& path = subpaths[i];
      if (path.steps.size() != expected_steps.size()) continue;

      bool steps_match = true;
      for (size_t j = 0; j < expected_steps.size(); ++j) {
        const Step& step = path.steps[j];
        const StepSpec& spec = expected_steps[j];
        if (data.GetStopName(step.origin.stop) != spec.origin ||
            data.GetStopName(step.destination.stop) != spec.destination ||
            step.origin.time.seconds != spec.start_time ||
            step.destination.time.seconds != spec.end_time) {
          steps_match = false;
          break;
        }
      }
      if (steps_match) {
        matched[i] = true;
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "Expected path not found";
  }
}

FindSubpathsTestData SetupFindSubpathsTest(
    const std::vector<StepSpec>& step_specs,
    const std::vector<std::string>& tour_stops) {
  // Collect unique stop names and assign StopIds.
  std::unordered_map<std::string, StopId> stop_ids;
  auto get_or_create_stop = [&](const std::string& name) -> StopId {
    auto it = stop_ids.find(name);
    if (it != stop_ids.end()) return it->second;
    StopId id{static_cast<int>(stop_ids.size())};
    stop_ids[name] = id;
    return id;
  };

  // Create steps from specs.
  TripId trip{1};
  StepPartitionId partition{0};
  std::vector<Step> steps;
  for (const auto& spec : step_specs) {
    StopId origin = get_or_create_stop(spec.origin);
    StopId dest = get_or_create_stop(spec.destination);
    steps.push_back(Step::PrimitiveScheduled(
      origin, dest,
      TimeSinceServiceStart{spec.start_time},
      TimeSinceServiceStart{spec.end_time},
      trip, partition
    ));
  }

  // Ensure tour stops exist.
  for (const auto& name : tour_stops) {
    get_or_create_stop(name);
  }

  // Build stop_names map.
  std::unordered_map<StopId, std::string> stop_names;
  std::unordered_set<StopId> all_stops;
  for (const auto& [name, id] : stop_ids) {
    stop_names[id] = name;
    all_stops.insert(id);
  }

  // Build ProblemState.
  StepsAdjacencyList minimal = MakeAdjacencyList(steps);
  StopId start = stop_ids.at(tour_stops.front());
  StopId end = stop_ids.at(tour_stops.back());
  ProblemState state = MakeProblemState(
    minimal,
    ProblemBoundary{start, end},
    all_stops,
    stop_names,
    {}
  );

  // Build tour edges.
  std::vector<TarelEdge> tour;
  for (size_t i = 0; i + 1 < tour_stops.size(); ++i) {
    StopId from = stop_ids.at(tour_stops[i]);
    StopId to = stop_ids.at(tour_stops[i + 1]);
    tour.push_back(TarelEdge{
      TarelState{from, partition},
      TarelState{to, partition},
      0, {}, {}
    });
  }

  return FindSubpathsTestData{std::move(state), std::move(tour), std::move(stop_ids)};
}

TEST(BranchAndBound2Test, FindSubpathsBasic) {
  auto data = SetupFindSubpathsTest(
    {{"A", "B", 0, 100}, {"B", "C", 100, 200}},
    {"A", "B", "C"}
  );
  ExpectSubpaths(data, FindSubpaths(data.state, data.tour), {
    {{"A", "B", 0, 100}, {"B", "C", 100, 200}}
  });
}

TEST(BranchAndBound2Test, FindSubpathsEnds) {
  auto data = SetupFindSubpathsTest(
    {{"A", "B", 0, 100}},
    {"A", "B", "C"}
  );
  ExpectSubpaths(data, FindSubpaths(data.state, data.tour), {
    {{"A", "B", 0, 100}}
  });
}

TEST(BranchAndBound2Test, FindSubpathsBegins) {
  auto data = SetupFindSubpathsTest(
    {{"B", "C", 100, 200}},
    {"A", "B", "C"}
  );
  ExpectSubpaths(data, FindSubpaths(data.state, data.tour), {
    {{"B", "C", 100, 200}}
  });
}

TEST(BranchAndBound2Test, FindSubpathsMultiRoute) {
  auto data = SetupFindSubpathsTest(
    {
      {"A", "B", 0, 100},
      {"A", "X", 20, 80},
      {"X", "B", 80, 120},
    },
    {"A", "B"}
  );
  ExpectSubpaths(data, FindSubpaths(data.state, data.tour), {
    {{"A", "B", 0, 100}},
    {{"A", "X", 20, 80}, {"X", "B", 80, 120}},
  });
}

}  // namespace vats5
