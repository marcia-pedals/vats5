#include "solver/embiggener.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <unordered_set>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"

namespace vats5 {
namespace {

using testing::AllOf;
using testing::ElementsAre;
using testing::Field;
using testing::Matcher;

testing::Matcher<const EmbiggenerEdge&> Edge(auto... steps) {
  return AllOf(
      Field(&EmbiggenerEdge::weight, 0),
      Field(&EmbiggenerEdge::steps, ElementsAre(steps...))
  );
}

template <typename K, typename V>
void ExpectMap(
    const std::unordered_map<K, V>& actual,
    std::vector<std::pair<K, Matcher<const V&>>> expected
) {
  std::unordered_set<K> expected_keys;
  for (const auto& [key, matcher] : expected) {
    expected_keys.insert(key);
    if (actual.contains(key)) {
      EXPECT_THAT(actual.at(key), matcher) << "at key " << key;
    } else {
      ADD_FAILURE() << "missing key " << key;
    }
  }
  for (const auto& [key, _] : actual) {
    if (!expected_keys.contains(key)) {
      ADD_FAILURE() << "unexpected key " << key << " = " << actual.at(key);
    }
  }
}

FlatStep FS(int t0, int t1) {
  return FlatStep{TimeSinceServiceStart{t0}, TimeSinceServiceStart{t1}};
}

constexpr StopId S_A{0}, S_B{1}, S_C{2}, S_START{3}, S_END{4};

// Build a small problem state with stops A(0), B(1), C(2), START(3), END(4).
//
// A -> B at 0, 100, 200 - duration 10
// B -> A at 10, 110, 210 - duration 10
//
// B -> C at 0, 100, 200 - duration 20
// C -> B at 20, 120, 220 - duration 20
//
// Plus boundary flex edges START->* and *->END (duration 0).
ProblemState MakeTestProblemState() {
  std::vector<Step> steps;
  for (int t_start : {0, 100, 200}) {
    steps.push_back(
        Step::PrimitiveScheduled(
            S_A,
            S_B,
            TimeSinceServiceStart{t_start},
            TimeSinceServiceStart{t_start + 10},
            TripId{t_start}
        )
    );
    steps.push_back(
        Step::PrimitiveScheduled(
            S_B,
            S_A,
            TimeSinceServiceStart{t_start + 10},
            TimeSinceServiceStart{t_start + 20},
            TripId{t_start}
        )
    );

    steps.push_back(
        Step::PrimitiveScheduled(
            S_B,
            S_C,
            TimeSinceServiceStart{t_start},
            TimeSinceServiceStart{t_start + 20},
            TripId{t_start}
        )
    );
    steps.push_back(
        Step::PrimitiveScheduled(
            S_C,
            S_B,
            TimeSinceServiceStart{t_start + 20},
            TimeSinceServiceStart{t_start + 40},
            TripId{t_start}
        )
    );
  }

  std::unordered_set<StopId> stops = {S_A, S_B, S_C};
  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos;
  stop_infos[S_A] = ProblemStateStopInfo{GtfsStopId{"a"}, "A"};
  stop_infos[S_B] = ProblemStateStopInfo{GtfsStopId{"b"}, "B"};
  stop_infos[S_C] = ProblemStateStopInfo{GtfsStopId{"c"}, "C"};

  ProblemBoundary boundary{.start = S_START, .end = S_END};
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

TEST(EmbiggenerTest, MakeEmbiggenerState_AllAllowed) {
  // MakeEmbiggenerState works correctly on the MakeTestProblemState when
  // everything is allowed.
  ProblemState problem = MakeTestProblemState();
  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());
  std::vector<PointBound> known_points{
      PointBound{
          problem.boundary.start,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{0}
      },
      PointBound{
          problem.boundary.end,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{1000}
      },
  };
  EmbiggenerState state =
      MakeEmbiggenerState(problem, completed, known_points, {});

  ExpectMap(
      state.edges,
      {
          {{S_A, S_B}, Edge(FS(0, 10), FS(20, 110), FS(120, 210))},
          {{S_B, S_A},
           Edge(
               FS(0, 20),
               FS(10, 20),
               FS(40, 120),
               FS(110, 120),
               FS(140, 220),
               FS(210, 220)
           )},
          {{S_A, S_C}, Edge(FS(0, 120), FS(20, 220))},
          {{S_B, S_C},
           Edge(
               FS(0, 20), FS(10, 120), FS(40, 120), FS(110, 220), FS(140, 220)
           )},
          {{S_C, S_B}, Edge(FS(0, 40), FS(20, 40), FS(120, 140), FS(220, 240))},
          {{S_C, S_A}, Edge(FS(0, 120), FS(20, 120), FS(120, 220))},
          {{S_START, S_A}, Edge(FS(0, 0))},
          {{S_START, S_B}, Edge(FS(0, 0))},
          {{S_START, S_C}, Edge(FS(0, 0))},
          {{S_A, S_END},
           Edge(FS(0, 0), FS(20, 20), FS(120, 120), FS(220, 220))},
          {{S_B, S_END},
           Edge(
               FS(0, 0),
               FS(10, 10),
               FS(40, 40),
               FS(110, 110),
               FS(140, 140),
               FS(210, 210),
               FS(240, 240)
           )},
          {{S_C, S_END},
           Edge(FS(0, 0), FS(20, 20), FS(120, 120), FS(220, 220))},
          {{S_START, S_END}, Edge(FS(0, 0))},
      }
  );
}

TEST(EmbiggenerTest, MakeEmbiggenerState_ForbidOnePoint) {
  // MakeEmbiggenerState works correctly on the MakeTestProblemState when
  // we forbid one particular point.
  ProblemState problem = MakeTestProblemState();
  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());
  std::vector<PointBound> known_points{
      PointBound{
          problem.boundary.start,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{0}
      },
      PointBound{
          problem.boundary.end,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{1000}
      },
  };
  std::unordered_set<PointInstant> forbidden_points{
      PointInstant{S_B, TimeSinceServiceStart{110}}
  };
  EmbiggenerState state =
      MakeEmbiggenerState(problem, completed, known_points, forbidden_points);

  ExpectMap(
      state.edges,
      {
          {{S_A, S_B}, Edge(FS(0, 10), FS(120, 210))},
          {{S_B, S_A},
           Edge(
               FS(0, 20), FS(10, 20), FS(40, 120), FS(140, 220), FS(210, 220)
           )},
          {{S_A, S_C}, Edge(FS(0, 120), FS(20, 220))},
          {{S_B, S_C}, Edge(FS(0, 20), FS(10, 120), FS(40, 120), FS(140, 220))},
          {{S_C, S_B}, Edge(FS(0, 40), FS(20, 40), FS(120, 140), FS(220, 240))},
          {{S_C, S_A}, Edge(FS(0, 120), FS(20, 120), FS(120, 220))},
          {{S_START, S_A}, Edge(FS(0, 0))},
          {{S_START, S_B}, Edge(FS(0, 0))},
          {{S_START, S_C}, Edge(FS(0, 0))},
          {{S_A, S_END},
           Edge(FS(0, 0), FS(20, 20), FS(120, 120), FS(220, 220))},
          {{S_B, S_END},
           Edge(
               FS(0, 0),
               FS(10, 10),
               FS(40, 40),
               FS(140, 140),
               FS(210, 210),
               FS(240, 240)
           )},
          {{S_C, S_END},
           Edge(FS(0, 0), FS(20, 20), FS(120, 120), FS(220, 220))},
          {{S_START, S_END}, Edge(FS(0, 0))},
      }
  );
}

TEST(EmbiggenerTest, MakeEmbiggenerState_MustNotEndBefore200) {
  // MakeEmbiggenerState works correctly on the MakeTestProblemState when
  // we must not end before 200. This allows everything except for ending before
  // 200 because we can keep going back and forth until t >= 200.
  ProblemState problem = MakeTestProblemState();
  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());
  std::vector<PointBound> known_points{
      PointBound{
          problem.boundary.start,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{0}
      },
      PointBound{
          problem.boundary.end,
          TimeSinceServiceStart{200},
          TimeSinceServiceStart{1000}
      },
  };
  EmbiggenerState state =
      MakeEmbiggenerState(problem, completed, known_points, {});

  ExpectMap(
      state.edges,
      {
          {{S_A, S_B}, Edge(FS(0, 10), FS(20, 110), FS(120, 210))},
          {{S_B, S_A},
           Edge(
               FS(0, 20),
               FS(10, 20),
               FS(40, 120),
               FS(110, 120),
               FS(140, 220),
               FS(210, 220)
           )},
          {{S_A, S_C}, Edge(FS(0, 120), FS(20, 220))},
          {{S_B, S_C},
           Edge(
               FS(0, 20), FS(10, 120), FS(40, 120), FS(110, 220), FS(140, 220)
           )},
          {{S_C, S_B}, Edge(FS(0, 40), FS(20, 40), FS(120, 140), FS(220, 240))},
          {{S_C, S_A}, Edge(FS(0, 120), FS(20, 120), FS(120, 220))},
          {{S_START, S_A}, Edge(FS(0, 0))},
          {{S_START, S_B}, Edge(FS(0, 0))},
          {{S_START, S_C}, Edge(FS(0, 0))},
          {{S_A, S_END}, Edge(FS(220, 220))},
          {{S_B, S_END}, Edge(FS(210, 210), FS(240, 240))},
          {{S_C, S_END}, Edge(FS(220, 220))},
      }
  );
}

TEST(EmbiggenerTest, MakeEmbiggenerState_MustEndBy100) {
  // MakeEmbiggenerState works correctly on the MakeTestProblemState when
  // we must reach the end by 100 -- this eliminates all the steps that arrive
  // >100.
  ProblemState problem = MakeTestProblemState();
  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());
  std::vector<PointBound> known_points{
      PointBound{
          problem.boundary.start,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{0}
      },
      PointBound{
          problem.boundary.end,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{100}
      },
  };
  EmbiggenerState state =
      MakeEmbiggenerState(problem, completed, known_points, {});

  ExpectMap(
      state.edges,
      {
          {{S_A, S_B}, Edge(FS(0, 10))},
          {{S_B, S_A}, Edge(FS(0, 20), FS(10, 20))},
          {{S_B, S_C}, Edge(FS(0, 20))},
          {{S_C, S_B}, Edge(FS(0, 40), FS(20, 40))},
          {{S_START, S_A}, Edge(FS(0, 0))},
          {{S_START, S_B}, Edge(FS(0, 0))},
          {{S_START, S_C}, Edge(FS(0, 0))},
          {{S_A, S_END}, Edge(FS(0, 0), FS(20, 20))},
          {{S_B, S_END}, Edge(FS(0, 0), FS(10, 10), FS(40, 40))},
          {{S_C, S_END}, Edge(FS(0, 0), FS(20, 20))},
          {{S_START, S_END}, Edge(FS(0, 0))},
      }
  );
}

TEST(EmbiggenerTest, MakeEmbiggenerState_KnownPoint_NoGoodPath) {
  // MakeEmbiggenerState works correctly on the MakeTestProblemState when
  // there is an intermediate known point that is not on any good path.
  ProblemState problem = MakeTestProblemState();
  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());
  std::vector<PointBound> known_points{
      PointBound{
          problem.boundary.start,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{0}
      },
      PointBound{
          S_B,
          TimeSinceServiceStart{110},
          TimeSinceServiceStart{110},
      },
      PointBound{
          problem.boundary.end,
          TimeSinceServiceStart{110},
          TimeSinceServiceStart{1000}
      },
  };
  EmbiggenerState state =
      MakeEmbiggenerState(problem, completed, known_points, {});
  ExpectMap(state.edges, {});
}

TEST(EmbiggenerTest, MakeEmbiggenerState_KnownPoint_GoodPath) {
  // MakeEmbiggenerState works correctly on the MakeTestProblemState when
  // there is an intermediate known point that is not on a good path.
  ProblemState problem = MakeTestProblemState();
  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());
  std::vector<PointBound> known_points{
      PointBound{
          problem.boundary.start,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{0}
      },
      PointBound{
          S_B,
          TimeSinceServiceStart{10},
          TimeSinceServiceStart{10},
      },
      PointBound{
          problem.boundary.end,
          TimeSinceServiceStart{10},
          TimeSinceServiceStart{1000}
      },
  };
  EmbiggenerState state =
      MakeEmbiggenerState(problem, completed, known_points, {});

  ExpectMap(
      state.edges,
      {
          {{S_A, S_B}, Edge(FS(0, 10))},
          {{S_B, S_A}, Edge(FS(10, 20))},
          {{S_A, S_C}, Edge(FS(20, 220))},
          {{S_B, S_C}, Edge(FS(10, 120))},
          {{S_C, S_A}, Edge(FS(120, 220))},
          {{S_START, S_A}, Edge(FS(0, 0))},
          {{S_A, S_END}, Edge(FS(20, 20), FS(220, 220))},
          {{S_B, S_END}, Edge(FS(10, 10))},
          {{S_C, S_END}, Edge(FS(120, 120), FS(220, 220))},
      }
  );
}

}  // namespace
}  // namespace vats5
