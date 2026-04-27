#include "solver/embiggener.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <algorithm>
#include <optional>
#include <unordered_set>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"
#include "solver/test_util/problem_state_gen.h"

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

// Follow a tour (START -> perm[0] -> ... -> perm[n-1] -> END) through an
// EmbiggenerState. Returns the arrival times at each stop (including END), or
// nullopt if the tour can't be completed.
std::optional<std::vector<TimeSinceServiceStart>> FollowTour(
    const EmbiggenerState& state,
    StopId start,
    StopId end,
    const std::vector<StopId>& perm
) {
  std::vector<TimeSinceServiceStart> times;
  TimeSinceServiceStart cur_time{0};
  StopId cur_stop = start;

  auto advance = [&](StopId next_stop) -> bool {
    PlainEdge edge{cur_stop, next_stop};
    auto it = state.edges.find(edge);
    if (it == state.edges.end()) return false;
    for (const FlatStep& step : it->second.steps) {
      // Only follow steps that originate exactly at cur_time. If you get to a
      // stop and there isn't a step at the current time but there is a stop at
      // a later time, waiting is not allowed because TODO EXPLAIN.
      // It has something to do with the "KnownForbiddenPartitioning" property
      // -- when you partition a problem into a "known point" and "forbidden
      // point" subproblem, we don't want the "forbidden point" subproblem to be
      // able to do essentialy the same tour as the "known point" one by just
      // waiting for a later step that does not violate the "forbidden point".
      // This would technically be a tour that doesn't violate the "forbidden
      // point", but there is no point in considering it because the tour in the
      // "known point" subproblem is at least as good.
      if (step.origin_time == cur_time) {
        cur_time = step.destination_time;
        times.push_back(cur_time);
        cur_stop = next_stop;
        return true;
      }
    }
    return false;
  };

  for (StopId s : perm) {
    if (!advance(s)) return std::nullopt;
  }
  if (!advance(end)) return std::nullopt;
  return times;
}

RC_GTEST_PROP(EmbiggenerTest, KnownForbiddenPartition, ()) {
  ProblemState problem = *GenProblemState();
  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());

  StopId start = problem.boundary.start;
  StopId end = problem.boundary.end;

  // Base state with no intermediate known points or forbidden points.
  std::vector<PointBound> base_known_points{
      PointBound{start, TimeSinceServiceStart{0}, TimeSinceServiceStart{0}},
      PointBound{end, TimeSinceServiceStart{0}, TimeSinceServiceStart{100000}},
  };
  EmbiggenerState base_state =
      MakeEmbiggenerState(problem, completed, base_known_points, {});

  // Collect the group representatives (excluding boundary) for tour perms.
  std::unordered_set<StopId> rep_set = problem.required.GroupRepresentatives();
  rep_set.erase(start);
  rep_set.erase(end);
  std::vector<StopId> stops(rep_set.begin(), rep_set.end());
  std::sort(stops.begin(), stops.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });

  // Collect all arrival times at each tour stop in the base state.
  std::unordered_map<StopId, std::vector<TimeSinceServiceStart>> arrival_times;
  for (const auto& [edge, emb_edge] : base_state.edges) {
    if (rep_set.contains(edge.b)) {
      for (const FlatStep& step : emb_edge.steps) {
        arrival_times[edge.b].push_back(step.destination_time);
      }
    }
  }

  // Generate an intermediate point: a tour stop at a time it actually
  // gets arrived at in the base state.
  StopId p_stop = *rc::gen::elementOf(stops);
  RC_PRE(arrival_times.contains(p_stop) && !arrival_times[p_stop].empty());
  TimeSinceServiceStart p_time = *rc::gen::elementOf(arrival_times[p_stop]);
  PointInstant p{p_stop, p_time};
  RC_LOG() << "p = (" << p_stop << ", " << p_time << ")\n";

  // Sub-state where p is a known intermediate point.
  std::vector<PointBound> known_kps{
      PointBound{start, TimeSinceServiceStart{0}, TimeSinceServiceStart{0}},
      PointBound{p.s, p.t, p.t},
      PointBound{end, p_time, TimeSinceServiceStart{100000}},
  };
  EmbiggenerState known_state =
      MakeEmbiggenerState(problem, completed, known_kps, {});

  // Sub-state where p is a forbidden point.
  std::unordered_set<PointInstant> forbidden{p};
  EmbiggenerState forbidden_state =
      MakeEmbiggenerState(problem, completed, base_known_points, forbidden);

  // Check partition property for all permutations of the tour stops.
  do {
    auto base_result = FollowTour(base_state, start, end, stops);
    auto known_result = FollowTour(known_state, start, end, stops);
    auto forbidden_result = FollowTour(forbidden_state, start, end, stops);

    RC_LOG() << "  perm=[";
    for (size_t i = 0; i < stops.size(); ++i) {
      if (i > 0) RC_LOG() << ",";
      RC_LOG() << stops[i];
    }
    RC_LOG() << "] base="
             << (base_result ? std::to_string(base_result->back().seconds)
                             : "FAIL")
             << " known="
             << (known_result ? std::to_string(known_result->back().seconds)
                              : "FAIL")
             << " forbidden="
             << (forbidden_result
                     ? std::to_string(forbidden_result->back().seconds)
                     : "FAIL")
             << "\n";

    if (!base_result.has_value()) {
      // If the base can't complete the tour, neither can the sub-states.
      RC_ASSERT(!known_result.has_value());
      RC_ASSERT(!forbidden_result.has_value());
    } else {
      bool known_matches = known_result == base_result;
      bool forbidden_matches = forbidden_result == base_result;
      // Exactly one sub-state matches the base timings.
      RC_ASSERT(known_matches != forbidden_matches);
      // The non-matching sub-state either fails or takes at least as long.
      if (!known_matches && known_result.has_value()) {
        RC_ASSERT(known_result->back() > base_result->back());
      }
      if (!forbidden_matches && forbidden_result.has_value()) {
        RC_ASSERT(forbidden_result->back() > base_result->back());
      }
    }
  } while (std::next_permutation(
      stops.begin(), stops.end(), [](StopId a, StopId b) { return a.v < b.v; }
  ));
}

RC_GTEST_PROP(EmbiggenerTest, LocalEmbiggenIterative_LowerBound, ()) {
  ProblemState problem = *GenProblemState();
  // TODO: Handle problems with non-trivial stop groups. For now we override
  // the grouping so that every stop is its own representative.
  for (auto& [stop, rep] : problem.required.representative) {
    rep = stop;
  }
  StepsAdjacencyList completed =
      MakeAdjacencyList(problem.ComputeCompletedGraph().AllMergedSteps());

  StopId start = problem.boundary.start;
  StopId end = problem.boundary.end;

  std::vector<PointBound> known_points{
      PointBound{start, TimeSinceServiceStart{0}, TimeSinceServiceStart{0}},
      PointBound{end, TimeSinceServiceStart{0}, TimeSinceServiceStart{100000}},
  };
  EmbiggenerState state =
      MakeEmbiggenerState(problem, completed, known_points, {});

  // Initialize each edge's weight to the minimum step duration on that edge,
  // matching what refine.cpp does before embiggening.
  for (auto& [_, edge_data] : state.edges) {
    auto min_dur_step_it =
        std::ranges::min_element(edge_data.steps, {}, [](const FlatStep& step) {
          return step.DurationSeconds();
        });
    RC_ASSERT(min_dur_step_it != edge_data.steps.end());
    edge_data.weight = min_dur_step_it->DurationSeconds();
  }

  // Collect all edges in the state in a deterministic order. Exclude the
  // trivial START -> END edge, which LocalEmbiggenIterative cannot target.
  std::vector<PlainEdge> all_edges;
  for (const auto& [edge, _] : state.edges) {
    if (edge.a == start && edge.b == end) continue;
    all_edges.push_back(edge);
  }
  RC_PRE(!all_edges.empty());
  std::sort(all_edges.begin(), all_edges.end(), [](PlainEdge x, PlainEdge y) {
    if (x.a.v != y.a.v) return x.a.v < y.a.v;
    return x.b.v < y.b.v;
  });

  // Pick a random sequence of targets and embiggen each in turn.
  int num_targets = *rc::gen::inRange(1, 10);
  for (int i = 0; i < num_targets; ++i) {
    PlainEdge target = *rc::gen::elementOf(all_edges);
    int delta = LocalEmbiggenIterative(problem, state, target, 0, 100);
    state.edges.at(target).weight += delta;
    RC_LOG() << "embiggened " << target.a << "->" << target.b << " by " << delta
             << " (now " << state.edges.at(target).weight << ")\n";
  }

  // Collect non-boundary tour stops.
  std::unordered_set<StopId> rep_set = problem.required.GroupRepresentatives();
  rep_set.erase(start);
  rep_set.erase(end);
  std::vector<StopId> stops(rep_set.begin(), rep_set.end());
  std::sort(stops.begin(), stops.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });

  // For every permutation of the tour stops: the sum of edge weights along
  // (start -> perm[0] -> ... -> perm[n-1] -> end) must be a lower bound on the
  // actual duration of FollowTour along that perm.
  do {
    auto tour_result = FollowTour(state, start, end, stops);
    if (!tour_result.has_value()) continue;

    int sum_weights = 0;
    StopId cur = start;
    auto add_edge_weight = [&](StopId next) {
      auto it = state.edges.find(PlainEdge{cur, next});
      RC_ASSERT(it != state.edges.end());
      sum_weights += it->second.weight;
      cur = next;
    };
    for (StopId s : stops) add_edge_weight(s);
    add_edge_weight(end);

    int duration = tour_result->back().seconds;
    RC_LOG() << "  perm=[";
    for (size_t i = 0; i < stops.size(); ++i) {
      if (i > 0) RC_LOG() << ",";
      RC_LOG() << stops[i];
    }
    RC_LOG() << "] duration=" << duration << " sum_weights=" << sum_weights
             << "\n";
    RC_ASSERT(sum_weights <= duration);
  } while (std::next_permutation(
      stops.begin(), stops.end(), [](StopId a, StopId b) { return a.v < b.v; }
  ));
}

}  // namespace
}  // namespace vats5
