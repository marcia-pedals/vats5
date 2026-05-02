#include "solver/embiggener.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <algorithm>
#include <limits>
#include <map>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"
#include "solver/test_util/problem_state_gen.h"

namespace vats5 {
namespace {

using testing::ElementsAreArray;

// Returns the FlatSteps for `edge` derived from the state's 2-hop primitive
// paths, sorted by origin_time.
std::vector<FlatStep> EdgeFlatSteps(
    const EmbiggenerState& state, PlainEdge edge
) {
  std::vector<FlatStep> result;
  for (std::size_t idx : state.by_edge[state.EdgeIndex(edge)]) {
    const LocalEmbiggenState& p = state.primitive_paths[idx];
    if (p.path.size() != 2) continue;
    if (p.path[0].s != edge.a || p.path[1].s != edge.b) continue;
    result.push_back(FlatStep{p.path[0].t, p.path[1].t});
  }
  std::sort(
      result.begin(), result.end(), [](const FlatStep& a, const FlatStep& b) {
        return a.origin_time < b.origin_time;
      }
  );
  return result;
}

std::vector<FlatStep> Edge(std::same_as<FlatStep> auto... steps) {
  return std::vector<FlatStep>{steps...};
}

void ExpectEdges(
    const EmbiggenerState& state,
    std::vector<std::pair<PlainEdge, std::vector<FlatStep>>> expected
) {
  std::unordered_set<PlainEdge> expected_keys;
  for (const auto& [edge, expected_steps] : expected) {
    expected_keys.insert(edge);
    auto it = state.edges.find(edge);
    if (it == state.edges.end()) {
      ADD_FAILURE() << "missing edge " << edge;
      continue;
    }
    EXPECT_THAT(EdgeFlatSteps(state, edge), ElementsAreArray(expected_steps))
        << "steps at edge " << edge;
    int min_duration = std::numeric_limits<int>::max();
    for (const FlatStep& s : expected_steps) {
      min_duration = std::min(min_duration, s.DurationSeconds());
    }
    EXPECT_EQ(it->second.weight, min_duration) << "weight at edge " << edge;
  }
  for (const auto& [edge, _] : state.edges) {
    if (!expected_keys.contains(edge)) {
      ADD_FAILURE() << "unexpected edge " << edge;
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

  ExpectEdges(
      state,
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

  ExpectEdges(
      state,
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

  ExpectEdges(
      state,
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

  ExpectEdges(
      state,
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
  ExpectEdges(state, {});
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

  ExpectEdges(
      state,
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
    for (const FlatStep& step : EdgeFlatSteps(state, edge)) {
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
  for (const LocalEmbiggenState& p : base_state.primitive_paths) {
    if (p.path.size() != 2) continue;
    if (rep_set.contains(p.path[1].s)) {
      arrival_times[p.path[1].s].push_back(p.path[1].t);
    }
  }

  // Generate an intermediate point: a tour stop at a time it actually
  // gets arrived at in the base state.
  StopId p_stop = *rc::gen::elementOf(stops);
  RC_PRE(arrival_times.contains(p_stop) && !arrival_times[p_stop].empty());
  TimeSinceServiceStart p_time = *rc::gen::elementOf(arrival_times[p_stop]);
  PointInstant p{p_stop, p_time};
  RC_LOG() << "p = (" << problem.StopName(p_stop) << ", " << p_time << ")\n";

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
      RC_LOG() << problem.StopName(stops[i]);
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

RC_GTEST_PROP(EmbiggenerTest, LocalEmbiggenCorrect_LowerBound, ()) {
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
  EmbiggenerState state = MakeEmbiggenerState(
      problem,
      completed,
      known_points,
      {},
      EmbiggenerOptions{.include_nonzero_flex = true}
  );

  // Collect all edges in the state in a deterministic order. Exclude the
  // trivial START -> END edge, which LocalEmbiggenCorrect cannot target.
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
  std::vector<PlainEdge> targets = *rc::gen::nonEmpty(
      rc::gen::resize(
          10,
          rc::gen::container<std::vector<PlainEdge>>(
              rc::gen::elementOf(all_edges)
          )
      )
  );
  for (PlainEdge target : targets) {
    RC_LOG() << "target " << problem.StopName(target.a) << "->"
             << problem.StopName(target.b) << "\n";
    std::optional<int> delta = LocalEmbiggenCorrect(problem, state, target);
    if (delta.has_value()) {
      RC_LOG() << "embiggened " << problem.StopName(target.a) << "->"
               << problem.StopName(target.b) << " by " << *delta << " (now "
               << state.edges.at(target).weight << ")\n";
    } else {
      RC_LOG() << "eliminated " << problem.StopName(target.a) << "->"
               << problem.StopName(target.b) << "\n";
    }
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
      RC_LOG() << problem.StopName(stops[i]);
    }
    RC_LOG() << "] duration=" << duration << " sum_weights=" << sum_weights
             << "\n";
    RC_ASSERT(sum_weights <= duration);
  } while (std::next_permutation(
      stops.begin(), stops.end(), [](StopId a, StopId b) { return a.v < b.v; }
  ));
}

// Normalize an EmbiggenerState so that two states which produce the same set of
// 2-hop primitive paths but differ only in how each edge's slack is split
// between weight and per-path delta become byte-for-byte comparable. For each
// edge:
//  1. Drop the edge if no 2-hop primitive path uses it.
//  (ConstrainEmbiggenerState
//     copies edges from the source state but may leave behind edges that have
//     no surviving primitive path; MakeEmbiggenerState never emits such edges.)
//  2. Otherwise let m be the smallest delta among 2-hop primitive paths on
//     that edge. Add m to the edge weight and subtract m from each matching
//     path's delta, so that at least one 2-hop path on every edge has delta=0
//     (matching the invariant established by MakeEmbiggenerState).
void NormalizeEmbiggenerState(EmbiggenerState& state) {
  std::unordered_map<PlainEdge, std::vector<std::size_t>> two_hop_by_edge;
  for (std::size_t i = 0; i < state.primitive_paths.size(); ++i) {
    const LocalEmbiggenState& p = state.primitive_paths[i];
    if (p.path.size() != 2) continue;
    two_hop_by_edge[PlainEdge{p.path.front().s, p.path.back().s}].push_back(i);
  }

  for (auto it = state.edges.begin(); it != state.edges.end();) {
    auto bucket_it = two_hop_by_edge.find(it->first);
    if (bucket_it == two_hop_by_edge.end()) {
      it = state.edges.erase(it);
      continue;
    }
    int min_delta = std::numeric_limits<int>::max();
    for (std::size_t i : bucket_it->second) {
      min_delta = std::min(min_delta, state.primitive_paths[i].delta);
    }
    if (min_delta > 0) {
      it->second.weight += min_delta;
      for (std::size_t i : bucket_it->second) {
        state.primitive_paths[i].delta -= min_delta;
      }
    }
    ++it;
  }
}

struct PathKey {
  // Encoded as alternating (stop_id, time_seconds) pairs.
  std::vector<int> path_v;
  int delta;
  bool operator==(const PathKey&) const = default;
  bool operator<(const PathKey& other) const {
    if (path_v != other.path_v) return path_v < other.path_v;
    return delta < other.delta;
  }
};

std::vector<PathKey> SortedPaths(const EmbiggenerState& state) {
  std::vector<PathKey> result;
  for (const LocalEmbiggenState& p : state.primitive_paths) {
    if (p.path.empty()) continue;
    PathKey k;
    k.path_v.reserve(p.path.size() * 2);
    for (const PointInstant& pi : p.path) {
      k.path_v.push_back(pi.s.v);
      k.path_v.push_back(pi.t.seconds);
    }
    k.delta = p.delta;
    result.push_back(std::move(k));
  }
  std::sort(result.begin(), result.end());
  return result;
}

std::map<std::pair<int, int>, int> EdgesAsSortedMap(
    const EmbiggenerState& state
) {
  std::map<std::pair<int, int>, int> result;
  for (const auto& [edge, data] : state.edges) {
    result[{edge.a.v, edge.b.v}] = data.weight;
  }
  return result;
}

// Generates a (problem, base_state, extra_intermediate_point) triple where
// `p` is a (stop, time) pair that the base state actually arrives at.
struct ConstrainSetup {
  ProblemState problem;
  StepsAdjacencyList completed;
  std::vector<PointBound> base_known_points;
  EmbiggenerState base_state;
  PointInstant p;
};

ConstrainSetup GenConstrainSetup() {
  ConstrainSetup setup;
  setup.problem = *GenProblemState();
  setup.completed =
      MakeAdjacencyList(setup.problem.ComputeCompletedGraph().AllMergedSteps());

  StopId start = setup.problem.boundary.start;
  StopId end = setup.problem.boundary.end;

  setup.base_known_points = {
      PointBound{start, TimeSinceServiceStart{0}, TimeSinceServiceStart{0}},
      PointBound{end, TimeSinceServiceStart{0}, TimeSinceServiceStart{100000}},
  };
  setup.base_state = MakeEmbiggenerState(
      setup.problem, setup.completed, setup.base_known_points, {}
  );

  std::unordered_set<StopId> rep_set =
      setup.problem.required.GroupRepresentatives();
  rep_set.erase(start);
  rep_set.erase(end);
  std::vector<StopId> stops(rep_set.begin(), rep_set.end());
  std::sort(stops.begin(), stops.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });
  RC_PRE(!stops.empty());

  std::unordered_map<StopId, std::vector<TimeSinceServiceStart>> arrival_times;
  for (const LocalEmbiggenState& p : setup.base_state.primitive_paths) {
    if (p.path.size() != 2) continue;
    if (rep_set.contains(p.path[1].s)) {
      arrival_times[p.path[1].s].push_back(p.path[1].t);
    }
  }
  StopId p_stop = *rc::gen::elementOf(stops);
  RC_PRE(arrival_times.contains(p_stop) && !arrival_times[p_stop].empty());
  TimeSinceServiceStart p_time = *rc::gen::elementOf(arrival_times[p_stop]);
  setup.p = PointInstant{p_stop, p_time};
  RC_LOG() << "p = (" << setup.problem.StopName(p_stop) << ", " << p_time
           << ")\n";

  return setup;
}

void CheckConstrainMatchesMake(
    const ConstrainSetup& setup,
    const std::vector<PointBound>& extra_known_points,
    const std::unordered_set<PointInstant>& extra_forbidden_points
) {
  EmbiggenerState from_make = MakeEmbiggenerState(
      setup.problem, setup.completed, extra_known_points, extra_forbidden_points
  );
  EmbiggenerState from_constrain = ConstrainEmbiggenerState(
      setup.problem,
      setup.base_state,
      extra_known_points,
      extra_forbidden_points
  );

  NormalizeEmbiggenerState(from_make);
  NormalizeEmbiggenerState(from_constrain);

  RC_ASSERT(EdgesAsSortedMap(from_make) == EdgesAsSortedMap(from_constrain));
  RC_ASSERT(SortedPaths(from_make) == SortedPaths(from_constrain));
}

// Property: starting from a base EmbiggenerState built with no intermediate
// known points, applying ConstrainEmbiggenerState with one extra known
// intermediate point should yield the same state (up to per-edge weight/delta
// slack) as calling MakeEmbiggenerState directly with that extra known point.
RC_GTEST_PROP(
    EmbiggenerTest, ConstrainEmbiggenerState_KnownPoint_MatchesMake, ()
) {
  ConstrainSetup setup = GenConstrainSetup();
  std::vector<PointBound> extra_known_points{
      PointBound{
          setup.problem.boundary.start,
          TimeSinceServiceStart{0},
          TimeSinceServiceStart{0}
      },
      PointBound{setup.p.s, setup.p.t, setup.p.t},
      PointBound{
          setup.problem.boundary.end, setup.p.t, TimeSinceServiceStart{100000}
      },
  };
  CheckConstrainMatchesMake(setup, extra_known_points, {});
}

// Property: same as above but with a forbidden point instead of a known one.
RC_GTEST_PROP(
    EmbiggenerTest, ConstrainEmbiggenerState_ForbiddenPoint_MatchesMake, ()
) {
  ConstrainSetup setup = GenConstrainSetup();
  CheckConstrainMatchesMake(setup, setup.base_known_points, {setup.p});
}

}  // namespace
}  // namespace vats5
