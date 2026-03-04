#include "solver/test_util/problem_state_gen.h"

#include <rapidcheck.h>

#include <algorithm>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rapidcheck/gen/Create.h"
#include "rapidcheck/gen/Select.h"
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"

namespace vats5 {

rc::Gen<std::unordered_map<StopId, StopId>> GenAlternateStop(int num_stops) {
  return rc::gen::map(
      rc::gen::mapcat(
          rc::gen::inRange(1, num_stops + 1),
          [num_stops](int num_groups) {
            return rc::gen::container<std::vector<int>>(
                num_stops, rc::gen::inRange(0, num_groups)
            );
          }
      ),
      [num_stops](std::vector<int> group_assignments)
          -> std::unordered_map<StopId, StopId> {
        std::unordered_map<int, std::vector<int>> groups;
        for (int i = 0; i < num_stops; ++i) {
          groups[group_assignments[i]].push_back(i);
        }
        std::unordered_map<StopId, StopId> alternate_stop;
        for (const auto& [group_id, members] : groups) {
          if (members.size() <= 1) continue;
          StopId representative{members[0]};
          for (size_t i = 1; i < members.size(); ++i) {
            alternate_stop[StopId{members[i]}] = representative;
          }
        }
        return alternate_stop;
      }
  );
}

rc::Gen<ProblemState> GenProblemState(
    std::optional<rc::Gen<CycleIsFlex>> cycle_is_flex_gen,
    std::optional<rc::Gen<StepPartitionId>> step_partition_gen
) {
  rc::Gen<CycleIsFlex> cycle_is_flex_gen_defaulted = cycle_is_flex_gen.value_or(
      rc::gen::element(CycleIsFlex::kNo, CycleIsFlex::kYes)
  );
  rc::Gen<StepPartitionId> step_partition_gen_defaulted =
      step_partition_gen.value_or(rc::gen::just(StepPartitionId::NONE));

  return rc::gen::mapcat(
      std::move(cycle_is_flex_gen_defaulted),
      [step_partition_gen_defaulted](CycleIsFlex cycle_is_flex)
          -> rc::Gen<ProblemState> {
        rc::Gen<int> num_actual_stops_gen = rc::gen::inRange(2, 5);

        return rc::gen::mapcat(
            num_actual_stops_gen,
            [cycle_is_flex, step_partition_gen_defaulted](int num_actual_stops)
                -> rc::Gen<ProblemState> {
              auto step_gen = rc::gen::apply(
                  [num_actual_stops](
                      int origin,
                      int dest_offset,
                      int origin_time,
                      int duration,
                      StepPartitionId step_partition
                  ) -> Step {
                    int destination =
                        (origin + 1 + dest_offset) % num_actual_stops;
                    return Step::PrimitiveScheduled(
                        StopId{origin},
                        StopId{destination},
                        TimeSinceServiceStart{origin_time},
                        TimeSinceServiceStart{origin_time + duration},
                        TripId::NOOP,
                        step_partition
                    );
                  },
                  rc::gen::inRange(0, num_actual_stops),
                  rc::gen::inRange(0, num_actual_stops - 1),
                  rc::gen::inRange(0, 3600),
                  rc::gen::inRange(0, 600),
                  step_partition_gen_defaulted
              );

              rc::Gen<std::vector<Step>> steps_gen = rc::gen::mapcat(
                  rc::gen::inRange(0, 100),
                  [step_gen](int num_steps) -> rc::Gen<std::vector<Step>> {
                    return rc::gen::container<std::vector<Step>>(
                        num_steps, step_gen
                    );
                  }
              );

              rc::Gen<std::vector<StepPartitionId>> cycle_steps_partition_gen =
                  rc::gen::container<std::vector<StepPartitionId>>(
                      num_actual_stops, step_partition_gen_defaulted
                  );

              return rc::gen::map(
                  rc::gen::tuple(
                      std::move(steps_gen),
                      std::move(cycle_steps_partition_gen),
                      GenAlternateStop(num_actual_stops)
                  ),
                  [num_actual_stops,
                   cycle_is_flex,
                   step_partition_gen_defaulted](auto arg) -> ProblemState {
                    auto [steps, cycle_step_partitions, alternate_stop] = arg;

                    // Set up all the stops.
                    std::unordered_set<StopId> stops;
                    std::unordered_map<StopId, ProblemStateStopInfo> stop_infos;
                    for (int i = 0; i < num_actual_stops; ++i) {
                      StopId stop{i};
                      stops.insert(stop);
                      stop_infos[stop] = ProblemStateStopInfo{
                          GtfsStopId{std::string(1, 'a' + i)},
                          std::string(1, 'a' + i)
                      };
                    }

                    // Fix up the trip ids for the random steps.
                    TripId next_trip_id{0};
                    for (Step& step : steps) {
                      step.origin.trip = next_trip_id;
                      step.destination.trip = next_trip_id;
                      next_trip_id.v += 1;
                    }

                    // Set up a cycle of steps so that there is always a tour.
                    bool flex = (cycle_is_flex == CycleIsFlex::kYes);
                    for (int i = 0; i < num_actual_stops; ++i) {
                      TripId trip_id = next_trip_id;
                      next_trip_id.v += 1;
                      steps.push_back(
                          flex ? Step::PrimitiveFlex(
                                     StopId{i},
                                     StopId{(i + 1) % num_actual_stops},
                                     1200,
                                     trip_id,
                                     cycle_step_partitions[i]
                                 )
                               : Step::PrimitiveScheduled(
                                     StopId{i},
                                     StopId{(i + 1) % num_actual_stops},
                                     TimeSinceServiceStart{1200 * i},
                                     TimeSinceServiceStart{1200 * (i + 1)},
                                     trip_id,
                                     cycle_step_partitions[i]
                                 )
                      );
                    }

                    // Add the boundary.
                    ProblemBoundary boundary{
                        .start = StopId{num_actual_stops},
                        .end = StopId{num_actual_stops + 1},
                    };
                    AddBoundary(steps, stops, stop_infos, boundary);

                    return MakeProblemState(
                        MakeAdjacencyList(steps),
                        boundary,
                        RequiredStops::FromStopsAndGroups(
                            stops, alternate_stop
                        ),
                        stop_infos,
                        {},
                        {}
                    );
                  }
              );
            }
        );
      }
  );
}

rc::Gen<ProblemState> GenFlexProblemState() {
  rc::Gen<int> num_actual_stops_gen = rc::gen::inRange(2, 5);

  return rc::gen::mapcat(
      num_actual_stops_gen, [](int num_actual_stops) -> rc::Gen<ProblemState> {
        int num_edges = num_actual_stops * (num_actual_stops - 1);

        rc::Gen<std::vector<int>> durations_gen =
            rc::gen::container<std::vector<int>>(
                num_edges, rc::gen::inRange(1, 600)
            );

        return rc::gen::map(
            rc::gen::tuple(
                std::move(durations_gen), GenAlternateStop(num_actual_stops)
            ),
            [num_actual_stops](auto arg) -> ProblemState {
              auto [durations, alternate_stop] = arg;

              std::unordered_set<StopId> stops;
              std::unordered_map<StopId, ProblemStateStopInfo> stop_infos;
              for (int i = 0; i < num_actual_stops; ++i) {
                StopId stop{i};
                stops.insert(stop);
                stop_infos[stop] = ProblemStateStopInfo{
                    GtfsStopId{std::string(1, 'a' + i)}, std::string(1, 'a' + i)
                };
              }

              // Complete bidirectional graph of flex steps, each with a
              // distinct partition.
              std::vector<Step> steps;
              TripId next_trip_id{0};
              int edge_idx = 0;
              for (int i = 0; i < num_actual_stops; ++i) {
                for (int j = 0; j < num_actual_stops; ++j) {
                  if (i == j) continue;
                  TripId trip_id = next_trip_id;
                  next_trip_id.v += 1;
                  steps.push_back(
                      Step::PrimitiveFlex(
                          StopId{i},
                          StopId{j},
                          durations[edge_idx],
                          trip_id,
                          StepPartitionId{edge_idx}
                      )
                  );
                  edge_idx += 1;
                }
              }

              ProblemBoundary boundary{
                  .start = StopId{num_actual_stops},
                  .end = StopId{num_actual_stops + 1},
              };
              AddBoundary(steps, stops, stop_infos, boundary);

              return MakeProblemState(
                  MakeAdjacencyList(steps),
                  boundary,
                  RequiredStops::FromStopsAndGroups(stops, alternate_stop),
                  stop_infos,
                  {},
                  {}
              );
            }
        );
      }
  );
}

void showValue(const NamedBranchEdge& e, std::ostream& os) {
  os << "BranchEdge{" << e.a_name << " -> " << e.b_name << "}";
}

rc::Gen<NamedBranchEdge> GenBranchEdge(const ProblemState& state) {
  std::unordered_set<StopId> required_flat = state.required.AllFlat();
  std::vector<StopId> stops(required_flat.begin(), required_flat.end());
  ProblemBoundary boundary = state.boundary;
  std::unordered_map<StopId, ProblemStateStopInfo> stop_infos =
      state.stop_infos;
  int n = static_cast<int>(stops.size());
  return rc::gen::suchThat(
      rc::gen::apply(
          [stops, stop_infos, n](int ai, int b_offset) -> NamedBranchEdge {
            int bi = (ai + 1 + b_offset) % n;
            return NamedBranchEdge{
                BranchEdge{stops[ai], stops[bi]},
                stop_infos.at(stops[ai]).stop_name,
                stop_infos.at(stops[bi]).stop_name,
            };
          },
          rc::gen::inRange(0, n),
          rc::gen::inRange(0, n - 1)
      ),
      [boundary](const NamedBranchEdge& e) {
        return e.edge.a != boundary.end && e.edge.b != boundary.start &&
               !(e.edge.a == boundary.start && e.edge.b == boundary.end);
      }
  );
}

}  // namespace vats5
