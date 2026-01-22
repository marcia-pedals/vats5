#include "solver/test_util/problem_state_gen.h"

#include <rapidcheck.h>

#include <algorithm>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"

namespace vats5 {

void showValue(const ProblemState& state, std::ostream& os) {
  std::vector<Step> steps = state.minimal.AllSteps();
  StopId start = state.boundary.start;
  StopId end = state.boundary.end;

  auto sort_key = [&](const Step& s) {
    bool adj_start = (s.origin_stop == start || s.destination_stop == start);
    bool adj_end = (s.origin_stop == end || s.destination_stop == end);
    int group;
    if (adj_start) group = 2;
    else if (adj_end) group = 3;
    else if (!s.is_flex) group = 0;
    else group = 1;
    return std::tuple(group, s.origin_stop.v, s.destination_stop.v);
  };
  std::sort(steps.begin(), steps.end(), [&](const Step& a, const Step& b) {
    return sort_key(a) < sort_key(b);
  });

  std::vector<StopId> all_stop_ids;
  for (const auto& [id, name] : state.stop_names) {
    all_stop_ids.push_back(id);
  }
  std::sort(all_stop_ids.begin(), all_stop_ids.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });

  std::vector<StopId> required_stop_ids(state.required_stops.begin(), state.required_stops.end());
  std::sort(required_stop_ids.begin(), required_stop_ids.end(), [](StopId a, StopId b) {
    return a.v < b.v;
  });

  os << "ProblemState{\n";
  os << "  stop_names=[";
  for (size_t i = 0; i < all_stop_ids.size(); ++i) {
    if (i > 0) os << ", ";
    os << state.stop_names.at(all_stop_ids[i]);
  }
  os << "]\n";
  os << "  required_stops=[";
  for (size_t i = 0; i < required_stop_ids.size(); ++i) {
    if (i > 0) os << ", ";
    os << state.stop_names.at(required_stop_ids[i]);
  }
  os << "]\n";
  os << "  steps=[\n";
  for (const auto& step : steps) {
    os << "    " << state.stop_names.at(step.origin_stop)
       << " -> " << state.stop_names.at(step.destination_stop);
    if (step.is_flex) {
      os << " (flex " << TimeSinceServiceStart{step.FlexDurationSeconds()}.ToString() << ")";
    } else {
      os << " [" << step.origin_time.ToString() << " -> " << step.destination_time.ToString() << "]";
    }
    os << "\n";
  }
  os << "  ]\n";
  os << "}";
}

rc::Gen<ProblemState> GenProblemState(rc::Gen<CycleIsFlex> cycle_is_flex_gen) {
  return rc::gen::mapcat(std::move(cycle_is_flex_gen), [](CycleIsFlex cycle_is_flex) -> rc::Gen<ProblemState> {
    rc::Gen<int> num_actual_stops_gen = rc::gen::inRange(2, 5);

    return rc::gen::mapcat(num_actual_stops_gen, [cycle_is_flex](int num_actual_stops) -> rc::Gen<ProblemState> {
    auto step_gen = rc::gen::apply([num_actual_stops](int origin, int dest_offset, int origin_time, int duration) -> Step {
      int destination = (origin + 1 + dest_offset) % num_actual_stops;
      return Step{
        .origin_stop=StopId{origin},
        .destination_stop=StopId{destination},
        .origin_time=TimeSinceServiceStart{origin_time},
        .destination_time=TimeSinceServiceStart{origin_time + duration},
        .origin_trip=TripId::NOOP,
        .destination_trip=TripId::NOOP,
        .is_flex=false,
      };
    }, rc::gen::inRange(0, num_actual_stops), rc::gen::inRange(0, num_actual_stops - 1), rc::gen::inRange(0, 3600), rc::gen::inRange(0, 600));

    rc::Gen<std::vector<Step>> steps_gen = rc::gen::mapcat(rc::gen::inRange(0, 100), [step_gen](int num_steps) -> rc::Gen<std::vector<Step>> {
      return rc::gen::container<std::vector<Step>>(num_steps, step_gen);
    });

    return rc::gen::map(std::move(steps_gen), [num_actual_stops, cycle_is_flex](std::vector<Step> steps) -> ProblemState {
      // Set up all the stops.
      std::unordered_set<StopId> stops;
      std::unordered_map<StopId, std::string> stop_names;
      for (int i = 0; i < num_actual_stops; ++i) {
        StopId stop{i};
        stops.insert(stop);
        stop_names[stop] = std::string(1, 'a' + i);
      }

      // Fix up the trip ids for the random steps.
      TripId next_trip_id{0};
      for (Step& step : steps) {
        step.origin_trip = next_trip_id;
        step.destination_trip = next_trip_id;
        next_trip_id.v += 1;
      }

      // Set up a cycle of steps so that there is always a tour.
      bool flex = (cycle_is_flex == CycleIsFlex::kYes);
      for (int i = 0; i < num_actual_stops; ++i) {
        TripId trip_id = next_trip_id;
        next_trip_id.v += 1;
        steps.push_back(Step{
          .origin_stop=StopId{i},
          .destination_stop=StopId{(i + 1) % num_actual_stops},
          .origin_time=TimeSinceServiceStart{flex ? 0 : 1200 * i},
          .destination_time=TimeSinceServiceStart{flex ? 1200 : 1200 * (i + 1)},
          .origin_trip=trip_id,
          .destination_trip=trip_id,
          .is_flex=flex,
        });
      }

      // Add the boundary.
      ProblemBoundary boundary{
        .start=StopId{num_actual_stops},
        .end=StopId{num_actual_stops + 1},
      };
      AddBoundary(steps, stops, stop_names, boundary);

      return MakeProblemState(MakeAdjacencyList(steps), boundary, stops, stop_names);
    });
    });
  });
}

}  // namespace vats5
