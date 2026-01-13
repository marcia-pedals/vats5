#pragma once

#include <optional>
#include <span>
#include <vector>

#include "solver/data.h"

namespace vats5 {

// A step in the adjacency list with only the necessary data.
// origin_stop and destination_stop are stored in the parent structures,
// and is_flex is inferred from context (flex_step vs steps array).
struct AdjacencyListStep {
  TimeSinceServiceStart origin_time;
  TimeSinceServiceStart destination_time;
  TripId origin_trip;
  TripId destination_trip;

  // Convert to a full Step given the context.
  Step ToStep(StopId origin_stop, StopId destination_stop, bool is_flex) const {
    return Step{
        origin_stop,
        destination_stop,
        origin_time,
        destination_time,
        origin_trip,
        destination_trip,
        is_flex
    };
  }

  // Create from a full Step.
  static AdjacencyListStep FromStep(const Step& step) {
    return AdjacencyListStep{
        step.origin_time,
        step.destination_time,
        step.origin_trip,
        step.destination_trip
    };
  }

  int FlexDurationSeconds() const {
    return destination_time.seconds - origin_time.seconds;
  }

  bool operator==(const AdjacencyListStep& other) const {
    return origin_time == other.origin_time &&
           destination_time == other.destination_time &&
           origin_trip == other.origin_trip &&
           destination_trip == other.destination_trip;
  }
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    AdjacencyListStep,
    origin_time,
    destination_time,
    origin_trip,
    destination_trip
)

// A group of steps from one origin to one destination, sorted by origin time.
// The fixed-schedule steps and their departure times are stored in the parent
// StepsAdjacencyList; this struct holds indices into those arrays.
struct StepGroup {
  // The destination stop for all steps in this group.
  // The origin stop is determined by the position in the CSR structure.
  StopId destination_stop;

  // Optional flex step for this origin-destination pair.
  // If present, this is a flex trip that can be taken at any time.
  // is_flex is implicitly true for this step.
  std::optional<AdjacencyListStep> flex_step;

  // Index range [steps_start, steps_end) into StepsAdjacencyList.steps
  // for fixed-schedule steps, sorted by origin time.
  // is_flex is implicitly false for these steps.
  int steps_start = 0;
  int steps_end = 0;

  bool operator==(const StepGroup& other) const {
    return destination_stop == other.destination_stop &&
           flex_step == other.flex_step && steps_start == other.steps_start &&
           steps_end == other.steps_end;
  }
};

inline void to_json(nlohmann::json& j, const StepGroup& sg) {
  j = nlohmann::json{
      {"destination_stop", sg.destination_stop},
      {"flex_step", sg.flex_step},
      {"steps_start", sg.steps_start},
      {"steps_end", sg.steps_end}
  };
}

inline void from_json(const nlohmann::json& j, StepGroup& sg) {
  sg.destination_stop = j.at("destination_stop").get<StopId>();
  sg.flex_step = j.at("flex_step").get<std::optional<AdjacencyListStep>>();
  sg.steps_start = j.at("steps_start").get<int>();
  sg.steps_end = j.at("steps_end").get<int>();
}

struct StepsAdjacencyList {
  // CSR (Compressed Sparse Row) representation of step groups per stop.
  // group_offsets[stop_id.v] is the start index into `groups` for that stop.
  // group_offsets has size NumStops().
  std::vector<int> group_offsets;

  // Flat vector of all StepGroups. Groups for stop s are at indices
  // [group_offsets[s.v], group_offsets[s.v + 1]) for s.v < NumStops() - 1,
  // or [group_offsets[s.v], groups.size()) for s.v == NumStops() - 1.
  std::vector<StepGroup> groups;

  // Flat vector of all fixed-schedule steps across all groups.
  // Each StepGroup references a range [steps_start, steps_end) into this.
  // The origin_stop, destination_stop, and is_flex are stored in the StepGroup.
  std::vector<AdjacencyListStep> steps;

  // Parallel array to steps: departure_times_div10[i] =
  // steps[i].origin_time.seconds / 10. Divided by 10 to fit in int16_t
  // (max 32767 * 10 = 327670 seconds ≈ 91 hours).
  std::vector<int16_t> departure_times_div10;

  // Strict upper bound on all StopId values in this adjacency list.
  // All stop IDs s satisfy s.v < NumStops().
  int NumStops() const { return static_cast<int>(group_offsets.size()); }

  // Get the step groups originating at the given stop.
  std::span<const StepGroup> GetGroups(StopId stop) const {
    if (stop.v < 0 || stop.v >= NumStops()) {
      return {};
    }
    int start = group_offsets[stop.v];
    int end = (stop.v + 1 < NumStops()) ? group_offsets[stop.v + 1]
                                        : static_cast<int>(groups.size());
    return std::span<const StepGroup>(groups.data() + start, end - start);
  }

  // Get the fixed-schedule steps for a StepGroup.
  std::span<const AdjacencyListStep> GetSteps(const StepGroup& group) const {
    return std::span<const AdjacencyListStep>(
        steps.data() + group.steps_start, group.steps_end - group.steps_start
    );
  }

  // Get the departure times (div 10) for a StepGroup.
  std::span<const int16_t> GetDepartureTimes(const StepGroup& group) const {
    return std::span<const int16_t>(
        departure_times_div10.data() + group.steps_start,
        group.steps_end - group.steps_start
    );
  }

  bool operator==(const StepsAdjacencyList& other) const {
    return group_offsets == other.group_offsets && groups == other.groups &&
           steps == other.steps;
    // Note: departure_times_div10 is derived from steps, so we don't compare it
  }
};

// Custom JSON serialization for StepsAdjacencyList
inline void to_json(nlohmann::json& j, const StepsAdjacencyList& adj) {
  j = nlohmann::json{
      {"group_offsets", adj.group_offsets},
      {"groups", adj.groups},
      {"steps", adj.steps},
      {"departure_times_div10", adj.departure_times_div10},
  };
}

inline void from_json(const nlohmann::json& j, StepsAdjacencyList& adj) {
  adj.group_offsets = j.at("group_offsets").get<std::vector<int>>();
  adj.groups = j.at("groups").get<std::vector<StepGroup>>();
  adj.steps = j.at("steps").get<std::vector<AdjacencyListStep>>();
  adj.departure_times_div10 =
      j.at("departure_times_div10").get<std::vector<int16_t>>();
}

// Group steps into an adjacency list.
StepsAdjacencyList MakeAdjacencyList(const std::vector<Step>& steps);

struct StepPathsAdjacencyList {
  // Mapping from stop to paths originating at that stop, grouped by destination
  // stop. Each group of paths is sorted by origin time and minimal.
  std::unordered_map<StopId, std::vector<std::vector<Path>>> adjacent;
};

// Custom JSON serialization for StepPathsAdjacencyList
// Convert the StopId-keyed map to int-keyed for JSON
inline void to_json(nlohmann::json& j, const StepPathsAdjacencyList& adj) {
  std::vector<std::pair<int, std::vector<std::vector<Path>>>> pairs;
  for (const auto& [k, v] : adj.adjacent) {
    pairs.emplace_back(k.v, v);
  }
  j = nlohmann::json{{"adjacent", pairs}};
}

inline void from_json(const nlohmann::json& j, StepPathsAdjacencyList& adj) {
  auto pairs =
      j.at("adjacent")
          .get<std::vector<std::pair<int, std::vector<std::vector<Path>>>>>();
  for (const auto& [k, v] : pairs) {
    adj.adjacent[StopId{k}] = v;
  }
}

// Convert a StepPathsAdjacencyList to a StepsAdjacencyList by extracting
// merged steps from all paths.
StepsAdjacencyList AdjacentPathsToStepsList(const StepPathsAdjacencyList& paths);

}  // namespace vats5
