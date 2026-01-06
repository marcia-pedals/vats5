#pragma once

#include <span>

#include "solver/data.h"

namespace vats5 {

// A fixed-schedule step stored in an adjacency list. Omits origin_stop (known
// from CSR position), destination_stop (stored once in StepGroup), and is_flex
// (inferred from flex_step vs steps in StepGroup).
struct AdjacencyListStep {
  TimeSinceServiceStart origin_time;
  TimeSinceServiceStart destination_time;

  TripId origin_trip;
  TripId destination_trip;

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

// A flex step stored in an adjacency list. Only stores duration since flex
// trips can depart at any time. Omits origin_stop and destination_stop (known
// from StepGroup).
struct AdjacencyListFlexStep {
  int duration_seconds;

  TripId origin_trip;
  TripId destination_trip;

  bool operator==(const AdjacencyListFlexStep& other) const {
    return duration_seconds == other.duration_seconds &&
           origin_trip == other.origin_trip &&
           destination_trip == other.destination_trip;
  }
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    AdjacencyListFlexStep, duration_seconds, origin_trip, destination_trip
)

// A group of steps from one origin to one destination, sorted by origin time.
// The steps themselves are stored in StepsAdjacencyList::all_steps, and this
// struct stores an index into that array. The end of the range is determined
// by the next StepGroup's steps_begin (or all_steps.size() for the last group).
struct StepGroup {
  // The destination stop for all steps in this group.
  StopId destination_stop;

  // Flex step for this origin-destination pair.
  // If origin_trip == TripId::NOOP, there is no flex step.
  // Otherwise, this is a flex trip that can be taken at any time.
  AdjacencyListFlexStep flex_step;

  // Index into StepsAdjacencyList::all_steps and
  // StepsAdjacencyList::all_departure_times_div10 where this group's
  // fixed-schedule steps begin. The end is the next StepGroup's steps_begin
  // (or all_steps.size() for the last group).
  uint32_t steps_begin = 0;

  // Returns true if this group has a flex step.
  bool has_flex_step() const { return flex_step.origin_trip != TripId::NOOP; }
};

// Forward declaration for StepGroup JSON serialization
struct StepsAdjacencyList;

struct StepsAdjacencyList {
  // CSR (Compressed Sparse Row) format for step groups.
  // step_groups_for_stop(i) returns a span of step_groups for stop i.
  //
  // offsets[i] is the index into step_groups where stop i's groups begin.
  // offsets has size num_stops + 1, with offsets[num_stops] ==
  // step_groups.size().
  std::vector<uint32_t> offsets;

  // Flat array of all step groups, grouped by origin stop.
  std::vector<StepGroup> step_groups;

  // Flat array of all fixed-schedule steps. StepGroup::steps_begin indexes
  // into this array. Steps are stored contiguously in the same order as
  // step_groups, so the end of one group is the start of the next.
  std::vector<AdjacencyListStep> all_steps;

  // Parallel array to all_steps: departure_times_div10[i] =
  // all_steps[i].origin_time.seconds / 10. Divided by 10 to fit in int16_t
  // (max 32767 * 10 = 327670 seconds ≈ 91 hours).
  std::vector<int16_t> all_departure_times_div10;

  // Number of stops (offsets.size() - 1, or 0 if empty).
  size_t num_stops() const { return offsets.empty() ? 0 : offsets.size() - 1; }

  // Get step groups for a given stop.
  std::span<const StepGroup> step_groups_for_stop(size_t stop_id) const {
    if (stop_id >= num_stops()) {
      return {};
    }
    return std::span<const StepGroup>(
        step_groups.data() + offsets[stop_id],
        offsets[stop_id + 1] - offsets[stop_id]
    );
  }

  // Get the end index for a step group's steps (steps_begin of next group,
  // or all_steps.size() for the last group).
  size_t steps_end_for_group(const StepGroup& sg) const {
    size_t group_idx = static_cast<size_t>(&sg - step_groups.data());
    return (group_idx + 1 < step_groups.size())
               ? step_groups[group_idx + 1].steps_begin
               : all_steps.size();
  }

  // Get steps for a step group.
  std::span<const AdjacencyListStep> steps_for_group(const StepGroup& sg
  ) const {
    size_t steps_end = steps_end_for_group(sg);
    return std::span<const AdjacencyListStep>(
        all_steps.data() + sg.steps_begin, steps_end - sg.steps_begin
    );
  }

  // Get departure times for a step group.
  std::span<const int16_t> departure_times_for_group(const StepGroup& sg
  ) const {
    size_t steps_end = steps_end_for_group(sg);
    return std::span<const int16_t>(
        all_departure_times_div10.data() + sg.steps_begin,
        steps_end - sg.steps_begin
    );
  }
};

// Custom JSON serialization for StepGroup
inline void to_json(nlohmann::json& j, const StepGroup& sg) {
  j = nlohmann::json{
      {"destination_stop", sg.destination_stop},
      {"flex_step", sg.flex_step},
      {"steps_begin", sg.steps_begin}
  };
}

inline void from_json(const nlohmann::json& j, StepGroup& sg) {
  sg.destination_stop = j.at("destination_stop").get<StopId>();
  sg.flex_step = j.at("flex_step").get<AdjacencyListFlexStep>();
  sg.steps_begin = j.at("steps_begin").get<uint32_t>();
}

// Custom JSON serialization for StepsAdjacencyList
inline void to_json(nlohmann::json& j, const StepsAdjacencyList& adj) {
  j = nlohmann::json{
      {"offsets", adj.offsets},
      {"step_groups", adj.step_groups},
      {"all_steps", adj.all_steps},
      {"all_departure_times_div10", adj.all_departure_times_div10}
  };
}

inline void from_json(const nlohmann::json& j, StepsAdjacencyList& adj) {
  adj.offsets = j.at("offsets").get<std::vector<uint32_t>>();
  adj.step_groups = j.at("step_groups").get<std::vector<StepGroup>>();
  adj.all_steps = j.at("all_steps").get<std::vector<AdjacencyListStep>>();
  adj.all_departure_times_div10 =
      j.at("all_departure_times_div10").get<std::vector<int16_t>>();
}

struct PathsAdjacencyList {
  // Mapping from stop to paths originating at that stop, grouped by destination
  // stop. Each group of paths is sorted by origin time and minimal.
  std::unordered_map<StopId, std::vector<std::vector<Path>>> adjacent;
};

// Custom JSON serialization for PathsAdjacencyList
// Convert the StopId-keyed map to int-keyed for JSON
inline void to_json(nlohmann::json& j, const PathsAdjacencyList& adj) {
  std::vector<std::pair<int, std::vector<std::vector<Path>>>> pairs;
  for (const auto& [k, v] : adj.adjacent) {
    pairs.emplace_back(k.v, v);
  }
  j = nlohmann::json{{"adjacent", pairs}};
}

inline void from_json(const nlohmann::json& j, PathsAdjacencyList& adj) {
  auto pairs =
      j.at("adjacent")
          .get<std::vector<std::pair<int, std::vector<std::vector<Path>>>>>();
  for (const auto& [k, v] : pairs) {
    adj.adjacent[StopId{k}] = v;
  }
}

// Group steps into an adjacency list.
StepsAdjacencyList MakeAdjacencyList(const std::vector<Step>& steps);

// The data we keep at each stop along a shortest path search.
//
// Stores the step used to reach this stop. The full path can be reconstructed
// by backtracking through the result map.
struct PathState {
  // The step taken to reach this stop.
  Step step;
};

// Backtrack through the search results to reconstruct the full path.
// Returns the steps in order from origin to destination.
std::vector<Step> BacktrackPath(
    const std::vector<PathState>& search_result, StopId dest
);

// Compute the merged step for a path, with proper origin time calculation.
// The origin time adjustment handles flex paths that transition to fixed trips.
Step ComputeMergedStep(const std::vector<Step>& path);

// Find earliest times you can get to a set of stops from `origin` when starting
// at `time`.
//
// The return includes at least values for `destinations` and for all
// intermediate stops on the optimal routes to them. It may include values for
// other stops visited during the search.
//
// The returned steps may depart later than `time` from `origin`, but you should
// not rely on this to find the latest possible departure time (see the
// SuboptimalDepartureTimeExposure test).
//
// Returns a vector indexed by StopId.v. Unvisited stops have
// destination_time.seconds == numeric_limits<int>::max().
std::vector<PathState> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list,
    TimeSinceServiceStart time,
    StopId origin,
    const std::unordered_set<StopId>& destinations
);

// Return a minimal set of paths from `origin` to destinations, with origin
// times in [00:00, 36:00). Return value is a map from destination to the paths
// from `origin` to that destination.
//
// Minimal means that these properties hold:
// (a) Any path in `adjacency_list` from `origin` to a destination can be
// matched or beat by a path in the return. (b) If you remove any path from the
// return value, (a) no longer holds.
std::unordered_map<StopId, std::vector<Path>> FindMinimalPathSet(
    const StepsAdjacencyList& adjacency_list,
    StopId origin,
    const std::unordered_set<StopId>& destinations,
    TimeSinceServiceStart origin_time_lb = TimeSinceServiceStart{0},
    TimeSinceServiceStart origin_time_ub = TimeSinceServiceStart{36 * 3600}
);

// Return an adjacency list ("reduced list") with these properties:
// (a) Any path in `adjacency_list` between two `system_stops` [1] can be
// matched or beat by a path in the reduced list. (b) If you remove any step
// from the reduced list, (a) no longer holds.
//
// [1] Usually-unimportant qualification: All departures from `system_stops` in
// the path happen at <36:00.
PathsAdjacencyList ReduceToMinimalSystemPaths(
    const StepsAdjacencyList& adjacency_list,
    const std::unordered_set<StopId>& system_stops
);

PathsAdjacencyList SplitPathsAt(
    const PathsAdjacencyList& paths,
    const std::unordered_set<StopId> intermediate_stops
);

StepsAdjacencyList AdjacentPathsToStepsList(const PathsAdjacencyList& paths);

// Return a modified `path` where any intermediate stops within
// `threshold_meters` of a stop in `stops` are replaced by that stop, and then
// steps that are thusly changed to be from a stop to itself are dropped.
//
// Note that this doesn't change any trip ids or times in the modified steps, so
// the steps become steps that may not actually be possible, so maybe this
// should be used only to clean up visualizations and not to do actual
// computations.
std::vector<Step> SnapToStops(
    const DataGtfsMapping& mapping,
    const std::unordered_set<StopId>& stops,
    double threshold_meters,
    const std::vector<Step>& path
);

// Snap all paths in the adjacency list to the stops that are keys of the
// adjacency list.
PathsAdjacencyList AdjacencyListSnapToStops(
    const DataGtfsMapping& mapping,
    double threshold_meters,
    const PathsAdjacencyList& paths
);

}  // namespace vats5
