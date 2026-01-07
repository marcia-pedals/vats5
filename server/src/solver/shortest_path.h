#pragma once

#include <span>

#include "solver/data.h"

namespace vats5 {

// A group of steps from one origin to one destination, sorted by origin time.
// The fixed-schedule steps and their departure times are stored in the parent
// StepsAdjacencyList; this struct holds indices into those arrays.
struct StepGroup {
  // Optional flex step for this origin-destination pair.
  // If present, this is a flex trip that can be taken at any time.
  std::optional<Step> flex_step;

  // Index range [steps_start, steps_end) into StepsAdjacencyList.steps
  // for fixed-schedule steps, sorted by origin time.
  int steps_start = 0;
  int steps_end = 0;
};

inline void to_json(nlohmann::json& j, const StepGroup& sg) {
  j = nlohmann::json{
      {"flex_step", sg.flex_step},
      {"steps_start", sg.steps_start},
      {"steps_end", sg.steps_end}
  };
}

inline void from_json(const nlohmann::json& j, StepGroup& sg) {
  sg.flex_step = j.at("flex_step").get<std::optional<Step>>();
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
  std::vector<Step> steps;

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
  std::span<const Step> GetSteps(const StepGroup& group) const {
    return std::span<const Step>(
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
  adj.steps = j.at("steps").get<std::vector<Step>>();
  adj.departure_times_div10 =
      j.at("departure_times_div10").get<std::vector<int16_t>>();
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
//
// If `smallest_next_departure_gap_from_flex` is specified, it'll be set to the
// smallest gap between any visited flex path from the origin and the next
// departure that departs strictly after the visited flex path visits the stop.
// This is useful if you are scanning forwards to find all best paths from A->B,
// and you get a flex result, then you know that the result continues to be flex
// up at least until `time + smallest_next_departure_gap_from_flex`. It gets set
// to `std::numeric_limits<int>::max()` if there are no departures after any
// visited flex paths.
std::vector<PathState> FindShortestPathsAtTime(
    const StepsAdjacencyList& adjacency_list,
    TimeSinceServiceStart time,
    StopId origin,
    const std::unordered_set<StopId>& destinations,
    int* smallest_next_departure_gap_from_flex = nullptr
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
    const std::unordered_set<StopId>& destinations
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
