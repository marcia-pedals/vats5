#include "solver/data.h"

#include <algorithm>
#include <unordered_set>

namespace vats5 {

StepsFromGtfs GetStepsFromGtfs(GtfsDay gtfs) {
  StepsFromGtfs result;
  result.mapping = DataGtfsMapping{};
  result.steps = {};

  // Create mappings for stops and trips, starting with ID 1
  int next_stop_id = 1;
  int next_trip_id = 1;

  // Collect all unique stops from stop_times
  std::unordered_set<GtfsStopId> unique_stops;
  for (const auto& stop_time : gtfs.stop_times) {
    unique_stops.insert(stop_time.stop_id);
  }

  // Create bidirectional mappings for stops
  for (const auto& gtfs_stop_id : unique_stops) {
    StopId stop_id{next_stop_id++};
    result.mapping.gtfs_stop_id_to_stop_id[gtfs_stop_id] = stop_id;
    result.mapping.stop_id_to_gtfs_stop_id[stop_id] = gtfs_stop_id;
  }

  // Populate stop name to stop IDs mapping
  for (const auto& gtfs_stop : gtfs.stops) {
    auto it = result.mapping.gtfs_stop_id_to_stop_id.find(gtfs_stop.stop_id);
    if (it != result.mapping.gtfs_stop_id_to_stop_id.end()) {
      result.mapping.stop_name_to_stop_ids[gtfs_stop.stop_name].push_back(
          it->second);
    }
  }

  // Collect all unique trips from stop_times
  std::unordered_set<GtfsTripId> unique_trips;
  for (const auto& stop_time : gtfs.stop_times) {
    unique_trips.insert(stop_time.trip_id);
  }

  // Create bidirectional mappings for trips
  for (const auto& gtfs_trip_id : unique_trips) {
    TripId trip_id{next_trip_id++};
    result.mapping.gtfs_trip_id_to_trip_id[gtfs_trip_id] = trip_id;
    result.mapping.trip_id_to_gtfs_trip_id[trip_id] = gtfs_trip_id;
  }

  // Sort stop_times by trip_id and stop_sequence to ensure correct order
  std::vector<GtfsStopTime> sorted_stop_times = gtfs.stop_times;
  std::sort(sorted_stop_times.begin(), sorted_stop_times.end(),
            [](const GtfsStopTime& a, const GtfsStopTime& b) {
              if (a.trip_id.v != b.trip_id.v) {
                return a.trip_id.v < b.trip_id.v;
              }
              return a.stop_sequence < b.stop_sequence;
            });

  // Generate steps from consecutive stop_times for each trip
  for (size_t i = 0; i < sorted_stop_times.size() - 1; ++i) {
    const auto& current_stop_time = sorted_stop_times[i];
    const auto& next_stop_time = sorted_stop_times[i + 1];

    // Only create step if consecutive stops are from the same trip
    if (current_stop_time.trip_id == next_stop_time.trip_id) {
      StopId origin_stop_id =
          result.mapping.gtfs_stop_id_to_stop_id[current_stop_time.stop_id];
      StopId destination_stop_id =
          result.mapping.gtfs_stop_id_to_stop_id[next_stop_time.stop_id];
      TripId trip_id =
          result.mapping.gtfs_trip_id_to_trip_id[current_stop_time.trip_id];

      Step step{origin_stop_id,
                destination_stop_id,
                TimeSinceServiceStart{current_stop_time.departure_time.seconds},
                TimeSinceServiceStart{next_stop_time.arrival_time.seconds},
                trip_id,
                trip_id};

      result.steps.push_back(step);
    }
  }

  return result;
}

}  // namespace vats5