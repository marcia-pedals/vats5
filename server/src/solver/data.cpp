#include "solver/data.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace vats5 {

StepsFromGtfs GetStepsFromGtfs(GtfsDay gtfs, const GetStepsOptions& options) {
  StepsFromGtfs result;
  result.mapping = DataGtfsMapping{};
  result.steps = {};

  // Create mappings for stops and trips, starting with ID 1
  int next_stop_id = 1;
  int next_trip_id = 1;

  // Create bidirectional mappings for stops and populate stop name mapping
  for (const auto& gtfs_stop : gtfs.stops) {
    StopId stop_id{next_stop_id++};
    result.mapping.gtfs_stop_id_to_stop_id[gtfs_stop.stop_id] = stop_id;
    result.mapping.stop_id_to_gtfs_stop_id[stop_id] = gtfs_stop.stop_id;
    result.mapping.stop_name_to_stop_ids[gtfs_stop.stop_name].push_back(stop_id
    );
    result.mapping.stop_id_to_stop_name[stop_id] = gtfs_stop.stop_name;
  }

  // Create bidirectional mappings for trips and build route descriptions in one
  // loop
  for (const auto& trip : gtfs.trips) {
    TripId trip_id{next_trip_id++};
    result.mapping.gtfs_trip_id_to_trip_id[trip.trip_id] = trip_id;
    result.mapping.trip_id_to_trip_info[trip_id] = TripInfo{trip.trip_id};

    // Find the route for this trip
    const GtfsRoute* route = nullptr;
    for (const auto& r : gtfs.routes) {
      if (r.route_id == trip.route_direction_id.route_id) {
        route = &r;
        break;
      }
    }

    // Find the direction for this trip
    const GtfsDirection* direction = nullptr;
    for (const auto& d : gtfs.directions) {
      if (d.route_direction_id.route_id == trip.route_direction_id.route_id &&
          d.route_direction_id.direction_id ==
              trip.route_direction_id.direction_id) {
        direction = &d;
        break;
      }
    }

    // Build the route description
    std::string route_desc;
    if (route && direction) {
      route_desc = route->route_short_name + " " + direction->direction;
    } else {
      // Fallback to trip_id if route or direction not found
      route_desc = trip.trip_id.v;
    }

    result.mapping.trip_id_to_route_desc[trip_id] = route_desc;
  }

  // Sort stop_times by trip_id and stop_sequence to ensure correct order
  std::vector<GtfsStopTime> sorted_stop_times = gtfs.stop_times;
  std::sort(
      sorted_stop_times.begin(),
      sorted_stop_times.end(),
      [](const GtfsStopTime& a, const GtfsStopTime& b) {
        if (a.trip_id.v != b.trip_id.v) {
          return a.trip_id.v < b.trip_id.v;
        }
        return a.stop_sequence < b.stop_sequence;
      }
  );

  // Generate steps from consecutive stop_times for each trip
  for (int i = 0; i < static_cast<int>(sorted_stop_times.size()) - 1; ++i) {
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

      Step step{
          origin_stop_id,
          destination_stop_id,
          TimeSinceServiceStart{current_stop_time.departure_time.seconds},
          TimeSinceServiceStart{next_stop_time.arrival_time.seconds},
          trip_id,
          trip_id,
          false  // is_flex
      };

      result.steps.push_back(step);
    }
  }

  // Add walking connections between stops within 500m
  // Create stop positions and convert lat/lon to meters
  // Using approximate conversion: 1 degree lat ≈ 111,000m, 1 degree lon ≈
  // 111,000m * cos(lat)
  // Resize vector to accommodate all stop IDs (IDs start at 1)
  result.mapping.stop_positions.resize(next_stop_id);
  for (const auto& gtfs_stop : gtfs.stops) {
    auto it = result.mapping.gtfs_stop_id_to_stop_id.find(gtfs_stop.stop_id);
    if (it != result.mapping.gtfs_stop_id_to_stop_id.end()) {
      double lat_rad = 37.0 * M_PI / 180.0;
      double x_meters = gtfs_stop.stop_lon * 111000.0 * std::cos(lat_rad);
      double y_meters = gtfs_stop.stop_lat * 111000.0;

      result.mapping.stop_positions[it->second.v] =
          StopPosition{it->second, x_meters, y_meters};
    }
  }

  // Create a sorted copy for sliding window algorithm
  std::vector<StopPosition> stop_positions = result.mapping.stop_positions;

  // Sort stops by x_meters (longitude equivalent) for sliding window
  std::sort(
      stop_positions.begin(),
      stop_positions.end(),
      [](const StopPosition& a, const StopPosition& b) {
        return a.x_meters < b.x_meters;
      }
  );

  // Sliding window to find stops within specified walking distance
  const double WALKING_SPEED_MS = 1.0;  // 1 meter per second

  for (size_t i = 0; i < stop_positions.size(); ++i) {
    const auto& current_stop = stop_positions[i];

    // Find window of stops within x-coordinate range (500m in x direction)
    size_t window_start = i;
    size_t window_end = i;

    // Extend window to include all stops within x-coordinate range
    while (window_end < stop_positions.size() &&
           stop_positions[window_end].x_meters <=
               current_stop.x_meters + options.max_walking_distance_meters) {
      window_end++;
    }

    // Check actual distances within the window
    for (size_t j = window_start; j < window_end; ++j) {
      if (i == j) continue;  // Skip same stop

      const auto& other_stop = stop_positions[j];

      // Calculate actual Euclidean distance in meters
      double dx = current_stop.x_meters - other_stop.x_meters;
      double dy = current_stop.y_meters - other_stop.y_meters;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance <= options.max_walking_distance_meters) {
        // Create FlexTrip and add to trip mapping
        TripId walk_trip_id{next_trip_id++};
        FlexTrip flex_trip{
            current_stop.stop_id,
            other_stop.stop_id,
            static_cast<int>(
                distance / WALKING_SPEED_MS
            )  // duration in seconds
        };

        result.mapping.trip_id_to_trip_info[walk_trip_id] = TripInfo{flex_trip};

        // Create route description for walking - lookup stop names
        std::string current_stop_name =
            result.mapping.stop_id_to_stop_name[current_stop.stop_id];
        std::string other_stop_name =
            result.mapping.stop_id_to_stop_name[other_stop.stop_id];

        std::string walk_desc =
            "Walk from " + current_stop_name + " to " + other_stop_name;
        result.mapping.trip_id_to_route_desc[walk_trip_id] = walk_desc;

        // Create walking step with flex time markers
        Step walk_step{
            current_stop.stop_id,
            other_stop.stop_id,
            TimeSinceServiceStart{0},  // origin time
            TimeSinceServiceStart{static_cast<int>(distance / WALKING_SPEED_MS)
            },  // duration as destination time
            walk_trip_id,
            walk_trip_id,
            true  // is_flex
        };

        result.steps.push_back(walk_step);

        // Create the reverse walking step (bidirectional walking)
        TripId reverse_walk_trip_id{next_trip_id++};
        FlexTrip reverse_flex_trip{
            other_stop.stop_id,
            current_stop.stop_id,
            static_cast<int>(distance / WALKING_SPEED_MS)  // same duration
        };

        result.mapping.trip_id_to_trip_info[reverse_walk_trip_id] =
            TripInfo{reverse_flex_trip};

        std::string reverse_walk_desc =
            "Walk from " + other_stop_name + " to " + current_stop_name;
        result.mapping.trip_id_to_route_desc[reverse_walk_trip_id] =
            reverse_walk_desc;

        Step reverse_walk_step{
            other_stop.stop_id,
            current_stop.stop_id,
            TimeSinceServiceStart{0},  // origin time
            TimeSinceServiceStart{static_cast<int>(distance / WALKING_SPEED_MS)
            },  // duration as destination time
            reverse_walk_trip_id,
            reverse_walk_trip_id,
            true  // is_flex
        };

        result.steps.push_back(reverse_walk_step);
      }
    }
  }

  return result;
}

std::unordered_set<StopId> GetStopsForTripIdPrefix(
    const GtfsDay& gtfs,
    const DataGtfsMapping& mapping,
    const std::string& trip_id_prefix
) {
  std::unordered_set<StopId> result;

  // Find all trip IDs that match the prefix
  std::unordered_set<GtfsTripId> matching_trip_ids;
  for (const auto& trip : gtfs.trips) {
    if (trip.trip_id.v.substr(0, trip_id_prefix.size()) == trip_id_prefix) {
      matching_trip_ids.insert(trip.trip_id);
    }
  }

  // Collect all stops visited by matching trips
  for (const auto& stop_time : gtfs.stop_times) {
    if (matching_trip_ids.count(stop_time.trip_id) > 0) {
      auto it = mapping.gtfs_stop_id_to_stop_id.find(stop_time.stop_id);
      if (it != mapping.gtfs_stop_id_to_stop_id.end()) {
        result.insert(it->second);
      }
    }
  }

  return result;
}

}  // namespace vats5