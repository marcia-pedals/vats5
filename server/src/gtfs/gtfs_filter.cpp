#include "gtfs/gtfs_filter.h"

#include <algorithm>
#include <stdexcept>
#include <toml++/toml.hpp>
#include <unordered_set>

#include "util/date.h"

namespace vats5 {

GtfsFilterConfig GtfsFilterConfigLoad(const std::string& config_path) {
  toml::table config = toml::parse_file(config_path);

  auto input_dir = config["input_dir"].value<std::string>();
  auto date = config["date"].value<std::string>();
  auto output_dir = config["output_dir"].value<std::string>();

  if (!input_dir || !date || !output_dir) {
    throw std::runtime_error(
        "Config file must contain input_dir, date, and output_dir"
    );
  }

  std::vector<std::string> prefixes;
  if (auto arr = config["prefixes"].as_array()) {
    for (const auto& elem : *arr) {
      if (auto val = elem.value<std::string>()) {
        prefixes.push_back(*val);
      }
    }
  }

  return GtfsFilterConfig{
      .input_dir = *input_dir,
      .date = *date,
      .output_dir = *output_dir,
      .prefixes = std::move(prefixes),
  };
}

GtfsDay GtfsFilterFromConfig(const GtfsFilterConfig& config) {
  Gtfs gtfs = GtfsLoad(config.input_dir);

  if (!config.prefixes.empty()) {
    gtfs = GtfsFilterByPrefixes(gtfs, config.prefixes);
  }

  return GtfsFilterDateWithServiceDays(gtfs, config.date);
}

void RemoveUnreferencedTripsRoutesAndDirections(GtfsDay& gtfs_day) {
  // Collect trip_ids referenced by stop_times
  std::unordered_set<GtfsTripId> referenced_trip_ids;
  for (const auto& st : gtfs_day.stop_times) {
    referenced_trip_ids.insert(st.trip_id);
  }

  // Remove unreferenced trips and collect used route_direction_ids
  std::unordered_set<GtfsRouteDirectionId> used_route_direction_ids;
  std::unordered_set<GtfsRouteId> used_route_ids;
  std::erase_if(gtfs_day.trips, [&](const GtfsTrip& trip) {
    if (referenced_trip_ids.count(trip.trip_id)) {
      used_route_direction_ids.insert(trip.route_direction_id);
      used_route_ids.insert(trip.route_direction_id.route_id);
      return false;
    }
    return true;
  });

  // Remove unreferenced routes
  std::erase_if(gtfs_day.routes, [&](const GtfsRoute& route) {
    return !used_route_ids.count(route.route_id);
  });

  // Remove unreferenced directions
  std::erase_if(gtfs_day.directions, [&](const GtfsDirection& dir) {
    return !used_route_direction_ids.count(dir.route_direction_id);
  });
}

Gtfs GtfsFilterByPrefixes(
    const Gtfs& gtfs, const std::vector<std::string>& prefixes
) {
  std::unordered_set<GtfsTripId> trip_ids_set;
  for (const auto& trip : gtfs.trips) {
    for (const auto& prefix : prefixes) {
      if (trip.trip_id.v.substr(0, prefix.length()) == prefix) {
        trip_ids_set.insert(trip.trip_id);
        break;
      }
    }
  }
  return GtfsFilterByTrips(gtfs, trip_ids_set);
}

GtfsDay GtfsFilterDateWithServiceDays(
    const Gtfs& gtfs, const std::string& date
) {
  constexpr int kSecondsPerDay = 24 * 3600;

  // Filter to the target date.
  GtfsDay gtfs_day = GtfsFilterByDate(gtfs, date);

  // Take all >=24:00 stop times from the day before and add them to
  // `gtfs_day` with time -24:00.
  const std::string prev_date = OffsetDate(date, -1);
  GtfsDay prev_gtfs_day = GtfsFilterByDate(gtfs, prev_date);
  prev_gtfs_day.AppendToTripIds(":prev-sd");
  std::erase_if(prev_gtfs_day.stop_times, [](const GtfsStopTime& st) {
    return st.arrival_time.seconds < kSecondsPerDay;
  });
  RemoveUnreferencedTripsRoutesAndDirections(prev_gtfs_day);
  for (auto& stop_time : prev_gtfs_day.stop_times) {
    stop_time.arrival_time.seconds -= kSecondsPerDay;
    stop_time.departure_time.seconds -= kSecondsPerDay;
  }

  // Take all <24:00 stop times from the next day and add them to `gtfs_day`
  // with time +24:00.
  const std::string next_date = OffsetDate(date, 1);
  GtfsDay next_gtfs_day = GtfsFilterByDate(gtfs, next_date);
  next_gtfs_day.AppendToTripIds(":next-sd");
  std::erase_if(next_gtfs_day.stop_times, [](const GtfsStopTime& st) {
    return st.arrival_time.seconds >= kSecondsPerDay;
  });
  RemoveUnreferencedTripsRoutesAndDirections(next_gtfs_day);
  for (auto& stop_time : next_gtfs_day.stop_times) {
    stop_time.arrival_time.seconds += kSecondsPerDay;
    stop_time.departure_time.seconds += kSecondsPerDay;
  }

  return GtfsDayCombine({prev_gtfs_day, gtfs_day, next_gtfs_day});
}

}  // namespace vats5
