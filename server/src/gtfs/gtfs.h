#pragma once

#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace vats5 {

struct GtfsStopId {
  std::string v;
  
  bool operator==(const GtfsStopId& other) const {
    return v == other.v;
  }
};

struct GtfsRouteId {
  std::string v;
  
  bool operator==(const GtfsRouteId& other) const {
    return v == other.v;
  }
};

struct GtfsTripId {
  std::string v;
  
  bool operator==(const GtfsTripId& other) const {
    return v == other.v;
  }
};

struct GtfsServiceId {
  std::string v;
  
  bool operator==(const GtfsServiceId& other) const {
    return v == other.v;
  }
};

struct GtfsTimeSinceServiceStart {
  int seconds;
  
  bool operator==(const GtfsTimeSinceServiceStart& other) const {
    return seconds == other.seconds;
  }
};

struct GtfsRouteDirectionId {
  GtfsRouteId route_id;
  int direction_id;
  
  bool operator==(const GtfsRouteDirectionId& other) const {
    return route_id == other.route_id && direction_id == other.direction_id;
  }
};

struct GtfsStop {
  GtfsStopId stop_id;
  std::string stop_name;
  double stop_lat;
  double stop_lon;
  std::optional<GtfsStopId> parent_station;
};

struct GtfsTrip {
  GtfsRouteDirectionId route_direction_id;
  GtfsTripId trip_id;
  GtfsServiceId service_id;
};

struct GtfsCalendar {
  GtfsServiceId service_id;
  bool monday;
  bool tuesday;
  bool wednesday;
  bool thursday;
  bool friday;
  bool saturday;
  bool sunday;
  std::string start_date;
  std::string end_date;
};

struct GtfsStopTime {
  GtfsTripId trip_id;
  GtfsStopId stop_id;
  int stop_sequence;
  GtfsTimeSinceServiceStart arrival_time;
  GtfsTimeSinceServiceStart departure_time;
};

struct GtfsRoute {
  GtfsRouteId route_id;
  std::string route_short_name;
  std::string route_long_name;
};

struct GtfsDirection {
  GtfsRouteDirectionId route_direction_id;
  std::string direction;
};

struct Gtfs {
  std::vector<GtfsStop> stops;
  std::vector<GtfsTrip> trips;
  std::vector<GtfsCalendar> calendar;
  std::vector<GtfsStopTime> stop_times;
  std::vector<GtfsRoute> routes;
  std::vector<GtfsDirection> directions;
};

struct GtfsDay {
  std::vector<GtfsStop> stops;
  std::vector<GtfsTrip> trips;
  std::vector<GtfsStopTime> stop_times;
  std::vector<GtfsRoute> routes;
  std::vector<GtfsDirection> directions;
};

// Load all GTFS data from a directory containing GTFS files
Gtfs GtfsLoad(const std::string& gtfs_directory_path);

// Filter GTFS data to only include services that run on the given date
// Date should be in "YYYYMMDD" format (e.g., "20240315")
GtfsDay GtfsFilterByDate(const Gtfs& gtfs, const std::string& date);

// Filter GTFS data to only include the specified trips
// All other fields are filtered to only include data associated with those trips
GtfsDay GtfsFilterByTrips(const GtfsDay& gtfs_day, const std::unordered_set<GtfsTripId>& trips_set);

}  // namespace vats5

// Hash functions for GTFS Id structs to use in unordered_set/unordered_map
namespace std {
template<>
struct hash<vats5::GtfsStopId> {
  size_t operator()(const vats5::GtfsStopId& stop_id) const {
    return hash<string>()(stop_id.v);
  }
};

template<>
struct hash<vats5::GtfsRouteId> {
  size_t operator()(const vats5::GtfsRouteId& route_id) const {
    return hash<string>()(route_id.v);
  }
};

template<>
struct hash<vats5::GtfsTripId> {
  size_t operator()(const vats5::GtfsTripId& trip_id) const {
    return hash<string>()(trip_id.v);
  }
};

template<>
struct hash<vats5::GtfsServiceId> {
  size_t operator()(const vats5::GtfsServiceId& service_id) const {
    return hash<string>()(service_id.v);
  }
};

template<>
struct hash<vats5::GtfsTimeSinceServiceStart> {
  size_t operator()(const vats5::GtfsTimeSinceServiceStart& time) const {
    return hash<int>()(time.seconds);
  }
};

template<>
struct hash<vats5::GtfsRouteDirectionId> {
  size_t operator()(const vats5::GtfsRouteDirectionId& route_dir_id) const {
    size_t h1 = hash<vats5::GtfsRouteId>()(route_dir_id.route_id);
    size_t h2 = hash<int>()(route_dir_id.direction_id);
    return h1 ^ (h2 << 1);
  }
};
}
