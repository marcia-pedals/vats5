#pragma once

#include <optional>
#include <string>
#include <vector>

namespace vats5 {

struct GtfsStopId {
  std::string v;
};

struct GtfsRouteId {
  std::string v;
};

struct GtfsTripId {
  std::string v;
};

struct GtfsServiceId {
  std::string v;
};

struct GtfsTimeSinceServiceStart {
  int seconds;
};

struct GtfsRouteDirectionId {
  GtfsRouteId route_id;
  int direction_id;
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

}  // namespace vats5
