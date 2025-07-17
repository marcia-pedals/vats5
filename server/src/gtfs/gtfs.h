#pragma once

#include <optional>
#include <string>
#include <vector>

namespace vats5 {

struct StopId {
  std::string v;
};

struct RouteId {
  std::string v;
};

struct TripId {
  std::string v;
};

struct ServiceId {
  std::string v;
};

struct TimeSinceServiceStart {
  int seconds;
};

struct RouteDirectionId {
  RouteId route_id;
  int direction_id;
};

struct Stop {
  StopId stop_id;
  std::string stop_name;
  double stop_lat;
  double stop_lon;
  std::optional<StopId> parent_station;
};

struct Trip {
  RouteDirectionId route_direction_id;
  TripId trip_id;
  ServiceId service_id;
};

struct Calendar {
  ServiceId service_id;
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

struct StopTime {
  TripId trip_id;
  StopId stop_id;
  int stop_sequence;
  std::string arrival_time;
  std::string departure_time;
};

struct Route {
  RouteId route_id;
  std::string route_short_name;
  std::string route_long_name;
};

struct Direction {
  RouteDirectionId route_direction_id;
  std::string direction;
};

struct Gtfs {
  std::vector<Stop> stops;
  std::vector<Trip> trips;
  std::vector<Calendar> calendar;
  std::vector<StopTime> stop_times;
  std::vector<Route> routes;
  std::vector<Direction> directions;
};

// Load all GTFS data from a directory containing GTFS files
Gtfs GtfsLoad(const std::string& gtfs_directory_path);

}  // namespace vats5
