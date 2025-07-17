#include "gtfs.h"

#include <csv.hpp>
#include <iostream>
#include <stdexcept>
#include <sstream>

namespace vats5 {

static TimeSinceServiceStart ParseGtfsTime(const std::string& time_str) {
  std::istringstream ss(time_str);
  std::string hours_str, minutes_str, seconds_str;
  
  if (!std::getline(ss, hours_str, ':') ||
      !std::getline(ss, minutes_str, ':') ||
      !std::getline(ss, seconds_str)) {
    throw std::runtime_error("Invalid time format: " + time_str);
  }
  
  int hours = std::stoi(hours_str);
  int minutes = std::stoi(minutes_str);
  int seconds = std::stoi(seconds_str);
  
  return TimeSinceServiceStart{hours * 3600 + minutes * 60 + seconds};
}

static std::vector<Stop> GtfsLoadStops(const std::string &stops_file_path) {
  std::vector<Stop> stops;

  try {
    csv::CSVReader reader(stops_file_path);

    for (csv::CSVRow &row : reader) {
      Stop &stop = stops.emplace_back();
      stop.stop_id = StopId{row["stop_id"].get<std::string>()};
      stop.stop_name = row["stop_name"].get<std::string>();
      stop.stop_lat = row["stop_lat"].get<double>();
      stop.stop_lon = row["stop_lon"].get<double>();
      std::string parent_station_str = row["parent_station"].get<std::string>();
      if (parent_station_str.empty()) {
        stop.parent_station = std::nullopt;
      } else {
        stop.parent_station = StopId{std::move(parent_station_str)};
      }
    }
  } catch (const std::exception &e) {
    throw std::runtime_error(
        "Could not open or parse file: " + stops_file_path + " - " + e.what());
  }

  return stops;
}

static std::vector<Trip> GtfsLoadTrips(const std::string &trips_file_path) {
  std::vector<Trip> trips;

  try {
    csv::CSVReader reader(trips_file_path);

    for (csv::CSVRow &row : reader) {
      Trip &trip = trips.emplace_back();
      trip.route_direction_id = RouteDirectionId{RouteId{row["route_id"].get<std::string>()}, row["direction_id"].get<int>()};
      trip.trip_id = TripId{row["trip_id"].get<std::string>()};
      trip.service_id = ServiceId{row["service_id"].get<std::string>()};
    }
  } catch (const std::exception &e) {
    throw std::runtime_error(
        "Could not open or parse file: " + trips_file_path + " - " + e.what());
  }

  return trips;
}

static std::vector<Calendar> GtfsLoadCalendar(const std::string &calendar_file_path) {
  std::vector<Calendar> calendars;

  try {
    csv::CSVReader reader(calendar_file_path);

    for (csv::CSVRow &row : reader) {
      Calendar &calendar = calendars.emplace_back();
      calendar.service_id = ServiceId{row["service_id"].get<std::string>()};
      calendar.monday = row["monday"].get<std::string>() == "1";
      calendar.tuesday = row["tuesday"].get<std::string>() == "1";
      calendar.wednesday = row["wednesday"].get<std::string>() == "1";
      calendar.thursday = row["thursday"].get<std::string>() == "1";
      calendar.friday = row["friday"].get<std::string>() == "1";
      calendar.saturday = row["saturday"].get<std::string>() == "1";
      calendar.sunday = row["sunday"].get<std::string>() == "1";
      calendar.start_date = row["start_date"].get<std::string>();
      calendar.end_date = row["end_date"].get<std::string>();
    }
  } catch (const std::exception &e) {
    throw std::runtime_error(
        "Could not open or parse file: " + calendar_file_path + " - " + e.what());
  }

  return calendars;
}

static std::vector<StopTime> GtfsLoadStopTimes(const std::string &stop_times_file_path) {
  std::vector<StopTime> stop_times;

  try {
    csv::CSVReader reader(stop_times_file_path);

    for (csv::CSVRow &row : reader) {
      std::string arrival_time_str = row["arrival_time"].get<std::string>();
      std::string departure_time_str = row["departure_time"].get<std::string>();
      
      // Skip this stop time if either arrival or departure time is empty
      if (arrival_time_str.empty() || departure_time_str.empty()) {
        continue;
      }
      
      StopTime &stop_time = stop_times.emplace_back();
      stop_time.trip_id = TripId{row["trip_id"].get<std::string>()};
      stop_time.stop_id = StopId{row["stop_id"].get<std::string>()};
      stop_time.stop_sequence = row["stop_sequence"].get<int>();
      stop_time.arrival_time = ParseGtfsTime(arrival_time_str);
      stop_time.departure_time = ParseGtfsTime(departure_time_str);
    }
  } catch (const std::exception &e) {
    throw std::runtime_error(
        "Could not open or parse file: " + stop_times_file_path + " - " + e.what());
  }

  return stop_times;
}

static std::vector<Route> GtfsLoadRoutes(const std::string &routes_file_path) {
  std::vector<Route> routes;

  try {
    csv::CSVReader reader(routes_file_path);

    for (csv::CSVRow &row : reader) {
      Route &route = routes.emplace_back();
      route.route_id = RouteId{row["route_id"].get<std::string>()};
      route.route_short_name = row["route_short_name"].get<std::string>();
      route.route_long_name = row["route_long_name"].get<std::string>();
    }
  } catch (const std::exception &e) {
    throw std::runtime_error(
        "Could not open or parse file: " + routes_file_path + " - " + e.what());
  }

  return routes;
}

static std::vector<Direction> GtfsLoadDirections(const std::string &directions_file_path) {
  std::vector<Direction> directions;

  try {
    csv::CSVReader reader(directions_file_path);

    for (csv::CSVRow &row : reader) {
      Direction &direction = directions.emplace_back();
      direction.route_direction_id = RouteDirectionId{RouteId{row["route_id"].get<std::string>()}, row["direction_id"].get<int>()};
      direction.direction = row["direction"].get<std::string>();
    }
  } catch (const std::exception &e) {
    throw std::runtime_error(
        "Could not open or parse file: " + directions_file_path + " - " + e.what());
  }

  return directions;
}

Gtfs GtfsLoad(const std::string& gtfs_directory_path) {
  Gtfs gtfs;
  
  gtfs.stops = GtfsLoadStops(gtfs_directory_path + "/stops.txt");
  gtfs.trips = GtfsLoadTrips(gtfs_directory_path + "/trips.txt");
  gtfs.calendar = GtfsLoadCalendar(gtfs_directory_path + "/calendar.txt");
  gtfs.stop_times = GtfsLoadStopTimes(gtfs_directory_path + "/stop_times.txt");
  gtfs.routes = GtfsLoadRoutes(gtfs_directory_path + "/routes.txt");
  gtfs.directions = GtfsLoadDirections(gtfs_directory_path + "/directions.txt");
  
  return gtfs;
}

}  // namespace vats5