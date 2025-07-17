#include "gtfs.h"

#include <csv.hpp>
#include <iostream>
#include <stdexcept>

namespace vats5 {

std::vector<Stop> GtfsLoadStops(const std::string &stops_file_path) {
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

std::vector<Trip> GtfsLoadTrips(const std::string &trips_file_path) {
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

std::vector<Calendar> GtfsLoadCalendar(const std::string &calendar_file_path) {
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

std::vector<StopTime> GtfsLoadStopTimes(const std::string &stop_times_file_path) {
  std::vector<StopTime> stop_times;

  try {
    csv::CSVReader reader(stop_times_file_path);

    for (csv::CSVRow &row : reader) {
      StopTime &stop_time = stop_times.emplace_back();
      stop_time.trip_id = TripId{row["trip_id"].get<std::string>()};
      stop_time.stop_id = StopId{row["stop_id"].get<std::string>()};
      stop_time.stop_sequence = row["stop_sequence"].get<int>();
      stop_time.arrival_time = row["arrival_time"].get<std::string>();
      stop_time.departure_time = row["departure_time"].get<std::string>();
    }
  } catch (const std::exception &e) {
    throw std::runtime_error(
        "Could not open or parse file: " + stop_times_file_path + " - " + e.what());
  }

  return stop_times;
}

std::vector<Route> GtfsLoadRoutes(const std::string &routes_file_path) {
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

std::vector<Direction> GtfsLoadDirections(const std::string &directions_file_path) {
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

}  // namespace vats5