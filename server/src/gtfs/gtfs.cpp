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
      trip.route_id = RouteId{row["route_id"].get<std::string>()};
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

}  // namespace vats5