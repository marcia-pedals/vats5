#include "gtfs.h"

#include <csv.hpp>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <unordered_set>
#include <ctime>
#include <iomanip>

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

static int GetDayOfWeek(const std::string& date) {
  // Parse YYYYMMDD format
  if (date.length() != 8) {
    throw std::runtime_error("Invalid date format: " + date + " (expected YYYYMMDD)");
  }
  
  int year = std::stoi(date.substr(0, 4));
  int month = std::stoi(date.substr(4, 2));
  int day = std::stoi(date.substr(6, 2));
  
  // Create tm structure
  std::tm tm = {};
  tm.tm_year = year - 1900;
  tm.tm_mon = month - 1;
  tm.tm_mday = day;
  
  // Convert to time_t and back to get day of week
  std::time_t time = std::mktime(&tm);
  std::tm* result = std::localtime(&time);
  
  // Return 0=Sunday, 1=Monday, ..., 6=Saturday
  return result->tm_wday;
}

static bool IsServiceActiveOnDay(const Calendar& calendar, const std::string& date, int day_of_week) {
  // Check if date is within the service period
  if (date < calendar.start_date || date > calendar.end_date) {
    return false;
  }
  
  // Check if service runs on this day of week
  switch (day_of_week) {
    case 0: return calendar.sunday;    // Sunday
    case 1: return calendar.monday;    // Monday
    case 2: return calendar.tuesday;   // Tuesday
    case 3: return calendar.wednesday; // Wednesday
    case 4: return calendar.thursday;  // Thursday
    case 5: return calendar.friday;    // Friday
    case 6: return calendar.saturday;  // Saturday
    default: return false;
  }
}

GtfsDay GtfsFilterByDate(const Gtfs& gtfs, const std::string& date) {
  GtfsDay result;
  
  // Get day of week for the given date
  int day_of_week = GetDayOfWeek(date);
  
  // Step 1: Find all service IDs that are active on this date
  std::unordered_set<std::string> active_service_ids;
  for (const auto& calendar : gtfs.calendar) {
    if (IsServiceActiveOnDay(calendar, date, day_of_week)) {
      active_service_ids.insert(calendar.service_id.v);
    }
  }
  
  // Step 2: Filter trips that use active services
  std::unordered_set<std::string> active_trip_ids;
  std::unordered_set<std::string> used_route_direction_ids;
  for (const auto& trip : gtfs.trips) {
    if (active_service_ids.count(trip.service_id.v)) {
      result.trips.push_back(trip);
      active_trip_ids.insert(trip.trip_id.v);
      // Track route+direction combinations used
      used_route_direction_ids.insert(trip.route_direction_id.route_id.v + ":" + std::to_string(trip.route_direction_id.direction_id));
    }
  }
  
  // Step 3: Filter stop times for active trips
  std::unordered_set<std::string> used_stop_ids;
  for (const auto& stop_time : gtfs.stop_times) {
    if (active_trip_ids.count(stop_time.trip_id.v)) {
      result.stop_times.push_back(stop_time);
      used_stop_ids.insert(stop_time.stop_id.v);
    }
  }
  
  // Step 4: Include all stops
  result.stops = gtfs.stops;
  
  // Step 5: Include only used routes
  std::unordered_set<std::string> used_route_ids;
  for (const auto& trip : result.trips) {
    used_route_ids.insert(trip.route_direction_id.route_id.v);
  }
  for (const auto& route : gtfs.routes) {
    if (used_route_ids.count(route.route_id.v)) {
      result.routes.push_back(route);
    }
  }
  
  // Step 6: Include only used directions
  for (const auto& direction : gtfs.directions) {
    std::string route_dir_key = direction.route_direction_id.route_id.v + ":" + std::to_string(direction.route_direction_id.direction_id);
    if (used_route_direction_ids.count(route_dir_key)) {
      result.directions.push_back(direction);
    }
  }
  
  return result;
}

}  // namespace vats5