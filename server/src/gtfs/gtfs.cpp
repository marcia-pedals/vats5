#include "gtfs/gtfs.h"

#include <csv.hpp>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace vats5 {

GtfsTimeSinceServiceStart ParseGtfsTime(std::string_view time_str) {
  // Highly optimized parsing for HH:MM:SS format without string operations
  if (time_str.size() < 8 || time_str[2] != ':' || time_str[5] != ':') {
    throw std::runtime_error("Invalid time format: " + std::string(time_str));
  }

  // Validate that all time characters are digits
  if (time_str[0] < '0' || time_str[0] > '9' || time_str[1] < '0' ||
      time_str[1] > '9' || time_str[3] < '0' || time_str[3] > '9' ||
      time_str[4] < '0' || time_str[4] > '9' || time_str[6] < '0' ||
      time_str[6] > '9' || time_str[7] < '0' || time_str[7] > '9') {
    throw std::runtime_error(
        "Invalid time format - non-digit characters: " + std::string(time_str)
    );
  }

  // Parse directly from characters to avoid substr and stoi overhead
  int hours = (time_str[0] - '0') * 10 + (time_str[1] - '0');
  int minutes = (time_str[3] - '0') * 10 + (time_str[4] - '0');
  int seconds = (time_str[6] - '0') * 10 + (time_str[7] - '0');

  return GtfsTimeSinceServiceStart{hours * 3600 + minutes * 60 + seconds};
}

static std::vector<GtfsStop> GtfsLoadStops(const std::string& stops_file_path) {
  std::vector<GtfsStop> stops;

  try {
    csv::CSVReader reader(stops_file_path);

    // Estimate capacity based on file size
    auto file_size = std::filesystem::file_size(stops_file_path);
    stops.reserve(file_size / 100);  // Rough estimate: ~100 bytes per record

    for (csv::CSVRow& row : reader) {
      GtfsStop& stop = stops.emplace_back();
      stop.stop_id = GtfsStopId{row["stop_id"].get<std::string>()};
      stop.stop_name = row["stop_name"].get<std::string>();
      stop.stop_lat = row["stop_lat"].get<double>();
      stop.stop_lon = row["stop_lon"].get<double>();
      std::string parent_station_str = row["parent_station"].get<std::string>();
      if (parent_station_str.empty()) {
        stop.parent_station = std::nullopt;
      } else {
        stop.parent_station = GtfsStopId{std::move(parent_station_str)};
      }
    }
  } catch (const std::exception& e) {
    throw std::runtime_error(
        "Could not open or parse file: " + stops_file_path + " - " + e.what()
    );
  }

  return stops;
}

static std::vector<GtfsTrip> GtfsLoadTrips(const std::string& trips_file_path) {
  std::vector<GtfsTrip> trips;

  try {
    csv::CSVReader reader(trips_file_path);

    // Estimate capacity based on file size
    auto file_size = std::filesystem::file_size(trips_file_path);
    trips.reserve(file_size / 80);  // Rough estimate: ~80 bytes per record

    for (csv::CSVRow& row : reader) {
      GtfsTrip& trip = trips.emplace_back();
      trip.route_direction_id = GtfsRouteDirectionId{
          GtfsRouteId{row["route_id"].get<std::string>()},
          row["direction_id"].get<int>()
      };
      trip.trip_id = GtfsTripId{row["trip_id"].get<std::string>()};
      trip.service_id = GtfsServiceId{row["service_id"].get<std::string>()};
    }
  } catch (const std::exception& e) {
    throw std::runtime_error(
        "Could not open or parse file: " + trips_file_path + " - " + e.what()
    );
  }

  return trips;
}

static std::vector<GtfsCalendar> GtfsLoadCalendar(
    const std::string& calendar_file_path
) {
  std::vector<GtfsCalendar> calendars;

  try {
    csv::CSVReader reader(calendar_file_path);

    // Estimate capacity based on file size
    auto file_size = std::filesystem::file_size(calendar_file_path);
    calendars.reserve(file_size / 60);  // Rough estimate: ~60 bytes per record

    for (csv::CSVRow& row : reader) {
      GtfsCalendar& calendar = calendars.emplace_back();
      calendar.service_id = GtfsServiceId{row["service_id"].get<std::string>()};
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
  } catch (const std::exception& e) {
    throw std::runtime_error(
        "Could not open or parse file: " + calendar_file_path + " - " + e.what()
    );
  }

  return calendars;
}

static std::vector<GtfsStopTime> GtfsLoadStopTimes(
    const std::string& stop_times_file_path
) {
  std::vector<GtfsStopTime> stop_times;

  try {
    csv::CSVReader reader(stop_times_file_path);

    // Estimate capacity based on file size - this is the largest file
    auto file_size = std::filesystem::file_size(stop_times_file_path);
    stop_times.reserve(file_size / 70);  // Rough estimate: ~70 bytes per record

    for (csv::CSVRow& row : reader) {
      std::string_view arrival_time_str =
          row["arrival_time"].get<std::string_view>();
      std::string_view departure_time_str =
          row["departure_time"].get<std::string_view>();

      // Skip this stop time if either arrival or departure time is empty
      if (arrival_time_str.empty() || departure_time_str.empty()) {
        continue;
      }

      GtfsStopTime& stop_time = stop_times.emplace_back();
      stop_time.trip_id = GtfsTripId{row["trip_id"].get<std::string>()};
      stop_time.stop_id = GtfsStopId{row["stop_id"].get<std::string>()};
      stop_time.stop_sequence = row["stop_sequence"].get<int>();
      stop_time.arrival_time = ParseGtfsTime(arrival_time_str);
      stop_time.departure_time = ParseGtfsTime(departure_time_str);
    }
  } catch (const std::exception& e) {
    throw std::runtime_error(
        "Could not open or parse file: " + stop_times_file_path + " - " +
        e.what()
    );
  }

  return stop_times;
}

static std::vector<GtfsRoute> GtfsLoadRoutes(const std::string& routes_file_path
) {
  std::vector<GtfsRoute> routes;

  try {
    csv::CSVReader reader(routes_file_path);

    // Estimate capacity based on file size
    auto file_size = std::filesystem::file_size(routes_file_path);
    routes.reserve(file_size / 120);  // Rough estimate: ~120 bytes per record

    for (csv::CSVRow& row : reader) {
      GtfsRoute& route = routes.emplace_back();
      route.route_id = GtfsRouteId{row["route_id"].get<std::string>()};
      route.route_short_name = row["route_short_name"].get<std::string>();
      route.route_long_name = row["route_long_name"].get<std::string>();
    }
  } catch (const std::exception& e) {
    throw std::runtime_error(
        "Could not open or parse file: " + routes_file_path + " - " + e.what()
    );
  }

  return routes;
}

static std::vector<GtfsDirection> GtfsLoadDirections(
    const std::string& directions_file_path
) {
  std::vector<GtfsDirection> directions;

  try {
    csv::CSVReader reader(directions_file_path);

    // Estimate capacity based on file size
    auto file_size = std::filesystem::file_size(directions_file_path);
    directions.reserve(file_size / 50);  // Rough estimate: ~50 bytes per record

    for (csv::CSVRow& row : reader) {
      GtfsDirection& direction = directions.emplace_back();
      direction.route_direction_id = GtfsRouteDirectionId{
          GtfsRouteId{row["route_id"].get<std::string>()},
          row["direction_id"].get<int>()
      };
      direction.direction = row["direction"].get<std::string>();
    }
  } catch (const std::exception& e) {
    throw std::runtime_error(
        "Could not open or parse file: " + directions_file_path + " - " +
        e.what()
    );
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

GtfsDay GtfsLoadDay(const std::string& gtfs_directory_path) {
  Gtfs gtfs = GtfsLoad(gtfs_directory_path);

  if (!gtfs.calendar.empty()) {
    throw std::runtime_error(
        "GtfsLoadDay expects pre-filtered data with no calendar entries, but "
        "found " +
        std::to_string(gtfs.calendar.size()) + " calendar entries"
    );
  }

  GtfsDay gtfs_day;
  gtfs_day.stops = gtfs.stops;
  gtfs_day.trips = gtfs.trips;
  gtfs_day.stop_times = gtfs.stop_times;
  gtfs_day.routes = gtfs.routes;
  gtfs_day.directions = gtfs.directions;

  return gtfs_day;
}

static int GetDayOfWeek(const std::string& date) {
  // Parse YYYYMMDD format
  if (date.length() != 8) {
    throw std::runtime_error(
        "Invalid date format: " + date + " (expected YYYYMMDD)"
    );
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

static bool IsServiceActiveOnDay(
    const GtfsCalendar& calendar, const std::string& date, int day_of_week
) {
  // Check if date is within the service period
  if (date < calendar.start_date || date > calendar.end_date) {
    return false;
  }

  // Check if service runs on this day of week
  switch (day_of_week) {
    case 0:
      return calendar.sunday;  // Sunday
    case 1:
      return calendar.monday;  // Monday
    case 2:
      return calendar.tuesday;  // Tuesday
    case 3:
      return calendar.wednesday;  // Wednesday
    case 4:
      return calendar.thursday;  // Thursday
    case 5:
      return calendar.friday;  // Friday
    case 6:
      return calendar.saturday;  // Saturday
    default:
      return false;
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
      used_route_direction_ids.insert(
          trip.route_direction_id.route_id.v + ":" +
          std::to_string(trip.route_direction_id.direction_id)
      );
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
    std::string route_dir_key =
        direction.route_direction_id.route_id.v + ":" +
        std::to_string(direction.route_direction_id.direction_id);
    if (used_route_direction_ids.count(route_dir_key)) {
      result.directions.push_back(direction);
    }
  }

  return result;
}

GtfsDay GtfsFilterByTrips(
    const GtfsDay& gtfs_day, const std::unordered_set<GtfsTripId>& trips_set
) {
  GtfsDay result;

  // Step 1: Filter trips to only include specified trips
  std::unordered_set<std::string> used_route_direction_ids;
  for (const auto& trip : gtfs_day.trips) {
    if (trips_set.count(trip.trip_id)) {
      result.trips.push_back(trip);
      // Track route+direction combinations used
      used_route_direction_ids.insert(
          trip.route_direction_id.route_id.v + ":" +
          std::to_string(trip.route_direction_id.direction_id)
      );
    }
  }

  // Step 2: Filter stop times for specified trips
  std::unordered_set<std::string> used_stop_ids;
  for (const auto& stop_time : gtfs_day.stop_times) {
    if (trips_set.count(stop_time.trip_id)) {
      result.stop_times.push_back(stop_time);
      used_stop_ids.insert(stop_time.stop_id.v);
    }
  }

  // Step 3: Include only used stops and recursively include parent stations
  std::unordered_set<std::string> stops_to_include = used_stop_ids;

  // Recursively add parent stations
  bool added_new_stops = true;
  while (added_new_stops) {
    added_new_stops = false;
    for (const auto& stop : gtfs_day.stops) {
      if (stops_to_include.count(stop.stop_id.v)) {
        // If this stop has a parent station, add it to the set
        if (stop.parent_station &&
            !stops_to_include.count(stop.parent_station->v)) {
          stops_to_include.insert(stop.parent_station->v);
          added_new_stops = true;
        }
      }
    }
  }

  // Now include all stops that are in the final set
  for (const auto& stop : gtfs_day.stops) {
    if (stops_to_include.count(stop.stop_id.v)) {
      result.stops.push_back(stop);
    }
  }

  // Step 4: Include only used routes
  std::unordered_set<std::string> used_route_ids;
  for (const auto& trip : result.trips) {
    used_route_ids.insert(trip.route_direction_id.route_id.v);
  }
  for (const auto& route : gtfs_day.routes) {
    if (used_route_ids.count(route.route_id.v)) {
      result.routes.push_back(route);
    }
  }

  // Step 5: Include only used directions
  for (const auto& direction : gtfs_day.directions) {
    std::string route_dir_key =
        direction.route_direction_id.route_id.v + ":" +
        std::to_string(direction.route_direction_id.direction_id);
    if (used_route_direction_ids.count(route_dir_key)) {
      result.directions.push_back(direction);
    }
  }

  return result;
}

static std::string FormatGtfsTime(const GtfsTimeSinceServiceStart& time) {
  int total_seconds = time.seconds;
  int hours = total_seconds / 3600;
  int minutes = (total_seconds % 3600) / 60;
  int seconds = total_seconds % 60;

  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(2) << hours << ":" << std::setw(2)
      << minutes << ":" << std::setw(2) << seconds;
  return oss.str();
}

void GtfsSave(const GtfsDay& gtfs_day, const std::string& gtfs_directory_path) {
  // Create directory if it doesn't exist
  std::filesystem::create_directories(gtfs_directory_path);

  // Save stops.txt
  if (!gtfs_day.stops.empty()) {
    std::ofstream stops_file(gtfs_directory_path + "/stops.txt");
    stops_file << "stop_id,stop_name,stop_lat,stop_lon,parent_station\n";
    for (const auto& stop : gtfs_day.stops) {
      stops_file << stop.stop_id.v << ","
                 << "\"" << stop.stop_name << "\"," << std::fixed
                 << std::setprecision(6) << stop.stop_lat << "," << std::fixed
                 << std::setprecision(6) << stop.stop_lon << ",";
      if (stop.parent_station) {
        stops_file << stop.parent_station->v;
      }
      stops_file << "\n";
    }
  }

  // Save trips.txt
  if (!gtfs_day.trips.empty()) {
    std::ofstream trips_file(gtfs_directory_path + "/trips.txt");
    trips_file << "route_id,direction_id,trip_id,service_id\n";
    for (const auto& trip : gtfs_day.trips) {
      trips_file << trip.route_direction_id.route_id.v << ","
                 << trip.route_direction_id.direction_id << ","
                 << trip.trip_id.v << "," << trip.service_id.v << "\n";
    }
  }

  // Save stop_times.txt
  if (!gtfs_day.stop_times.empty()) {
    std::ofstream stop_times_file(gtfs_directory_path + "/stop_times.txt");
    stop_times_file
        << "trip_id,stop_id,stop_sequence,arrival_time,departure_time\n";
    for (const auto& stop_time : gtfs_day.stop_times) {
      stop_times_file << stop_time.trip_id.v << "," << stop_time.stop_id.v
                      << "," << stop_time.stop_sequence << ","
                      << FormatGtfsTime(stop_time.arrival_time) << ","
                      << FormatGtfsTime(stop_time.departure_time) << "\n";
    }
  }

  // Save routes.txt
  if (!gtfs_day.routes.empty()) {
    std::ofstream routes_file(gtfs_directory_path + "/routes.txt");
    routes_file << "route_id,route_short_name,route_long_name\n";
    for (const auto& route : gtfs_day.routes) {
      routes_file << route.route_id.v << ","
                  << "\"" << route.route_short_name << "\","
                  << "\"" << route.route_long_name << "\"\n";
    }
  }

  // Save directions.txt
  if (!gtfs_day.directions.empty()) {
    std::ofstream directions_file(gtfs_directory_path + "/directions.txt");
    directions_file << "route_id,direction_id,direction\n";
    for (const auto& direction : gtfs_day.directions) {
      directions_file << direction.route_direction_id.route_id.v << ","
                      << direction.route_direction_id.direction_id << ","
                      << "\"" << direction.direction << "\"\n";
    }
  }

  // Create a blank calendar.txt file
  std::ofstream calendar_file(gtfs_directory_path + "/calendar.txt");
  calendar_file << "service_id,monday,tuesday,wednesday,thursday,friday,"
                   "saturday,sunday,start_date,end_date\n";
}

GtfsDay GtfsNormalizeStops(const GtfsDay& gtfs_day) {
  GtfsDay result = gtfs_day;  // Start with a copy

  // Step 1: Build a map from stop_id to stop for efficient lookup
  std::unordered_map<std::string, const GtfsStop*> stop_lookup;
  for (const auto& stop : gtfs_day.stops) {
    stop_lookup[stop.stop_id.v] = &stop;
  }

  // Step 2: Create a function to find the ultimate parent of a stop
  auto find_ultimate_parent = [&stop_lookup](const std::string& stop_id
                              ) -> std::string {
    std::string current_id = stop_id;
    std::unordered_set<std::string> visited;  // Prevent infinite loops

    while (true) {
      // Check for cycles
      if (visited.count(current_id)) {
        // Cycle detected, return the original stop_id
        return stop_id;
      }
      visited.insert(current_id);

      auto it = stop_lookup.find(current_id);
      if (it == stop_lookup.end()) {
        // Stop not found, return current_id
        return current_id;
      }

      const GtfsStop* stop = it->second;
      if (!stop->parent_station) {
        // This stop has no parent, it's the ultimate parent
        return current_id;
      }

      // Move to the parent
      current_id = stop->parent_station->v;
    }
  };

  // Step 3: Replace all stop_ids in stop_times with their ultimate parents
  for (auto& stop_time : result.stop_times) {
    std::string ultimate_parent = find_ultimate_parent(stop_time.stop_id.v);
    stop_time.stop_id = GtfsStopId{ultimate_parent};
  }

  // Step 4: Remove all stops that have parents from the stops list
  result.stops.clear();
  for (const auto& stop : gtfs_day.stops) {
    if (!stop.parent_station) {
      // This stop has no parent, keep it
      result.stops.push_back(stop);
    }
  }

  return result;
}

}  // namespace vats5