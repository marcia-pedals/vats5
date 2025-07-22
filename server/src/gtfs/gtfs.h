#pragma once

#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace vats5 {

struct GtfsStopId {
  std::string v;

  bool operator==(const GtfsStopId& other) const { return v == other.v; }
};

struct GtfsRouteId {
  std::string v;

  bool operator==(const GtfsRouteId& other) const { return v == other.v; }
};

struct GtfsTripId {
  std::string v;

  bool operator==(const GtfsTripId& other) const { return v == other.v; }
};

struct GtfsServiceId {
  std::string v;

  bool operator==(const GtfsServiceId& other) const { return v == other.v; }
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

  bool operator==(const GtfsStop& other) const {
    return stop_id == other.stop_id && stop_name == other.stop_name &&
           stop_lat == other.stop_lat && stop_lon == other.stop_lon &&
           parent_station == other.parent_station;
  }
};

struct GtfsTrip {
  GtfsRouteDirectionId route_direction_id;
  GtfsTripId trip_id;
  GtfsServiceId service_id;

  bool operator==(const GtfsTrip& other) const {
    return route_direction_id == other.route_direction_id &&
           trip_id == other.trip_id && service_id == other.service_id;
  }
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

  bool operator==(const GtfsCalendar& other) const {
    return service_id == other.service_id && monday == other.monday &&
           tuesday == other.tuesday && wednesday == other.wednesday &&
           thursday == other.thursday && friday == other.friday &&
           saturday == other.saturday && sunday == other.sunday &&
           start_date == other.start_date && end_date == other.end_date;
  }
};

struct GtfsStopTime {
  GtfsTripId trip_id;
  GtfsStopId stop_id;
  int stop_sequence;
  GtfsTimeSinceServiceStart arrival_time;
  GtfsTimeSinceServiceStart departure_time;

  bool operator==(const GtfsStopTime& other) const {
    return trip_id == other.trip_id && stop_id == other.stop_id &&
           stop_sequence == other.stop_sequence &&
           arrival_time == other.arrival_time &&
           departure_time == other.departure_time;
  }
};

struct GtfsRoute {
  GtfsRouteId route_id;
  std::string route_short_name;
  std::string route_long_name;

  bool operator==(const GtfsRoute& other) const {
    return route_id == other.route_id &&
           route_short_name == other.route_short_name &&
           route_long_name == other.route_long_name;
  }
};

struct GtfsDirection {
  GtfsRouteDirectionId route_direction_id;
  std::string direction;

  bool operator==(const GtfsDirection& other) const {
    return route_direction_id == other.route_direction_id &&
           direction == other.direction;
  }
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

// Load GTFS data from a directory and return as GtfsDay
GtfsDay GtfsLoadDay(const std::string& gtfs_directory_path);

// Filter GTFS data to only include services that run on the given date
// Date should be in "YYYYMMDD" format (e.g., "20240315")
GtfsDay GtfsFilterByDate(const Gtfs& gtfs, const std::string& date);

// Filter GTFS data to only include the specified trips
// All other fields are filtered to only include data associated with those
// trips
GtfsDay GtfsFilterByTrips(const GtfsDay& gtfs_day,
                          const std::unordered_set<GtfsTripId>& trips_set);

// Parse GTFS time string (HH:MM:SS format) to GtfsTimeSinceServiceStart
GtfsTimeSinceServiceStart ParseGtfsTime(std::string_view time_str);

// Save GtfsDay to GTFS text files in the specified directory
void GtfsSave(const GtfsDay& gtfs_day, const std::string& gtfs_directory_path);

// Normalize stops by replacing child stops with their ultimate parent stations
// (1) All stop_ids in stop_times are replaced with their ultimate parent
// (recursively) (2) All stops that have parents are removed from the stops list
GtfsDay GtfsNormalizeStops(const GtfsDay& gtfs_day);

// Pretty printing for Google Test
inline void PrintTo(const GtfsStopId& stop_id, std::ostream* os) {
  *os << "GtfsStopId{\"" << stop_id.v << "\"}";
}

inline void PrintTo(const GtfsRouteId& route_id, std::ostream* os) {
  *os << "GtfsRouteId{\"" << route_id.v << "\"}";
}

inline void PrintTo(const GtfsTripId& trip_id, std::ostream* os) {
  *os << "GtfsTripId{\"" << trip_id.v << "\"}";
}

inline void PrintTo(const GtfsServiceId& service_id, std::ostream* os) {
  *os << "GtfsServiceId{\"" << service_id.v << "\"}";
}

inline void PrintTo(const GtfsTimeSinceServiceStart& time, std::ostream* os) {
  int hours = time.seconds / 3600;
  int minutes = (time.seconds % 3600) / 60;
  int seconds = time.seconds % 60;
  *os << "ParseGtfsTime(\"" << std::setfill('0') << std::setw(2) << hours << ":"
      << std::setw(2) << minutes << ":" << std::setw(2) << seconds << "\")";
}

inline void PrintTo(const GtfsRouteDirectionId& route_dir_id,
                    std::ostream* os) {
  *os << "GtfsRouteDirectionId{";
  PrintTo(route_dir_id.route_id, os);
  *os << ", " << route_dir_id.direction_id << "}";
}

inline void PrintTo(const GtfsTrip& trip, std::ostream* os) {
  *os << "GtfsTrip{";
  PrintTo(trip.route_direction_id, os);
  *os << ", ";
  PrintTo(trip.trip_id, os);
  *os << ", ";
  PrintTo(trip.service_id, os);
  *os << "}";
}

inline void PrintTo(const GtfsStop& stop, std::ostream* os) {
  *os << "GtfsStop{";
  PrintTo(stop.stop_id, os);
  *os << ", \"" << stop.stop_name << "\", " << stop.stop_lat << ", "
      << stop.stop_lon;
  if (stop.parent_station) {
    *os << ", ";
    PrintTo(*stop.parent_station, os);
  } else {
    *os << ", nullopt";
  }
  *os << "}";
}

inline void PrintTo(const GtfsRoute& route, std::ostream* os) {
  *os << "GtfsRoute{";
  PrintTo(route.route_id, os);
  *os << ", \"" << route.route_short_name << "\", \"" << route.route_long_name
      << "\"}";
}

inline void PrintTo(const GtfsDirection& direction, std::ostream* os) {
  *os << "GtfsDirection{";
  PrintTo(direction.route_direction_id, os);
  *os << ", \"" << direction.direction << "\"}";
}

inline void PrintTo(const GtfsStopTime& stop_time, std::ostream* os) {
  *os << "GtfsStopTime{";
  PrintTo(stop_time.trip_id, os);
  *os << ", ";
  PrintTo(stop_time.stop_id, os);
  *os << ", " << stop_time.stop_sequence << ", ";
  PrintTo(stop_time.arrival_time, os);
  *os << ", ";
  PrintTo(stop_time.departure_time, os);
  *os << "}";
}

inline void PrintTo(const GtfsCalendar& calendar, std::ostream* os) {
  *os << "GtfsCalendar{";
  PrintTo(calendar.service_id, os);
  *os << ", " << calendar.monday << ", " << calendar.tuesday << ", "
      << calendar.wednesday << ", " << calendar.thursday << ", "
      << calendar.friday << ", " << calendar.saturday << ", " << calendar.sunday
      << ", \"" << calendar.start_date << "\", \"" << calendar.end_date
      << "\"}";
}

}  // namespace vats5

// Hash functions for GTFS Id structs to use in unordered_set/unordered_map
namespace std {
template <>
struct hash<vats5::GtfsStopId> {
  size_t operator()(const vats5::GtfsStopId& stop_id) const {
    return hash<string>()(stop_id.v);
  }
};

template <>
struct hash<vats5::GtfsRouteId> {
  size_t operator()(const vats5::GtfsRouteId& route_id) const {
    return hash<string>()(route_id.v);
  }
};

template <>
struct hash<vats5::GtfsTripId> {
  size_t operator()(const vats5::GtfsTripId& trip_id) const {
    return hash<string>()(trip_id.v);
  }
};

template <>
struct hash<vats5::GtfsServiceId> {
  size_t operator()(const vats5::GtfsServiceId& service_id) const {
    return hash<string>()(service_id.v);
  }
};

template <>
struct hash<vats5::GtfsTimeSinceServiceStart> {
  size_t operator()(const vats5::GtfsTimeSinceServiceStart& time) const {
    return hash<int>()(time.seconds);
  }
};

template <>
struct hash<vats5::GtfsRouteDirectionId> {
  size_t operator()(const vats5::GtfsRouteDirectionId& route_dir_id) const {
    size_t h1 = hash<vats5::GtfsRouteId>()(route_dir_id.route_id);
    size_t h2 = hash<int>()(route_dir_id.direction_id);
    return h1 ^ (h2 << 1);
  }
};
}  // namespace std
