#pragma once

#include <cassert>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <unordered_map>
#include <variant>
#include <vector>

#include "gtfs/gtfs.h"

namespace vats5 {

struct StopId {
  int v;

  bool operator==(const StopId& other) const { return v == other.v; }
  bool operator!=(const StopId& other) const { return v != other.v; }
  bool operator<(const StopId& other) const { return v < other.v; }
};

struct TripId {
  int v;

  // A special marker indicating that a Step is just staying put.
  static const TripId NOOP;

  bool operator==(const TripId& other) const { return v == other.v; }
};

inline const TripId TripId::NOOP = TripId{-1};

struct TimeSinceServiceStart {
  int seconds = 0;

  static TimeSinceServiceStart Parse(const std::string& time_str) {
    GtfsTimeSinceServiceStart gtfs_time = ParseGtfsTime(time_str);
    return TimeSinceServiceStart{gtfs_time.seconds};
  }

  bool operator==(const TimeSinceServiceStart& other) const {
    return seconds == other.seconds;
  }
  bool operator!=(const TimeSinceServiceStart& other) const {
    return seconds != other.seconds;
  }
  bool operator<(const TimeSinceServiceStart& other) const {
    return seconds < other.seconds;
  }
  bool operator>(const TimeSinceServiceStart& other) const {
    return seconds > other.seconds;
  }
  bool operator<=(const TimeSinceServiceStart& other) const {
    return seconds <= other.seconds;
  }
  bool operator>=(const TimeSinceServiceStart& other) const {
    return seconds >= other.seconds;
  }

  std::string ToString() const {
    int hours = seconds / 3600;
    int minutes = (seconds % 3600) / 60;
    int secs = seconds % 60;
    return (hours < 10 ? "0" : "") + std::to_string(hours) + ":" +
           (minutes < 10 ? "0" : "") + std::to_string(minutes) + ":" +
           (secs < 10 ? "0" : "") + std::to_string(secs);
  }
};

struct Step {
  StopId origin_stop;
  StopId destination_stop;

  TimeSinceServiceStart origin_time;
  TimeSinceServiceStart destination_time;

  TripId origin_trip;
  TripId destination_trip;

  bool is_flex;

  int FlexDurationSeconds() const {
    assert(is_flex);
    return destination_time.seconds - origin_time.seconds;
  }

  bool operator==(const Step& other) const {
    return origin_stop == other.origin_stop &&
           destination_stop == other.destination_stop &&
           origin_time == other.origin_time &&
           destination_time == other.destination_time &&
           origin_trip == other.origin_trip &&
           destination_trip == other.destination_trip &&
           is_flex == other.is_flex;
  }
};

// Output operators for debugging/logging
inline std::ostream& operator<<(std::ostream& os, const StopId& value) {
  return os << "StopId{" << value.v << "}";
}

inline std::ostream& operator<<(
    std::ostream& os, const TimeSinceServiceStart& value
) {
  return os << "Time{" << value.seconds << "}";
}

inline std::ostream& operator<<(std::ostream& os, const TripId& value) {
  return os << "TripId{" << value.v << "}";
}

inline std::ostream& operator<<(std::ostream& os, const Step& value) {
  return os << "Step{stop: " << value.origin_stop.v << " -> "
            << value.destination_stop.v << ", trip: " << value.origin_trip.v
            << " -> " << value.destination_trip.v
            << ", time: " << value.origin_time.seconds << " -> "
            << value.destination_time.seconds << (value.is_flex ? ", flex" : "")
            << "}";
}

}  // namespace vats5

// Hash functions for solver Id structs to use in unordered_set/unordered_map
namespace std {
template <>
struct hash<vats5::StopId> {
  size_t operator()(const vats5::StopId& stop_id) const {
    return hash<int>()(stop_id.v);
  }
};

template <>
struct hash<vats5::TripId> {
  size_t operator()(const vats5::TripId& trip_id) const {
    return hash<int>()(trip_id.v);
  }
};

template <>
struct hash<vats5::TimeSinceServiceStart> {
  size_t operator()(const vats5::TimeSinceServiceStart& time) const {
    return hash<int>()(time.seconds);
  }
};

template <>
struct hash<vats5::Step> {
  size_t operator()(const vats5::Step& step) const {
    size_t h1 = hash<vats5::StopId>()(step.origin_stop);
    size_t h2 = hash<vats5::StopId>()(step.destination_stop);
    size_t h3 = hash<vats5::TimeSinceServiceStart>()(step.origin_time);
    size_t h4 = hash<vats5::TimeSinceServiceStart>()(step.destination_time);
    size_t h5 = hash<vats5::TripId>()(step.origin_trip);
    size_t h6 = hash<vats5::TripId>()(step.destination_trip);
    size_t h7 = hash<bool>()(step.is_flex);

    // Combine hashes using a simple hash combiner
    return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3) ^ (h5 << 4) ^ (h6 << 5) ^
           (h7 << 6);
  }
};
}  // namespace std

namespace vats5 {

struct GetStepsOptions {
  double max_walking_distance_meters = 500.0;
};

struct FlexTrip {
  StopId origin;
  StopId destination;
  int duration_seconds;
};

// Bidirectional mappings between GtfsStopId<->StopId, etc.
struct DataGtfsMapping {
  // StopId mappings
  std::unordered_map<GtfsStopId, StopId> gtfs_stop_id_to_stop_id;
  std::unordered_map<StopId, GtfsStopId> stop_id_to_gtfs_stop_id;
  std::unordered_map<std::string, std::vector<StopId>> stop_name_to_stop_ids;

  // TripId mappings
  std::unordered_map<GtfsTripId, TripId> gtfs_trip_id_to_trip_id;
  std::unordered_map<TripId, std::variant<GtfsTripId, FlexTrip>>
      trip_id_to_trip_info;
  std::unordered_map<TripId, std::string> trip_id_to_route_desc;

  StopId GetStopIdFromName(const std::string& stop_name) const {
    auto it = stop_name_to_stop_ids.find(stop_name);
    if (it == stop_name_to_stop_ids.end() || it->second.empty()) {
      throw std::runtime_error("Stop name '" + stop_name + "' not found");
    }
    if (it->second.size() > 1) {
      throw std::runtime_error(
          "Stop name '" + stop_name + "' is ambiguous (multiple stops)"
      );
    }
    return it->second[0];
  }

  std::string GetRouteDescFromTrip(TripId trip_id) const {
    auto it = trip_id_to_route_desc.find(trip_id);
    if (it == trip_id_to_route_desc.end()) {
      throw std::runtime_error("TripId not found in route description mapping");
    }
    return it->second;
  }
};

struct StepsFromGtfs {
  DataGtfsMapping mapping;
  std::vector<Step> steps;
};

StepsFromGtfs GetStepsFromGtfs(
    GtfsDay gtfs, const GetStepsOptions& options = {}
);

}  // namespace vats5
