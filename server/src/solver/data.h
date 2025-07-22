#pragma once

#include <limits>
#include <ostream>
#include <stdexcept>
#include <unordered_map>
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
  int seconds;

  // A special marker indicating that a Step can happen at any time.
  // When a Step's origin time is this, the destination time is the duration of
  // the step.
  static const TimeSinceServiceStart FLEX_STEP_MARKER;

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

  std::string ToString() const {
    int hours = seconds / 3600;
    int minutes = (seconds % 3600) / 60;
    int secs = seconds % 60;
    return (hours < 10 ? "0" : "") + std::to_string(hours) + ":" +
           (minutes < 10 ? "0" : "") + std::to_string(minutes) + ":" +
           (secs < 10 ? "0" : "") + std::to_string(secs);
  }
};

inline const TimeSinceServiceStart TimeSinceServiceStart::FLEX_STEP_MARKER =
    TimeSinceServiceStart{std::numeric_limits<int>::min()};

struct Step {
  StopId origin_stop;
  StopId destination_stop;

  TimeSinceServiceStart origin_time;
  TimeSinceServiceStart destination_time;

  TripId origin_trip;
  TripId destination_trip;

  bool operator==(const Step& other) const {
    return origin_stop == other.origin_stop &&
           destination_stop == other.destination_stop &&
           origin_time == other.origin_time &&
           destination_time == other.destination_time &&
           origin_trip == other.origin_trip &&
           destination_trip == other.destination_trip;
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
            << value.destination_time.seconds << "}";
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

    // Combine hashes using a simple hash combiner
    return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3) ^ (h5 << 4) ^ (h6 << 5);
  }
};
}  // namespace std

namespace vats5 {

// Bidirectional mappings between GtfsStopId<->StopId, etc.
struct DataGtfsMapping {
  // StopId mappings
  std::unordered_map<GtfsStopId, StopId> gtfs_stop_id_to_stop_id;
  std::unordered_map<StopId, GtfsStopId> stop_id_to_gtfs_stop_id;
  std::unordered_map<std::string, std::vector<StopId>> stop_name_to_stop_ids;

  // TripId mappings
  std::unordered_map<GtfsTripId, TripId> gtfs_trip_id_to_trip_id;
  std::unordered_map<TripId, GtfsTripId> trip_id_to_gtfs_trip_id;
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

StepsFromGtfs GetStepsFromGtfs(GtfsDay gtfs);

}  // namespace vats5
