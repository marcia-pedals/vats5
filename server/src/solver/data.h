#pragma once

#include <cassert>
#include <nlohmann/json.hpp>
#include <ostream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
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
inline void to_json(nlohmann::json& j, const StopId& id) { j = id.v; }
inline void from_json(const nlohmann::json& j, StopId& id) {
  id.v = j.get<int>();
}

struct PlainEdge {
  StopId a;
  StopId b;
  bool operator==(const PlainEdge&) const = default;
};
inline void to_json(nlohmann::json& j, const PlainEdge& e) {
  j = nlohmann::json::array({e.a.v, e.b.v});
}
inline void from_json(const nlohmann::json& j, PlainEdge& e) {
  e.a.v = j[0];
  e.b.v = j[1];
}

struct PlainWeightedEdge {
  StopId a;
  StopId b;
  int weight;
};

struct TripId {
  int v;

  // A special marker indicating that a Step is just staying put.
  static const TripId NOOP;

  bool operator==(const TripId& other) const { return v == other.v; }
  bool operator!=(const TripId& other) const { return v != other.v; }
  bool operator<(const TripId& other) const { return v < other.v; }
};
inline void to_json(nlohmann::json& j, const TripId& id) { j = id.v; }
inline void from_json(const nlohmann::json& j, TripId& id) {
  id.v = j.get<int>();
}

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
    std::string prefix = seconds < 0 ? "-" : "";
    int abs_seconds = seconds < 0 ? -seconds : seconds;
    int hours = abs_seconds / 3600;
    int minutes = (abs_seconds % 3600) / 60;
    int secs = abs_seconds % 60;
    return prefix +
           (hours < 10 ? "0" : "") + std::to_string(hours) + ":" +
           (minutes < 10 ? "0" : "") + std::to_string(minutes) + ":" +
           (secs < 10 ? "0" : "") + std::to_string(secs);
  }
};
inline void to_json(nlohmann::json& j, const TimeSinceServiceStart& t) {
  j = t.seconds;
}
inline void from_json(const nlohmann::json& j, TimeSinceServiceStart& t) {
  t.seconds = j.get<int>();
}

struct StepPartitionId {
  int v;
  bool operator==(const StepPartitionId&) const = default;
  auto operator<=>(const StepPartitionId&) const = default;
  static const StepPartitionId NONE;
};
inline const StepPartitionId StepPartitionId::NONE{-1};
inline void to_json(nlohmann::json& j, const StepPartitionId& id) { j = id.v; }
inline void from_json(const nlohmann::json& j, StepPartitionId& id) {
  id.v = j.get<int>();
}

struct StepEndpoint {
  StopId stop;
  bool is_flex;
  StepPartitionId partition;
  TimeSinceServiceStart time;
  TripId trip;

  bool operator==(const StepEndpoint&) const = default;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(StepEndpoint, stop, is_flex, partition, time, trip)

struct Step {
  StepEndpoint origin;
  StepEndpoint destination;

  bool is_flex;

  int FlexDurationSeconds() const {
    assert(is_flex);
    return destination.time.seconds - origin.time.seconds;
  }

  int DurationSeconds() const {
    return destination.time.seconds - origin.time.seconds;
  }

  PlainEdge Plain() const {
    return PlainEdge{origin.stop, destination.stop};
  }

  PlainWeightedEdge PlainWeighted() const {
    return PlainWeightedEdge{origin.stop, destination.stop, DurationSeconds()};
  }

  static Step PrimitiveScheduled(
      StopId origin_stop, StopId destination_stop,
      TimeSinceServiceStart origin_time, TimeSinceServiceStart destination_time,
      TripId trip, StepPartitionId partition = StepPartitionId::NONE
  ) {
    return Step{
        StepEndpoint{origin_stop, false, partition, origin_time, trip},
        StepEndpoint{destination_stop, false, partition, destination_time, trip},
        false
    };
  }

  static Step PrimitiveFlex(
      StopId origin_stop, StopId destination_stop,
      int duration_seconds, TripId trip,
      StepPartitionId partition = StepPartitionId::NONE
  ) {
    return Step{
        StepEndpoint{origin_stop, true, partition, TimeSinceServiceStart{0}, trip},
        StepEndpoint{destination_stop, true, partition, TimeSinceServiceStart{duration_seconds}, trip},
        true
    };
  }

  bool operator==(const Step& other) const {
    return origin == other.origin &&
           destination == other.destination &&
           is_flex == other.is_flex;
  }
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    Step,
    origin,
    destination,
    is_flex
)

struct Path {
  Step merged_step;
  std::vector<Step> steps;

  int DurationSeconds() const {
    return merged_step.DurationSeconds();
  }

  int IntermediateStopCount() const {
    if (steps.size() <= 1) {
      return 0;
    }
    return steps.size() - 1;
  }

  template <typename Visitor>
  void VisitIntermediateStops(Visitor visitor) const {
    for (size_t i = 0; i + 1 < steps.size(); ++i) {
      visitor(steps[i].destination.stop);
    }
  }
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Path, merged_step, steps)

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
  return os << "Step{stop: " << value.origin.stop.v << " -> "
            << value.destination.stop.v << ", trip: " << value.origin.trip.v
            << " -> " << value.destination.trip.v
            << ", time: " << value.origin.time.seconds << " -> "
            << value.destination.time.seconds << (value.is_flex ? ", flex" : "")
            << "}";
}

struct StopPosition {
  StopId stop_id;
  double x_meters;  // approximate x position in meters
  double y_meters;  // approximate y position in meters
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(StopPosition, stop_id, x_meters, y_meters)

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
struct hash<vats5::PlainEdge> {
  size_t operator()(const vats5::PlainEdge& edge) const {
    size_t h1 = hash<vats5::StopId>()(edge.a);
    size_t h2 = hash<vats5::StopId>()(edge.b);
    return h1 ^ (h2 << 1);
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
struct hash<vats5::StepEndpoint> {
  size_t operator()(const vats5::StepEndpoint& ep) const {
    size_t h1 = hash<vats5::StopId>()(ep.stop);
    size_t h2 = hash<vats5::TimeSinceServiceStart>()(ep.time);
    size_t h3 = hash<vats5::TripId>()(ep.trip);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

template <>
struct hash<vats5::Step> {
  size_t operator()(const vats5::Step& step) const {
    size_t h1 = hash<vats5::StepEndpoint>()(step.origin);
    size_t h2 = hash<vats5::StepEndpoint>()(step.destination);
    size_t h3 = hash<bool>()(step.is_flex);
    return h1 ^ (h2 << 3) ^ (h3 << 6);
  }
};
}  // namespace std

namespace vats5 {

struct GetStepsOptions {
  double max_walking_distance_meters = 500.0;
  double walking_speed_ms = 1.0;  // meters per second
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    GetStepsOptions, max_walking_distance_meters, walking_speed_ms
)

struct FlexTrip {
  StopId origin;
  StopId destination;
  int duration_seconds;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    FlexTrip, origin, destination, duration_seconds
)

// Wrapper for std::variant<GtfsTripId, FlexTrip> for JSON serialization
struct TripInfo {
  std::variant<GtfsTripId, FlexTrip> v;
};
inline void to_json(nlohmann::json& j, const TripInfo& info) {
  if (std::holds_alternative<GtfsTripId>(info.v)) {
    j = {{"type", "gtfs"}, {"gtfs_trip_id", std::get<GtfsTripId>(info.v)}};
  } else {
    j = {{"type", "flex"}, {"flex_trip", std::get<FlexTrip>(info.v)}};
  }
}
inline void from_json(const nlohmann::json& j, TripInfo& info) {
  std::string type = j.at("type").get<std::string>();
  if (type == "gtfs") {
    info.v = j.at("gtfs_trip_id").get<GtfsTripId>();
  } else {
    info.v = j.at("flex_trip").get<FlexTrip>();
  }
}

// Bidirectional mappings between GtfsStopId<->StopId, etc.
struct DataGtfsMapping {
  // StopId mappings
  std::unordered_map<GtfsStopId, StopId> gtfs_stop_id_to_stop_id;
  std::unordered_map<StopId, GtfsStopId> stop_id_to_gtfs_stop_id;
  std::unordered_map<std::string, std::vector<StopId>> stop_name_to_stop_ids;
  std::unordered_map<StopId, std::string> stop_id_to_stop_name;

  // TripId mappings
  std::unordered_map<GtfsTripId, TripId> gtfs_trip_id_to_trip_id;
  std::unordered_map<TripId, TripInfo> trip_id_to_trip_info;
  std::unordered_map<TripId, std::string> trip_id_to_route_desc;

  // Position mappings (stop with id k is at index k)
  std::vector<StopPosition> stop_positions;

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

// Custom JSON serialization for DataGtfsMapping
// We convert maps keyed by custom types to arrays of pairs
inline void to_json(nlohmann::json& j, const DataGtfsMapping& m) {
  // Convert gtfs_stop_id_to_stop_id
  std::vector<std::pair<GtfsStopId, StopId>> gtfs_stop_id_pairs(
      m.gtfs_stop_id_to_stop_id.begin(), m.gtfs_stop_id_to_stop_id.end()
  );
  // Convert stop_id_to_gtfs_stop_id
  std::vector<std::pair<int, GtfsStopId>> stop_id_pairs;
  for (const auto& [k, v] : m.stop_id_to_gtfs_stop_id) {
    stop_id_pairs.emplace_back(k.v, v);
  }
  // Convert stop_id_to_stop_name
  std::vector<std::pair<int, std::string>> stop_id_name_pairs;
  for (const auto& [k, v] : m.stop_id_to_stop_name) {
    stop_id_name_pairs.emplace_back(k.v, v);
  }
  // Convert gtfs_trip_id_to_trip_id
  std::vector<std::pair<GtfsTripId, TripId>> gtfs_trip_id_pairs(
      m.gtfs_trip_id_to_trip_id.begin(), m.gtfs_trip_id_to_trip_id.end()
  );
  // Convert trip_id_to_trip_info
  std::vector<std::pair<int, TripInfo>> trip_id_info_pairs;
  for (const auto& [k, v] : m.trip_id_to_trip_info) {
    trip_id_info_pairs.emplace_back(k.v, v);
  }
  // Convert trip_id_to_route_desc
  std::vector<std::pair<int, std::string>> trip_id_route_pairs;
  for (const auto& [k, v] : m.trip_id_to_route_desc) {
    trip_id_route_pairs.emplace_back(k.v, v);
  }
  j = nlohmann::json{
      {"gtfs_stop_id_to_stop_id", gtfs_stop_id_pairs},
      {"stop_id_to_gtfs_stop_id", stop_id_pairs},
      {"stop_name_to_stop_ids", m.stop_name_to_stop_ids},
      {"stop_id_to_stop_name", stop_id_name_pairs},
      {"gtfs_trip_id_to_trip_id", gtfs_trip_id_pairs},
      {"trip_id_to_trip_info", trip_id_info_pairs},
      {"trip_id_to_route_desc", trip_id_route_pairs},
      {"stop_positions", m.stop_positions}
  };
}

inline void from_json(const nlohmann::json& j, DataGtfsMapping& m) {
  // Convert gtfs_stop_id_to_stop_id
  auto gtfs_stop_id_pairs =
      j.at("gtfs_stop_id_to_stop_id")
          .get<std::vector<std::pair<GtfsStopId, StopId>>>();
  for (const auto& [k, v] : gtfs_stop_id_pairs) {
    m.gtfs_stop_id_to_stop_id[k] = v;
  }
  // Convert stop_id_to_gtfs_stop_id
  auto stop_id_pairs = j.at("stop_id_to_gtfs_stop_id")
                           .get<std::vector<std::pair<int, GtfsStopId>>>();
  for (const auto& [k, v] : stop_id_pairs) {
    m.stop_id_to_gtfs_stop_id[StopId{k}] = v;
  }
  // stop_name_to_stop_ids
  m.stop_name_to_stop_ids =
      j.at("stop_name_to_stop_ids")
          .get<std::unordered_map<std::string, std::vector<StopId>>>();
  // Convert stop_id_to_stop_name
  auto stop_id_name_pairs =
      j.at("stop_id_to_stop_name")
          .get<std::vector<std::pair<int, std::string>>>();
  for (const auto& [k, v] : stop_id_name_pairs) {
    m.stop_id_to_stop_name[StopId{k}] = v;
  }
  // Convert gtfs_trip_id_to_trip_id
  auto gtfs_trip_id_pairs =
      j.at("gtfs_trip_id_to_trip_id")
          .get<std::vector<std::pair<GtfsTripId, TripId>>>();
  for (const auto& [k, v] : gtfs_trip_id_pairs) {
    m.gtfs_trip_id_to_trip_id[k] = v;
  }
  // Convert trip_id_to_trip_info
  auto trip_id_info_pairs =
      j.at("trip_id_to_trip_info").get<std::vector<std::pair<int, TripInfo>>>();
  for (const auto& [k, v] : trip_id_info_pairs) {
    m.trip_id_to_trip_info[TripId{k}] = v;
  }
  // Convert trip_id_to_route_desc
  auto trip_id_route_pairs =
      j.at("trip_id_to_route_desc")
          .get<std::vector<std::pair<int, std::string>>>();
  for (const auto& [k, v] : trip_id_route_pairs) {
    m.trip_id_to_route_desc[TripId{k}] = v;
  }
  // Convert stop_positions
  m.stop_positions = j.at("stop_positions").get<std::vector<StopPosition>>();
}

struct StepsFromGtfs {
  DataGtfsMapping mapping;
  std::vector<Step> steps;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(StepsFromGtfs, mapping, steps)

StepsFromGtfs GetStepsFromGtfs(
    GtfsDay gtfs, const GetStepsOptions& options = {}
);

// Returns all stops visited by all trips with `trip_id_prefix`.
//
// This is useful for finding all the stops in a given system within the
// regional GTFS, because the trip ids are prefixed by the system, e.g. BART
// trips are "BA:...".
std::unordered_set<StopId> GetStopsForTripIdPrefix(
    const GtfsDay& gtfs,
    const DataGtfsMapping& mapping,
    const std::string& trip_id_prefix
);

}  // namespace vats5
