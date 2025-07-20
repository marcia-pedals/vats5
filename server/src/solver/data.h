#pragma once

#include <vector>
#include <unordered_map>
#include <ostream>

#include "gtfs/gtfs.h"

namespace vats5 {

struct StopId {
    int v;
    
    bool operator==(const StopId& other) const {
        return v == other.v;
    }
};

// id -1 is a special marker indicating walking.
struct TripId {
    int v;
    
    bool operator==(const TripId& other) const {
        return v == other.v;
    }
};

struct TimeSinceServiceStart {
    int seconds;
    
    bool operator==(const TimeSinceServiceStart& other) const {
        return seconds == other.seconds;
    }
};

struct Step {
    StopId origin_stop;
    StopId destination_stop;

    // origin_time = -1 is a special marker indicating that this trip can happen at any time. In
    // this case, `destination_time` is the trip duration.
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

inline std::ostream& operator<<(std::ostream& os, const TimeSinceServiceStart& value) {
    return os << "Time{" << value.seconds << "}";
}

inline std::ostream& operator<<(std::ostream& os, const TripId& value) {
    return os << "TripId{" << value.v << "}";
}

inline std::ostream& operator<<(std::ostream& os, const Step& value) {
    return os << "Step{stop: " << value.origin_stop.v << " -> " << value.destination_stop.v 
              << ", trip: " << value.origin_trip.v << " -> " << value.destination_trip.v
              << ", time: " << value.origin_time.seconds << " -> " << value.destination_time.seconds << "}";
}

}  // namespace vats5

// Hash functions for solver Id structs to use in unordered_set/unordered_map
namespace std {
template<>
struct hash<vats5::StopId> {
  size_t operator()(const vats5::StopId& stop_id) const {
    return hash<int>()(stop_id.v);
  }
};

template<>
struct hash<vats5::TripId> {
  size_t operator()(const vats5::TripId& trip_id) const {
    return hash<int>()(trip_id.v);
  }
};

template<>
struct hash<vats5::TimeSinceServiceStart> {
  size_t operator()(const vats5::TimeSinceServiceStart& time) const {
    return hash<int>()(time.seconds);
  }
};

template<>
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
}

namespace vats5 {

// Bidirectional mappings between GtfsStopId<->StopId, etc.
struct DataGtfsMapping {
    // StopId mappings
    std::unordered_map<GtfsStopId, StopId> gtfs_stop_id_to_stop_id;
    std::unordered_map<StopId, GtfsStopId> stop_id_to_gtfs_stop_id;
    
    // TripId mappings
    std::unordered_map<GtfsTripId, TripId> gtfs_trip_id_to_trip_id;
    std::unordered_map<TripId, GtfsTripId> trip_id_to_gtfs_trip_id;
};

struct StepsFromGtfs {
    DataGtfsMapping mapping;
    std::vector<Step> steps;
};

StepsFromGtfs GetStepsFromGtfs(GtfsDay gtfs);

}  // namespace vats5
