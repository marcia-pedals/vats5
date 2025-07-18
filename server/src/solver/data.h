#pragma once

#include <vector>

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
};

// Bidirectional mappings between GtfsStopId<->StopId, etc.
struct DataGtfsMapping {
    // TODO: Fill in.
};

struct StepsFromGtfs {
    DataGtfsMapping mapping;
    std::vector<Step> steps;
};

StepsFromGtfs GetStepsFromGtfs(GtfsDay gtfs);

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
}
