#pragma once

#include <vector>

#include "gtfs/gtfs.h"

namespace vats5 {

struct StopId {
    int v;
};

// id -1 is a special marker indicating walking.
struct TripId {
    int v;
};

struct TimeSinceServiceStart {
    int seconds;
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
