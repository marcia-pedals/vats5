#pragma once

#include <string>
#include <vector>
#include <optional>

namespace vats5 {

struct StopId {
    std::string v;
};

struct Stop {
    StopId stop_id;
    std::string stop_name;
    double stop_lat;
    double stop_lon;
    std::optional<StopId> parent_station;
};

// Load all stops from the GTFS stops.txt file
std::vector<Stop> GtfsLoadStops(const std::string& stops_file_path);

} // namespace vats5
