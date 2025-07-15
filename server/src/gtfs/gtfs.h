#pragma once

#include <string>
#include <vector>
#include <optional>

namespace vats5 {

struct Stop {
    std::string stop_id;
    std::string stop_name;
    double stop_lat;
    double stop_lon;
    std::optional<std::string> parent_station;
};

// Load all stops from the GTFS stops.txt file
std::vector<Stop> GtfsLoadStops(const std::string& stops_file_path);

} // namespace vats5
