#pragma once

#include <string>
#include <vector>

#include "gtfs/gtfs.h"
#include "log.h"

namespace vats5 {

struct GtfsFilterConfig {
  std::string input_dir;
  std::string date;
  std::vector<std::string> prefixes;
};

// Parse a TOML config file into a GtfsFilterConfig.
GtfsFilterConfig GtfsFilterConfigLoad(const std::string& config_path);

// Load GTFS data, apply prefix and date filters, and return the result.
GtfsDay GtfsFilterFromConfig(
    const GtfsFilterConfig& config, const TextLogger& log
);

// Removes trips not referenced by stop_times, then removes routes and
// directions not referenced by remaining trips.
void RemoveUnreferencedTripsRoutesAndDirections(GtfsDay& gtfs_day);

// Filter a Gtfs dataset to only include trips whose trip_id starts with one
// of the given prefixes.
Gtfs GtfsFilterByPrefixes(
    const Gtfs& gtfs, const std::vector<std::string>& prefixes
);

// Filter GTFS data for a given date and combine with the next service day.
// Takes the target day and the next day's early morning trips (times < 24:00,
// shifted to +24:00). Returns a single combined GtfsDay.
GtfsDay GtfsFilterDateWithServiceDays(
    const Gtfs& gtfs, const std::string& date
);

}  // namespace vats5
