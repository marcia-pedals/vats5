#pragma once

#include <string>
#include <vector>

#include "gtfs/gtfs.h"

namespace vats5 {

struct GtfsFilterConfig {
  std::string input_dir;
  std::string date;
  std::vector<std::string> prefixes;
};

// Parse a TOML config file into a GtfsFilterConfig.
GtfsFilterConfig GtfsFilterConfigLoad(const std::string& config_path);

// Load GTFS data, apply prefix and date filters, and return the result.
// `subsequent_service_days` is passed on to GtfsFilterDateWithServiceDays.
GtfsDay GtfsFilterFromConfig(
    const GtfsFilterConfig& config, int subsequent_service_days = 1
);

// Removes trips not referenced by stop_times, then removes routes and
// directions not referenced by remaining trips.
void RemoveUnreferencedTripsRoutesAndDirections(GtfsDay& gtfs_day);

// Filter a Gtfs dataset to only include trips whose trip_id starts with one
// of the given prefixes.
Gtfs GtfsFilterByPrefixes(
    const Gtfs& gtfs, const std::vector<std::string>& prefixes
);

// The suffix that marks a trip id as belonging to the `day`'th service day
// after the target date, which is what distinguishes it from the target day's
// own trips (and from the other subsequent days'). Empty for day 0.
std::string ServiceDayTripIdSuffix(int day);

// The inverse of ServiceDayTripIdSuffix: which service day of a combined
// GtfsDay a trip id belongs to, 0 being the target date. Throws on a trip id
// carrying a malformed suffix.
int ServiceDayFromTripId(const GtfsTripId& trip_id);

// Filter GTFS data for a given date and combine it with the
// `subsequent_service_days` days that follow it, each shifted forward by 24:00
// per day and with its trip ids suffixed by ServiceDayTripIdSuffix.
//
// Every day but the last contributes all of its trips, including those running
// past midnight into the following day, which is also how the target day is
// treated. The last day is the horizon: its trips at or after 24:00 would land
// on a day that is not modelled at all, so they are dropped.
//
// Returns a single combined GtfsDay.
GtfsDay GtfsFilterDateWithServiceDays(
    const Gtfs& gtfs, const std::string& date, int subsequent_service_days = 1
);

}  // namespace vats5
