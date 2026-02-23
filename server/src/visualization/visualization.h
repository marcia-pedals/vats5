#pragma once

#include <string>

namespace vats5 {
// Forward declarations
struct InitializeProblemStateResult;
struct GtfsDay;
struct DataGtfsMapping;
}  // namespace vats5

namespace vats5::viz {

// Write visualization data directly to a SQLite database file.
// Extracts required stops and paths from result/gtfs_day, skipping
// synthetic START/END boundary stops.
void WriteVisualizationSqlite(
    const InitializeProblemStateResult& result,
    const GtfsDay& gtfs_day,
    const DataGtfsMapping& mapping,
    const std::string& path
);

}  // namespace vats5::viz
