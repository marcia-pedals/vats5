#pragma once

#include <string>

namespace vats5 {
// Forward declarations
struct ProblemState;
struct GtfsDay;
}  // namespace vats5

namespace vats5::viz {

// Write visualization data directly to a SQLite database file.
// Extracts required stops and paths from state/gtfs_day, skipping
// synthetic START/END boundary stops.
void WriteVisualizationSqlite(
    const ProblemState& state, const GtfsDay& gtfs_day, const std::string& path
);

}  // namespace vats5::viz
