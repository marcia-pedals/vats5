#pragma once

#include <string>

namespace vats5 {
// Forward declarations
struct InitializeProblemStateResult;
struct GtfsDay;
struct DataGtfsMapping;
struct ProblemState;
}  // namespace vats5

namespace vats5::viz {

// Derives viz SQLite path: "problem.json" -> "problem-viz.sqlite"
std::string VizSqlitePath(const std::string& input_path);

// Write visualization data directly to a SQLite database file.
// Extracts required stops and paths from result/gtfs_day, skipping
// synthetic START/END boundary stops.
void WriteVisualizationSqlite(
    const InitializeProblemStateResult& result,
    const GtfsDay& gtfs_day,
    const DataGtfsMapping& mapping,
    const std::string& path
);

// Write a self-contained viz SQLite for a ProblemState, copying stop
// coordinates, routes, and trips from an existing viz SQLite file.
// The partial_solutions table is left empty.
void WriteProblemStateVisualizationSqlite(
    const ProblemState& state,
    const std::string& original_viz_path,
    const std::string& output_path
);

}  // namespace vats5::viz
