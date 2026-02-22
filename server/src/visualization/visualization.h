#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace vats5 {
// Forward declarations
struct ProblemState;
struct GtfsDay;
}  // namespace vats5

namespace vats5::viz {

struct Stop {
  std::string stop_id;
  std::string stop_name;
  double lat;
  double lon;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Stop, stop_id, stop_name, lat, lon);

struct VizPath {
  int depart_time;  // seconds since service start
  int arrive_time;
  int duration_seconds;
  bool is_flex;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VizPath, depart_time, arrive_time, duration_seconds, is_flex
);

struct VizPathGroup {
  std::string origin_stop_id;
  std::string destination_stop_id;
  std::vector<VizPath> paths;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VizPathGroup, origin_stop_id, destination_stop_id, paths
);

struct Visualization {
  std::vector<Stop> stops;
  std::vector<VizPathGroup> path_groups;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Visualization, stops, path_groups);

// Create a Visualization from a ProblemState and GTFS data
// Filters out START and END boundary stops as they are synthetic
Visualization MakeVisualization(
    const ProblemState& state, const GtfsDay& gtfs_day
);

// Write a Visualization to a SQLite database file
void WriteVisualizationSqlite(
    const Visualization& viz, const std::string& path
);

}  // namespace vats5::viz
