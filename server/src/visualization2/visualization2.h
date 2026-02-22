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

struct Visualization {
  std::vector<Stop> stops;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Visualization, stops);

// Create a Visualization from a ProblemState and GTFS data
// Filters out START and END boundary stops as they are synthetic
Visualization MakeVisualization(
    const ProblemState& state,
    const GtfsDay& gtfs_day
);

}  // namespace vats5::viz
