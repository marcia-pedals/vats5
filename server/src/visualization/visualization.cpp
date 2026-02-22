#include "visualization/visualization.h"

#include <unordered_map>

#include "gtfs/gtfs.h"
#include "solver/tarel_graph.h"

namespace vats5::viz {

Visualization MakeVisualization(
    const ProblemState& state,
    const GtfsDay& gtfs_day
) {
  // Build a map from stop name to GtfsStop for lookup
  // (StopIds have been remapped by compaction, we only have stop_names)
  std::unordered_map<std::string, GtfsStop> gtfs_stop_by_name;
  for (const auto& stop : gtfs_day.stops) {
    gtfs_stop_by_name[stop.stop_name] = stop;
  }

  Visualization viz;
  for (const StopId& stop_id : state.required_stops) {
    // Skip START and END boundary stops (they are synthetic)
    if (stop_id == state.boundary.start || stop_id == state.boundary.end) {
      continue;
    }

    // Get the stop name from the state
    auto stop_name_it = state.stop_names.find(stop_id);
    if (stop_name_it == state.stop_names.end()) {
      throw std::runtime_error(
          "StopId " + std::to_string(stop_id.v) + " not found in stop_names"
      );
    }
    const std::string& stop_name = stop_name_it->second;

    // Get the GtfsStop by name
    auto gtfs_stop_it = gtfs_stop_by_name.find(stop_name);
    if (gtfs_stop_it == gtfs_stop_by_name.end()) {
      throw std::runtime_error(
          "Stop name '" + stop_name + "' not found in GTFS data"
      );
    }
    const GtfsStop& gtfs_stop = gtfs_stop_it->second;

    // Create viz::Stop
    viz.stops.push_back(Stop{
        .stop_id = gtfs_stop.stop_id.v,
        .stop_name = gtfs_stop.stop_name,
        .lat = gtfs_stop.stop_lat,
        .lon = gtfs_stop.stop_lon,
    });
  }

  return viz;
}

}  // namespace vats5::viz
