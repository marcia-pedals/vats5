#include "visualization/visualization.h"

#include <unordered_map>

#include "gtfs/gtfs.h"
#include "solver/tarel_graph.h"

namespace vats5::viz {

Visualization MakeVisualization(
    const ProblemState& state,
    const GtfsDay& gtfs_day
) {
  // Build a map from GtfsStopId to GtfsStop for direct lookup
  std::unordered_map<GtfsStopId, GtfsStop> gtfs_stop_by_id;
  for (const auto& stop : gtfs_day.stops) {
    gtfs_stop_by_id[stop.stop_id] = stop;
  }

  Visualization viz;
  for (const StopId& stop_id : state.required_stops) {
    // Skip START and END boundary stops (they are synthetic)
    if (stop_id == state.boundary.start || stop_id == state.boundary.end) {
      continue;
    }

    // Get the stop info from the state
    auto stop_info_it = state.stop_infos.find(stop_id);
    if (stop_info_it == state.stop_infos.end()) {
      throw std::runtime_error(
          "StopId " + std::to_string(stop_id.v) + " not found in stop_infos"
      );
    }
    const ProblemStateStopInfo& stop_info = stop_info_it->second;

    // Get the GtfsStop by ID for coordinates
    auto gtfs_stop_it = gtfs_stop_by_id.find(stop_info.gtfs_stop_id);
    if (gtfs_stop_it == gtfs_stop_by_id.end()) {
      throw std::runtime_error(
          "GtfsStopId '" + stop_info.gtfs_stop_id.v + "' not found in GTFS data"
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
