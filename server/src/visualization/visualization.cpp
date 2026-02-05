#include "visualization/visualization.h"

namespace vats5 {

bool IsBartTrip(const DataGtfsMapping& mapping, TripId trip_id) {
  auto it = mapping.trip_id_to_trip_info.find(trip_id);
  if (it == mapping.trip_id_to_trip_info.end()) {
    return false;
  }
  if (!std::holds_alternative<GtfsTripId>(it->second.v)) {
    return false;
  }
  const std::string& gtfs_trip_id = std::get<GtfsTripId>(it->second.v).v;
  return gtfs_trip_id.starts_with("BA:");
}

nlohmann::json MakeVisualization(
    const GtfsDay& gtfs_day,
    const StepsFromGtfs& steps_from_gtfs,
    const std::unordered_set<StopId>& bart_stops,
    const std::unordered_set<StopId>& intermediate_stops,
    const StepPathsAdjacencyList& split
) {
  Visualization viz;
  for (const GtfsStop& stop : gtfs_day.stops) {
    const StopId stop_id =
        steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(stop.stop_id);
    viz.stops[stop_id.v] = VisualizationStop{
        .gtfs_stop = stop,
        .system_stop = bart_stops.contains(stop_id),
        .intermediate_stop = intermediate_stops.contains(stop_id) &&
                             !bart_stops.contains(stop_id),
    };
  }

  for (const auto& [stop_id, path_groups] : split.adjacent) {
    std::vector<std::vector<VisualizationPath>> viz_path_groups;
    for (const auto& path_group : path_groups) {
      std::vector<VisualizationPath> viz_paths;
      for (const auto& path : path_group) {
        VisualizationPath viz_path;
        for (const auto& step : path.steps) {
          viz_path.steps.push_back(
              VisualizationStep{
                  .origin_stop = step.origin.stop,
                  .destination_stop = step.destination.stop,
                  .origin_time = step.origin.time,
                  .destination_time = step.destination.time,
                  .trip_description =
                      steps_from_gtfs.mapping.GetRouteDescFromTrip(
                          step.origin.trip
                      ),
                  .system_step =
                      IsBartTrip(steps_from_gtfs.mapping, step.origin.trip) &&
                      IsBartTrip(
                          steps_from_gtfs.mapping, step.destination.trip
                      ),
              }
          );
        }
        viz_paths.push_back(viz_path);
      }
      viz_path_groups.push_back(viz_paths);
    }
    viz.adjacent[stop_id.v] = viz_path_groups;
  }

  nlohmann::json j = viz;
  return j;
}

}  // namespace vats5
