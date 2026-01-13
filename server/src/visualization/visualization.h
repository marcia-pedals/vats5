#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gtfs/gtfs.h"
#include "solver/data.h"
#include "solver/shortest_path.h"

namespace vats5 {

struct VisualizationStep {
  StopId origin_stop;
  StopId destination_stop;
  TimeSinceServiceStart origin_time;
  TimeSinceServiceStart destination_time;
  std::string trip_description;
  bool system_step;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VisualizationStep,
    origin_stop,
    destination_stop,
    origin_time,
    destination_time,
    trip_description,
    system_step
);

struct VisualizationPath {
  std::vector<VisualizationStep> steps;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VisualizationPath, steps);

struct VisualizationStop {
  GtfsStop gtfs_stop;
  bool system_stop;
  bool intermediate_stop;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VisualizationStop, gtfs_stop, system_stop, intermediate_stop
);

struct Visualization {
  // Key is actually a StopId but I don't know how to make the NLHOMANN macro
  // recognize that that is a number and can be serialized to an object key.
  std::unordered_map<int, VisualizationStop> stops;

  // Similar thing about the key.
  std::unordered_map<int, std::vector<std::vector<VisualizationPath>>> adjacent;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Visualization, stops, adjacent);

// Check if a trip is a BART trip based on its ID prefix.
bool IsBartTrip(const DataGtfsMapping& mapping, TripId trip_id);

// Create a visualization JSON object from the given data.
nlohmann::json MakeVisualization(
    const GtfsDay& gtfs_day,
    const StepsFromGtfs& steps_from_gtfs,
    const std::unordered_set<StopId>& bart_stops,
    const std::unordered_set<StopId>& intermediate_stops,
    const StepPathsAdjacencyList& split
);

}  // namespace vats5
