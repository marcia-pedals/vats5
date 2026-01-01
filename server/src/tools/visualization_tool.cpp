#include <fstream>
#include <iostream>
#include <unordered_set>

#include "gtfs/gtfs.h"
#include "solver/data.h"
#include "solver/shortest_path.h"

using namespace vats5;

struct VisualizationStep {
  StopId origin_stop;
  StopId destination_stop;
  TimeSinceServiceStart origin_time;
  TimeSinceServiceStart destination_time;
  std::string trip_description;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VisualizationStep,
    origin_stop,
    destination_stop,
    origin_time,
    destination_time,
    trip_description
);

struct VisualizationPath {
  std::vector<VisualizationStep> steps;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VisualizationPath, steps);

struct VisualizationStop {
  GtfsStop gtfs_stop;
  bool included_in_reduced_graph;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VisualizationStop, gtfs_stop, included_in_reduced_graph
);

struct Visualization {
  // Key is actually a StopId but I don't know how to make the NLHOMANN macro
  // recognize that that is a number and can be serialized to an object key.
  // TODO: Figure out how to do that.
  std::unordered_map<int, VisualizationStop> stops;

  // Similar thing about the key.
  std::unordered_map<int, std::vector<std::vector<VisualizationPath>>> adjacent;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Visualization, stops, adjacent);

int main() {
  std::string gtfs_path = "../data/RG_20260109_BA_CT_SC_SM_AC";

  std::cout << "Loading GTFS data from: " << gtfs_path << std::endl;
  GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);

  std::cout << "Normalizing stops..." << std::endl;
  gtfs_day = GtfsNormalizeStops(gtfs_day);

  std::cout << "Getting steps..." << std::endl;
  StepsFromGtfs steps_from_gtfs =
      GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});

  std::cout << "Making adjacency list..." << std::endl;
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);

  std::unordered_set<StopId> bart_stops =
      GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

  std::unordered_set<StopId> intermediate_stops;
  for (const auto& gtfs_stop_id :
       {"mtc:san-jose-diridon-station",
        "mtc:mountain-view-station",
        "mtc:palo-alto-station",
        "mtc:salesforce-transit-center",
        "mtc:santa-clara-caltrain"}) {
    auto r_it = steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.find(
        GtfsStopId{gtfs_stop_id}
    );
    if (r_it == steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.end()) {
      std::cout << "OH NO: " << gtfs_stop_id << " not found!\n";
      return 1;
    }
    intermediate_stops.insert(r_it->second);
  }

  std::cout << "Reducing to minimal system steps..." << std::endl;
  auto minimal = ReduceToMinimalSystemPaths(adjacency_list, bart_stops);
  auto split = SplitPathsAt(minimal, intermediate_stops);

  std::cout << "BART stops count: " << bart_stops.size() << std::endl;
  std::cout << "Reduced adjacency list size: " << split.size() << std::endl;

  Visualization viz;
  for (const GtfsStop& stop : gtfs_day.stops) {
    const StopId stop_id =
        steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(stop.stop_id);
    viz.stops[stop_id.v] = VisualizationStop{
        .gtfs_stop = stop,
        .included_in_reduced_graph = bart_stops.contains(stop_id) ||
                                     intermediate_stops.contains(stop_id),
    };
  }

  for (const auto& [stop_id, path_groups] : split) {
    std::vector<std::vector<VisualizationPath>> viz_path_groups;
    for (const auto& path_group : path_groups) {
      std::vector<VisualizationPath> viz_paths;
      for (const auto& path : path_group) {
        VisualizationPath viz_path;
        for (const auto& step : path.steps) {
          viz_path.steps.push_back(VisualizationStep{
              .origin_stop = step.origin_stop,
              .destination_stop = step.destination_stop,
              .origin_time = step.origin_time,
              .destination_time = step.destination_time,
              .trip_description =
                  steps_from_gtfs.mapping.GetRouteDescFromTrip(step.origin_trip
                  ),
          });
        }
        viz_paths.push_back(viz_path);
      }
      viz_path_groups.push_back(viz_paths);
    }
    viz.adjacent[stop_id.v] = viz_path_groups;
  }

  nlohmann::json j = viz;
  std::ofstream out("../data/visualization.json");
  out << j.dump(2) << std::endl;
  std::cout << "Wrote visualization to ../data/visualization.json" << std::endl;

  return 0;
}
