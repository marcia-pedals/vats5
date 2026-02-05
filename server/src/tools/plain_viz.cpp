#include <fstream>
#include <iostream>
#include <unordered_set>

#include "visualization/visualization.h"

using namespace vats5;

int main() {
  const std::string gtfs_path = "../data/RG_20260108_all";

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

  std::cout << "Reducing to minimal system paths..." << std::endl;
  StepPathsAdjacencyList minimal =
      ReduceToMinimalSystemPaths(adjacency_list, bart_stops);

  std::cout << "Done. minimal has " << minimal.adjacent.size()
            << " origin stops." << std::endl;

  // Save visualization
  std::unordered_set<StopId> intermediate_stops;
  nlohmann::json viz = MakeVisualization(
      gtfs_day, steps_from_gtfs, bart_stops, intermediate_stops, minimal
  );
  std::string filename = "../data/visualization0.json";
  std::ofstream out(filename);
  out << viz.dump(2) << std::endl;
  std::cout << "Wrote " << filename << std::endl;

  return 0;
}
