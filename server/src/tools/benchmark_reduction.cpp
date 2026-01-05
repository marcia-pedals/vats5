#include <chrono>
#include <fstream>
#include <iostream>

#include "gtfs/gtfs.h"
#include "solver/data.h"
#include "solver/shortest_path.h"

using namespace vats5;

int main(int argc, char* argv[]) {
  const std::string gtfs_path = "../data/RG_20260108_all";

  std::string output_path;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--output" && i + 1 < argc) {
      output_path = argv[++i];
    }
  }

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

  std::cout << "Number of stops in adjacency list: "
            << adjacency_list.adjacent.size() << std::endl;
  std::cout << "Number of BART stops: " << bart_stops.size() << std::endl;

  std::cout << "Benchmarking ReduceToMinimalSystemPaths..." << std::endl;

  auto start = std::chrono::high_resolution_clock::now();
  PathsAdjacencyList minimal =
      ReduceToMinimalSystemPaths(adjacency_list, bart_stops);
  auto end = std::chrono::high_resolution_clock::now();

  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "ReduceToMinimalSystemPaths took " << duration.count() << " ms"
            << std::endl;

  std::cout << "Number of origin stops in result: " << minimal.adjacent.size()
            << std::endl;

  int total_paths = 0;
  for (const auto& [origin, path_groups] : minimal.adjacent) {
    for (const auto& path_group : path_groups) {
      total_paths += path_group.size();
    }
  }
  std::cout << "Total paths in result: " << total_paths << std::endl;

  if (!output_path.empty()) {
    std::cout << "Saving result to: " << output_path << std::endl;
    nlohmann::json j = minimal;
    std::ofstream out(output_path);
    out << j.dump(2) << std::endl;
  }

  return 0;
}
