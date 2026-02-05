#include <CLI/CLI.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "gtfs/gtfs.h"
#include "solver/data.h"
#include "solver/steps_shortest_path.h"
#include "tools/reduction_output.h"

using namespace vats5;

int main(int argc, char* argv[]) {
  CLI::App app{"Benchmark ReduceToMinimalSystemPaths"};

  std::string gtfs_path = "../data/RG_20260108_all";
  std::string output_path;

  app.add_option("--gtfs-path", gtfs_path, "Path to GTFS data directory")
      ->default_val("../data/RG_20260108_all");
  app.add_option("--output,-o", output_path, "Output file path for results");

  CLI11_PARSE(app, argc, argv);

  std::cout << "Loading GTFS data from: " << gtfs_path << std::endl;
  GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);

  std::cout << "Normalizing stops..." << std::endl;
  auto normalize_start = std::chrono::high_resolution_clock::now();
  gtfs_day = GtfsNormalizeStops(gtfs_day);
  auto normalize_end = std::chrono::high_resolution_clock::now();
  auto normalize_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          normalize_end - normalize_start
      );
  std::cout << "Normalizing stops took " << normalize_duration.count() << " ms"
            << std::endl;

  std::cout << "Getting steps..." << std::endl;
  auto steps_start = std::chrono::high_resolution_clock::now();
  StepsFromGtfs steps_from_gtfs =
      GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});
  auto steps_end = std::chrono::high_resolution_clock::now();
  auto steps_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      steps_end - steps_start
  );
  std::cout << "Getting steps took " << steps_duration.count() << " ms"
            << std::endl;

  std::cout << "Making adjacency list..." << std::endl;
  auto adjacency_start = std::chrono::high_resolution_clock::now();
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);
  auto adjacency_end = std::chrono::high_resolution_clock::now();
  auto adjacency_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          adjacency_end - adjacency_start
      );
  std::cout << "Making adjacency list took " << adjacency_duration.count()
            << " ms" << std::endl;

  std::unordered_set<StopId> bart_stops =
      GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

  int num_origins_with_groups = 0;
  for (int i = 0; i < adjacency_list.NumStops(); ++i) {
    if (!adjacency_list.GetGroups(StopId{i}).empty()) {
      ++num_origins_with_groups;
    }
  }
  std::cout << "Number of stops in adjacency list: " << num_origins_with_groups
            << std::endl;
  std::cout << "Number of BART stops: " << bart_stops.size() << std::endl;

  std::cout << "Benchmarking ReduceToMinimalSystemPaths..." << std::endl;

  auto start = std::chrono::high_resolution_clock::now();
  StepPathsAdjacencyList minimal =
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
    ReductionOutput output{minimal, steps_from_gtfs.mapping};
    nlohmann::json j = output;
    std::ofstream out(output_path);
    out << j;
  }

  return 0;
}
