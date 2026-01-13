#pragma once

#include <iostream>
#include <string>
#include <unordered_map>

#include "gtfs/gtfs.h"
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

struct CachedTestData {
  GtfsDay gtfs_day;
  StepsFromGtfs steps_from_gtfs;
  StepsAdjacencyList adjacency_list;
};

inline CachedTestData GetCachedTestData(const std::string& gtfs_path) {
  static std::unordered_map<std::string, CachedTestData> cache;

  auto it = cache.find(gtfs_path);
  if (it != cache.end()) {
    return it->second;
  }

  std::cout << "Loading for test: " << gtfs_path << "\n";
  GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);
  std::cout << "Normalizing stops...\n";
  gtfs_day = GtfsNormalizeStops(gtfs_day);
  std::cout << "Getting steps...\n";
  StepsFromGtfs steps_from_gtfs =
      GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});
  std::cout << "Making adjacency list...\n";
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);
  std::cout << "Done computing test data.\n";

  CachedTestData data{
      std::move(gtfs_day), std::move(steps_from_gtfs), std::move(adjacency_list)
  };
  cache[gtfs_path] = data;
  return data;
}

}  // namespace vats5
