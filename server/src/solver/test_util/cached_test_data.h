#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtfs/gtfs.h"
#include "gtfs/gtfs_filter.h"
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

struct CachedTestData {
  GtfsDay gtfs_day;
  StepsFromGtfs steps_from_gtfs;
  StepsAdjacencyList<> adjacency_list;
};

struct FilterOptions {
  std::string raw_gtfs_path;
  std::string date;
  std::vector<std::string> prefixes;
  bool combine_service_days = true;
};

inline CachedTestData MakeCachedTestData(GtfsDay gtfs_day) {
  gtfs_day = GtfsNormalizeStops(gtfs_day);
  StepsFromGtfs steps_from_gtfs =
      GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});
  StepsAdjacencyList<> adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);
  return CachedTestData{
      std::move(gtfs_day), std::move(steps_from_gtfs), std::move(adjacency_list)
  };
}

inline CachedTestData GetCachedFilteredTestData(const FilterOptions& options) {
  static std::unordered_map<std::string, CachedTestData> cache;

  std::string cache_key = options.raw_gtfs_path + "|" + options.date + "|" +
                          (options.combine_service_days ? "csd" : "no-csd");
  for (const auto& prefix : options.prefixes) {
    cache_key += "|" + prefix;
  }

  auto it = cache.find(cache_key);
  if (it != cache.end()) {
    return it->second;
  }

  std::cout << "Loading raw GTFS for test: " << options.raw_gtfs_path << "\n";
  Gtfs gtfs = GtfsLoad(options.raw_gtfs_path);
  if (!options.prefixes.empty()) {
    std::cout << "Filtering by prefixes...\n";
    gtfs = GtfsFilterByPrefixes(gtfs, options.prefixes);
  }
  std::cout << "Filtering by date: " << options.date
            << (options.combine_service_days ? " (with service days)" : "")
            << "\n";
  GtfsDay gtfs_day = options.combine_service_days
                         ? GtfsFilterDateWithServiceDays(gtfs, options.date)
                         : GtfsFilterByDate(gtfs, options.date);
  std::cout << "Computing test data...\n";
  CachedTestData data = MakeCachedTestData(std::move(gtfs_day));
  std::cout << "Done computing test data.\n";

  cache[cache_key] = data;
  return data;
}

}  // namespace vats5
