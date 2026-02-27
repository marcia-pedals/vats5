#pragma once

#include <format>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtfs/gtfs.h"
#include "gtfs/gtfs_filter.h"
#include "log.h"
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

struct CachedTestData {
  GtfsDay gtfs_day;
  StepsFromGtfs steps_from_gtfs;
  StepsAdjacencyList adjacency_list;
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
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);
  return CachedTestData{
      std::move(gtfs_day), std::move(steps_from_gtfs), std::move(adjacency_list)
  };
}

inline CachedTestData GetCachedFilteredTestData(
    const FilterOptions& options, const TextLogger& log = OstreamLogger(std::cout)
) {
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

  log(std::format("Loading raw GTFS for test: {}", options.raw_gtfs_path));
  Gtfs gtfs = GtfsLoad(options.raw_gtfs_path);
  if (!options.prefixes.empty()) {
    log("Filtering by prefixes...");
    gtfs = GtfsFilterByPrefixes(gtfs, options.prefixes);
  }
  log(std::format(
      "Filtering by date: {}{}",
      options.date,
      options.combine_service_days ? " (with service days)" : ""
  ));
  GtfsDay gtfs_day = options.combine_service_days
                         ? GtfsFilterDateWithServiceDays(gtfs, options.date)
                         : GtfsFilterByDate(gtfs, options.date);
  log("Computing test data...");
  CachedTestData data = MakeCachedTestData(std::move(gtfs_day));
  log("Done computing test data.");

  cache[cache_key] = data;
  return data;
}

}  // namespace vats5
