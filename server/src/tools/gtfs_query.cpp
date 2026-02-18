#include <CLI/CLI.hpp>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtfs/gtfs.h"
#include "gtfs/gtfs_filter.h"

using namespace vats5;

void QueryStops(const GtfsDay& data) {
  for (const auto& stop : data.stops) {
    std::cout << stop.stop_id.v << "\t" << stop.stop_name << std::endl;
  }
}

void QueryRoutes(const GtfsDay& data) {
  std::unordered_map<GtfsRouteId, std::vector<const GtfsDirection*>> dir_map;
  for (const auto& dir : data.directions) {
    dir_map[dir.route_direction_id.route_id].push_back(&dir);
  }

  for (const auto& route : data.routes) {
    std::cout << route.route_id.v << "\t" << route.route_short_name << "\t"
              << route.route_long_name << std::endl;
    auto it = dir_map.find(route.route_id);
    if (it != dir_map.end()) {
      for (const auto* dir : it->second) {
        std::cout << "  direction " << dir->route_direction_id.direction_id
                  << ": " << dir->direction << std::endl;
      }
    }
  }
}

int main(int argc, char* argv[]) {
  CLI::App app{"Query filtered GTFS data"};

  std::string config_path;
  std::string command;

  app.add_option("config_path", config_path, "Path to TOML config file")
      ->required();
  app.add_option("command", command, "Query command (stops, routes)")
      ->required();

  CLI11_PARSE(app, argc, argv);

  try {
    GtfsFilterConfig config = GtfsFilterConfigLoad(config_path);
    GtfsDay data = GtfsFilterFromConfig(config);

    if (command == "stops") {
      QueryStops(data);
    } else if (command == "routes") {
      QueryRoutes(data);
    } else {
      std::cerr << "Unknown command: " << command << std::endl;
      std::cerr << "Available commands: stops, routes" << std::endl;
      return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
