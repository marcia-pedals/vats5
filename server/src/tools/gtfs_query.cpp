#include <CLI/CLI.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
  std::string trip_id_prefix;
  std::string route_ids;

  app.add_option("config_path", config_path, "Path to TOML config file")
      ->required();
  app.add_option("command", command, "Query command (stops, routes)")
      ->required();
  app.add_option("--trip_id_prefix", trip_id_prefix,
                 "Comma-separated list of trip ID prefixes to filter by");
  app.add_option("--route_ids", route_ids,
                 "Comma-separated list of exact route IDs to include");

  CLI11_PARSE(app, argc, argv);

  try {
    GtfsFilterConfig config = GtfsFilterConfigLoad(config_path);
    GtfsDay data = GtfsNormalizeStops(GtfsFilterFromConfig(config));

    if (!trip_id_prefix.empty()) {
      std::vector<std::string> prefixes;
      std::istringstream ss(trip_id_prefix);
      std::string prefix;
      while (std::getline(ss, prefix, ',')) {
        prefixes.push_back(prefix);
      }

      std::unordered_set<GtfsTripId> matching_trips;
      for (const auto& trip : data.trips) {
        for (const auto& p : prefixes) {
          if (trip.trip_id.v.substr(0, p.length()) == p) {
            matching_trips.insert(trip.trip_id);
            break;
          }
        }
      }
      data = GtfsDayFilterByTrips(data, matching_trips);
      RemoveUnreferencedTripsRoutesAndDirections(data);
    }

    if (!route_ids.empty()) {
      std::unordered_set<GtfsRouteId> route_id_set;
      std::istringstream ss(route_ids);
      std::string id;
      while (std::getline(ss, id, ',')) {
        route_id_set.insert(GtfsRouteId{id});
      }

      std::unordered_set<GtfsTripId> matching_trips;
      for (const auto& trip : data.trips) {
        if (route_id_set.count(trip.route_direction_id.route_id)) {
          matching_trips.insert(trip.trip_id);
        }
      }
      data = GtfsDayFilterByTrips(data, matching_trips);
      RemoveUnreferencedTripsRoutesAndDirections(data);
    }

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
