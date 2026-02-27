#include <CLI/CLI.hpp>
#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gtfs/gtfs.h"
#include "gtfs/gtfs_filter.h"
#include "log.h"

using namespace vats5;

// Shell-escape an argument for safe inclusion in shell commands
std::string ShellEscape(const std::string& arg) {
  // Check if the argument needs quoting
  bool needs_quoting = false;
  for (char c : arg) {
    if (!std::isalnum(c) && c != '/' && c != '.' && c != '_' && c != '-' &&
        c != ':') {
      needs_quoting = true;
      break;
    }
  }

  if (!needs_quoting && !arg.empty()) {
    return arg;
  }

  // Use single quotes and escape any single quotes in the argument
  std::string escaped = "'";
  for (char c : arg) {
    if (c == '\'') {
      escaped += "'\\''";  // End quote, escaped quote, start quote
    } else {
      escaped += c;
    }
  }
  escaped += "'";
  return escaped;
}

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

void QueryRequiredStopsConfig(
    const GtfsDay& data,
    int argc,
    char* argv[],
    const std::unordered_set<GtfsStopId>& exclude_stop_ids = {}
) {
  // Build a map of stop_id to stop_name
  std::unordered_map<GtfsStopId, std::string> stop_names;
  for (const auto& stop : data.stops) {
    stop_names[stop.stop_id] = stop.stop_name;
  }

  // Build a map of route_id to route_short_name
  std::unordered_map<GtfsRouteId, std::string> route_short_names;
  for (const auto& route : data.routes) {
    route_short_names[route.route_id] = route.route_short_name;
  }

  // Build a map of trip_id to route_direction_id
  std::unordered_map<GtfsTripId, GtfsRouteDirectionId> trip_to_route_dir;
  for (const auto& trip : data.trips) {
    trip_to_route_dir[trip.trip_id] = trip.route_direction_id;
  }

  // Build a map of route_direction_id to direction name
  std::unordered_map<GtfsRouteDirectionId, std::string> route_dir_to_name;
  for (const auto& dir : data.directions) {
    route_dir_to_name[dir.route_direction_id] = dir.direction;
  }

  // Build a map of all directions for each route (globally)
  // Map: route_short_name -> set of all directions for that route
  std::map<std::string, std::set<std::string>> route_all_directions;
  for (const auto& trip : data.trips) {
    auto route_name_it =
        route_short_names.find(trip.route_direction_id.route_id);
    auto dir_it = route_dir_to_name.find(trip.route_direction_id);
    if (route_name_it != route_short_names.end() &&
        dir_it != route_dir_to_name.end()) {
      route_all_directions[route_name_it->second].insert(dir_it->second);
    }
  }

  // Collect all unique stop IDs and map them to routes with their directions
  std::unordered_set<GtfsStopId> stop_ids;
  // Map: stop_id -> (route_short_name -> set of directions)
  std::unordered_map<GtfsStopId, std::map<std::string, std::set<std::string>>>
      stop_to_route_directions;
  for (const auto& stop_time : data.stop_times) {
    stop_ids.insert(stop_time.stop_id);
    auto trip_it = trip_to_route_dir.find(stop_time.trip_id);
    if (trip_it != trip_to_route_dir.end()) {
      auto dir_it = route_dir_to_name.find(trip_it->second);
      auto route_name_it = route_short_names.find(trip_it->second.route_id);
      if (dir_it != route_dir_to_name.end() &&
          route_name_it != route_short_names.end()) {
        stop_to_route_directions[stop_time.stop_id][route_name_it->second]
            .insert(dir_it->second);
      }
    }
  }

  // Helper function to generate the full comment for a stop
  auto generate_comment = [&](const GtfsStopId& stop_id) -> std::string {
    std::stringstream ss;
    ss << stop_names[stop_id];
    auto route_dir_it = stop_to_route_directions.find(stop_id);
    if (route_dir_it != stop_to_route_directions.end() &&
        !route_dir_it->second.empty()) {
      ss << ":";
      bool first_route = true;
      for (const auto& [route_name, directions] : route_dir_it->second) {
        if (!first_route) ss << ",";
        ss << " " << route_name;
        auto all_dirs_it = route_all_directions.find(route_name);
        if (all_dirs_it != route_all_directions.end() &&
            directions != all_dirs_it->second) {
          ss << " [";
          bool first_dir = true;
          for (const auto& dir : directions) {
            if (!first_dir) ss << ", ";
            ss << dir;
            first_dir = false;
          }
          ss << "]";
        }
        first_route = false;
      }
    }
    return ss.str();
  };

  // Convert to sorted vector, sorted by full comment, excluding any excluded
  // stops
  std::vector<GtfsStopId> sorted_stops;
  for (const auto& stop_id : stop_ids) {
    if (exclude_stop_ids.find(stop_id) == exclude_stop_ids.end()) {
      sorted_stops.push_back(stop_id);
    }
  }
  std::sort(
      sorted_stops.begin(),
      sorted_stops.end(),
      [&generate_comment](const GtfsStopId& a, const GtfsStopId& b) {
        return generate_comment(a) < generate_comment(b);
      }
  );

  // Output the TOML file with original command
  std::cout << "# Generated by: gtfs_query";
  for (int i = 1; i < argc; ++i) {
    std::cout << " " << ShellEscape(argv[i]);
  }
  std::cout << "\n# Number of stops: " << sorted_stops.size();
  std::cout << "\n\nstop_ids = [\n";

  for (const auto& stop_id : sorted_stops) {
    std::cout << "  \"" << stop_id.v << "\", # " << generate_comment(stop_id)
              << "\n";
  }

  std::cout << "]\n";
}

int main(int argc, char* argv[]) {
  CLI::App app{"Query filtered GTFS data"};

  std::string config_path;
  std::string command;
  std::string trip_id_prefix;
  std::string route_ids;
  std::string exclude_stop_ids;

  app.add_option("config_path", config_path, "Path to TOML config file")
      ->required();
  app.add_option(
         "command",
         command,
         "Query command (stops, routes, required_stops_config)"
  )
      ->required();
  app.add_option(
      "--trip_id_prefix",
      trip_id_prefix,
      "Comma-separated list of trip ID prefixes to filter by"
  );
  app.add_option(
      "--route_ids",
      route_ids,
      "Comma-separated list of exact route IDs to include"
  );
  app.add_option(
      "--exclude_stop_ids",
      exclude_stop_ids,
      "Comma-separated list of stop IDs to exclude from output"
  );

  CLI11_PARSE(app, argc, argv);

  try {
    GtfsFilterConfig config = GtfsFilterConfigLoad(config_path);
    GtfsDay data = GtfsNormalizeStops(
        GtfsFilterFromConfig(config, OstreamLogger(std::cerr))
    );

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

    // Parse exclude_stop_ids if provided
    std::unordered_set<GtfsStopId> exclude_stop_id_set;
    if (!exclude_stop_ids.empty()) {
      std::istringstream ss(exclude_stop_ids);
      std::string id;
      while (std::getline(ss, id, ',')) {
        exclude_stop_id_set.insert(GtfsStopId{id});
      }
    }

    if (command == "stops") {
      QueryStops(data);
    } else if (command == "routes") {
      QueryRoutes(data);
    } else if (command == "required_stops_config") {
      QueryRequiredStopsConfig(data, argc, argv, exclude_stop_id_set);
    } else {
      std::cerr << "Unknown command: " << command << std::endl;
      std::cerr << "Available commands: stops, routes, required_stops_config"
                << std::endl;
      return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
