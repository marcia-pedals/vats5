#include <CLI/CLI.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <toml++/toml.hpp>
#include <unordered_set>

#include "gtfs/gtfs_filter.h"
#include "solver/data.h"
#include "solver/tarel_graph.h"
#include "visualization/visualization.h"

using namespace vats5;

int main(int argc, char* argv[]) {
  CLI::App app{"Initialize problem state from GTFS data"};

  std::string world_config_path;
  std::string required_stops_config;
  std::string output_path;
  double max_walking_distance = 500.0;
  double walking_speed = 1.0;

  app.add_option(
         "world_config_path",
         world_config_path,
         "Path to world TOML config file"
  )
      ->required();
  app.add_option(
         "required_stops_config",
         required_stops_config,
         "Path to required stops TOML config file"
  )
      ->required();
  app.add_option("output_path", output_path, "Output JSON file path")
      ->required();
  app.add_option(
         "--max-walking-distance",
         max_walking_distance,
         "Maximum walking distance in meters"
  )
      ->default_val(500.0);
  app.add_option("--walking-speed", walking_speed, "Walking speed in m/s")
      ->default_val(1.0);

  CLI11_PARSE(app, argc, argv);

  GetStepsOptions options{
      .max_walking_distance_meters = max_walking_distance,
      .walking_speed_ms = walking_speed,
  };

  GtfsFilterConfig filter_config = GtfsFilterConfigLoad(world_config_path);
  GtfsDay gtfs_day = GtfsNormalizeStops(GtfsFilterFromConfig(filter_config));

  toml::table required_stops_toml;
  try {
    required_stops_toml = toml::parse_file(required_stops_config);
  } catch (const toml::parse_error& err) {
    throw std::runtime_error(
        "Failed to parse required stops config file '" + required_stops_config +
        "': " + std::string(err.what())
    );
  }
  auto stop_ids_array = required_stops_toml["stop_ids"].as_array();
  if (!stop_ids_array) {
    throw std::runtime_error(
        "Required stops config must contain stop_ids array"
    );
  }

  std::cout << "Options: max_walking_distance=" << max_walking_distance
            << "m, walking_speed=" << walking_speed << "m/s\n";
  StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(gtfs_day, options);

  auto resolve_gtfs_stop_id = [&](const std::string& stop_id_str) -> StopId {
    GtfsStopId gtfs_stop_id{stop_id_str};
    auto it =
        steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.find(gtfs_stop_id);
    if (it == steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.end()) {
      throw std::runtime_error(
          "Stop ID '" + stop_id_str +
          "' from required stops config not found in GTFS data"
      );
    }
    return it->second;
  };

  std::unordered_set<StopId> required_stops;
  for (const auto& stop_id_elem : *stop_ids_array) {
    auto stop_id_str = stop_id_elem.value<std::string>();
    if (!stop_id_str) {
      throw std::runtime_error("Invalid stop_id in required stops config");
    }
    required_stops.insert(resolve_gtfs_stop_id(*stop_id_str));
  }

  // Parse optional stop_groups: array of arrays of GTFS stop IDs.
  // We store these as GtfsStopId strings because InitializeProblemState
  // compacts the StopIds, so we remap after initialization.
  std::vector<std::vector<std::string>> stop_groups;
  if (auto stop_groups_array = required_stops_toml["stop_groups"].as_array()) {
    std::unordered_set<StopId> seen_in_groups;
    for (size_t group_idx = 0; group_idx < stop_groups_array->size();
         ++group_idx) {
      auto group_elem = (*stop_groups_array)[group_idx].as_array();
      if (!group_elem) {
        throw std::runtime_error(
            "stop_groups[" + std::to_string(group_idx) +
            "] must be an array of stop IDs"
        );
      }
      std::vector<std::string> group;
      for (const auto& stop_id_elem : *group_elem) {
        auto stop_id_str = stop_id_elem.value<std::string>();
        if (!stop_id_str) {
          throw std::runtime_error(
              "Invalid stop_id in stop_groups[" + std::to_string(group_idx) +
              "]"
          );
        }
        StopId stop = resolve_gtfs_stop_id(*stop_id_str);
        if (!required_stops.contains(stop)) {
          throw std::runtime_error(
              "Stop ID '" + *stop_id_str + "' in stop_groups is not in stop_ids"
          );
        }
        if (!seen_in_groups.insert(stop).second) {
          throw std::runtime_error(
              "Stop ID '" + *stop_id_str + "' appears in multiple stop groups"
          );
        }
        group.push_back(*stop_id_str);
      }
      stop_groups.push_back(std::move(group));
    }
  }

  std::cout << "Initializing solution state...\n";
  auto init_result = InitializeProblemState(
      steps_from_gtfs, required_stops, /*optimize_edges=*/true
  );

  // Build stop_group_representative map using compact StopIds from the
  // problem state. First build a reverse lookup from GtfsStopId to compact
  // StopId.
  if (!stop_groups.empty()) {
    std::unordered_map<GtfsStopId, StopId> gtfs_to_compact;
    for (const auto& [stop_id, info] : init_result.problem_state.stop_infos) {
      gtfs_to_compact[info.gtfs_stop_id] = stop_id;
    }

    std::unordered_map<StopId, StopId> stop_group_representative;
    for (const auto& group : stop_groups) {
      StopId representative = gtfs_to_compact.at(GtfsStopId{group[0]});
      for (size_t i = 1; i < group.size(); ++i) {
        StopId member = gtfs_to_compact.at(GtfsStopId{group[i]});
        stop_group_representative[member] = representative;
      }
    }
    init_result.problem_state.stop_group_representative =
        std::move(stop_group_representative);
  }

  std::cout << "Serializing to JSON...\n";
  nlohmann::json j = init_result.problem_state;
  std::ofstream out(output_path);
  out << j.dump();
  out.close();
  std::cout << "Saved problem state to: " << output_path << "\n";

  // Create visualization SQLite database with required stops
  std::cout << "Creating visualization SQLite database...\n";

  std::string viz_output_path = viz::VizSqlitePath(output_path);

  viz::WriteVisualizationSqlite(
      init_result, gtfs_day, steps_from_gtfs.mapping, viz_output_path
  );
  std::cout << "Saved visualization to: " << viz_output_path << "\n";

  return 0;
}
