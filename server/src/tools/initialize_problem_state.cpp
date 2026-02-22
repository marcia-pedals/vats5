#include <CLI/CLI.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <toml++/toml.hpp>
#include <unordered_set>

#include "gtfs/gtfs_filter.h"
#include "solver/data.h"
#include "solver/graph_util.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"
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

  std::unordered_set<StopId> required_stops;
  for (const auto& stop_id_elem : *stop_ids_array) {
    auto stop_id_str = stop_id_elem.value<std::string>();
    if (!stop_id_str) {
      throw std::runtime_error("Invalid stop_id in required stops config");
    }
    GtfsStopId gtfs_stop_id{*stop_id_str};
    auto it = steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.find(gtfs_stop_id);
    if (it == steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.end()) {
      throw std::runtime_error(
          "Stop ID '" + *stop_id_str + "' from required stops config not found in GTFS data"
      );
    }
    required_stops.insert(it->second);
  }

  std::cout << "Initializing solution state...\n";
  ProblemState state = InitializeProblemState(
      steps_from_gtfs, required_stops, /*optimize_edges=*/true
  );

  auto extreme_stops = ComputeExtremeStops(
      state.completed, state.required_stops, state.boundary.start
  );
  std::cout << "Final extreme stop count: " << extreme_stops.size() << "\n";
  state = MakeProblemState(
      MakeAdjacencyList(ReduceToMinimalSystemPaths(state.minimal, extreme_stops)
                            .AllMergedSteps()),
      state.boundary,
      extreme_stops,
      state.stop_infos,
      state.step_partition_names,
      state.original_edges
  );

  std::cout << "Serializing to JSON...\n";
  nlohmann::json j = state;
  std::ofstream out(output_path);
  out << j.dump();
  out.close();
  std::cout << "Saved problem state to: " << output_path << "\n";

  // Create visualization JSON with required stops
  std::cout << "Creating visualization JSON...\n";
  viz::Visualization visualization = viz::MakeVisualization(state, gtfs_day);
  nlohmann::json viz_json = visualization;

  // Determine visualization output path (e.g., "problem.json" -> "problem-viz.json")
  std::string viz_output_path = output_path;
  size_t dot_pos = viz_output_path.rfind('.');
  if (dot_pos != std::string::npos) {
    viz_output_path = viz_output_path.substr(0, dot_pos) + "-viz" + viz_output_path.substr(dot_pos);
  } else {
    viz_output_path += "-viz";
  }

  std::ofstream viz_out(viz_output_path);
  viz_out << viz_json.dump(2);  // Pretty print with indent
  viz_out.close();
  std::cout << "Saved visualization to: " << viz_output_path << "\n";

  return 0;
}
