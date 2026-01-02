#include <algorithm>
#include <fstream>
#include <iostream>
#include <unordered_set>

#include "gtfs/gtfs.h"
#include "solver/data.h"
#include "solver/shortest_path.h"

using namespace vats5;

struct VisualizationStep {
  StopId origin_stop;
  StopId destination_stop;
  TimeSinceServiceStart origin_time;
  TimeSinceServiceStart destination_time;
  std::string trip_description;
  bool system_step;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VisualizationStep,
    origin_stop,
    destination_stop,
    origin_time,
    destination_time,
    trip_description,
    system_step
);

struct VisualizationPath {
  std::vector<VisualizationStep> steps;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VisualizationPath, steps);

struct VisualizationStop {
  GtfsStop gtfs_stop;
  bool system_stop;
  bool intermediate_stop;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VisualizationStop, gtfs_stop, system_stop, intermediate_stop
);

struct Visualization {
  // Key is actually a StopId but I don't know how to make the NLHOMANN macro
  // recognize that that is a number and can be serialized to an object key.
  // TODO: Figure out how to do that.
  std::unordered_map<int, VisualizationStop> stops;

  // Similar thing about the key.
  std::unordered_map<int, std::vector<std::vector<VisualizationPath>>> adjacent;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Visualization, stops, adjacent);

// Helper to convert between StopId set and int vector for JSON
std::vector<int> ToStopIdsJson(const std::unordered_set<StopId>& stops) {
  std::vector<int> result;
  for (const auto& stop_id : stops) {
    result.push_back(stop_id.v);
  }
  return result;
}

std::unordered_set<StopId> FromStopIdsJson(const std::vector<int>& json) {
  std::unordered_set<StopId> result;
  for (const int id : json) {
    result.insert(StopId{id});
  }
  return result;
}

// Combined state for serialization
struct VisualizationToolState {
  GtfsDay gtfs_day;
  StepsFromGtfs steps_from_gtfs;
  StepsAdjacencyList adjacency_list;
  std::vector<int> bart_stops_json;
  PathsAdjacencyList minimal;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VisualizationToolState,
    gtfs_day,
    steps_from_gtfs,
    adjacency_list,
    bart_stops_json,
    minimal
)

struct PairHash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);
    return h1 ^ (h2 << 1);
  }
};

struct StopGoodness {
  std::unordered_set<StopId> touch_stops;
  std::unordered_set<std::pair<StopId, StopId>, PairHash> touch_edges;

  int Goodness() const { return touch_edges.size() - touch_stops.size(); }

  std::unordered_set<StopId> partial_touch_stops;
  std::unordered_set<std::pair<StopId, StopId>, PairHash> partial_touch_edges;

  int PartialGoodness() const {
    return partial_touch_edges.size() - partial_touch_stops.size();
  }
};

std::vector<StopId> SelectIntermediateStop(
    const PathsAdjacencyList& split, const DataGtfsMapping& mapping
) {
  // The ultimate origins and destinations of stops in `split`. These are not
  // canidates for new intermediate stops.
  std::unordered_set<StopId> ultimate_stops;

  std::unordered_map<StopId, StopGoodness> goodness;

  for (const auto& [origin_stop, path_groups] : split.adjacent) {
    for (const auto& path_group : path_groups) {
      // A path_group is all the paths within a single (origin, destination)
      // edge.
      if (path_group.size() == 0) {
        continue;
      }
      const StopId destination_stop =
          path_group[0].merged_step.destination_stop;
      ultimate_stops.insert(origin_stop);
      ultimate_stops.insert(destination_stop);

      // Mapping from `StopId` to number of paths in this group that touch that
      // stop.
      std::unordered_map<StopId, int> paths_touched;
      for (const auto& path : path_group) {
        std::unordered_set<StopId> stops_in_path;
        for (const auto& step : path.steps) {
          if (!stops_in_path.contains(step.origin_stop)) {
            paths_touched[step.origin_stop] += 1;
            stops_in_path.insert(step.origin_stop);
          }
        }
      }

      for (const auto& [stop_id, count] : paths_touched) {
        StopGoodness& stop_goodness = goodness[stop_id];

        if (count == path_group.size()) {
          stop_goodness.touch_stops.insert(origin_stop);
          stop_goodness.touch_stops.insert(destination_stop);
          stop_goodness.touch_edges.insert(
              {std::min(origin_stop, destination_stop),
               std::max(origin_stop, destination_stop)}
          );
        }

        stop_goodness.partial_touch_stops.insert(origin_stop);
        stop_goodness.partial_touch_stops.insert(destination_stop);
        stop_goodness.partial_touch_edges.insert(
            {std::min(origin_stop, destination_stop),
             std::max(origin_stop, destination_stop)}
        );
      }
    }
  }

  for (const StopId stop : ultimate_stops) {
    goodness.erase(stop);
  }

  std::vector<std::pair<StopId, StopGoodness>> sorted_goodness(
      goodness.begin(), goodness.end()
  );
  std::sort(
      sorted_goodness.begin(),
      sorted_goodness.end(),
      [](const auto& a, const auto& b) {
        if (a.second.Goodness() == b.second.Goodness()) {
          return a.second.partial_touch_edges.size() >
                 b.second.partial_touch_edges.size();
        }
        return a.second.Goodness() > b.second.Goodness();
      }
  );

  std::cout << "\nTop 10 stops:\n";
  for (size_t i = 0; i < std::min(sorted_goodness.size(), size_t{10}); ++i) {
    const auto& [stop_id, stop_goodness] = sorted_goodness[i];
    const auto& stop_name = mapping.stop_id_to_stop_name.at(stop_id);
    const auto& gtfs_stop_id = mapping.stop_id_to_gtfs_stop_id.at(stop_id);
    std::cout << i + 1 << ". " << stop_name << " (" << gtfs_stop_id.v
              << "): " << stop_goodness.Goodness() << " "
              << stop_goodness.partial_touch_edges.size() << "\n";
  }
  std::cout << std::endl;

  std::vector<StopId> result;
  for (size_t i = 0; i < std::min(sorted_goodness.size(), size_t{10}); ++i) {
    result.push_back(sorted_goodness[i].first);
  }
  return result;
}

bool IsBartTrip(const DataGtfsMapping& mapping, TripId trip_id) {
  auto it = mapping.trip_id_to_trip_info.find(trip_id);
  if (it == mapping.trip_id_to_trip_info.end()) {
    return false;
  }
  if (!std::holds_alternative<GtfsTripId>(it->second.v)) {
    return false;
  }
  const std::string& gtfs_trip_id = std::get<GtfsTripId>(it->second.v).v;
  return gtfs_trip_id.starts_with("BA:");
}

void SaveVisualization(
    const GtfsDay& gtfs_day,
    const StepsFromGtfs& steps_from_gtfs,
    const std::unordered_set<StopId>& bart_stops,
    const std::unordered_set<StopId>& intermediate_stops,
    const PathsAdjacencyList& split
) {
  Visualization viz;
  for (const GtfsStop& stop : gtfs_day.stops) {
    const StopId stop_id =
        steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(stop.stop_id);
    viz.stops[stop_id.v] = VisualizationStop{
        .gtfs_stop = stop,
        .system_stop = bart_stops.contains(stop_id),
        .intermediate_stop = intermediate_stops.contains(stop_id) &&
                             !bart_stops.contains(stop_id),
    };
  }

  for (const auto& [stop_id, path_groups] : split.adjacent) {
    std::vector<std::vector<VisualizationPath>> viz_path_groups;
    for (const auto& path_group : path_groups) {
      std::vector<VisualizationPath> viz_paths;
      for (const auto& path : path_group) {
        VisualizationPath viz_path;
        for (const auto& step : path.steps) {
          viz_path.steps.push_back(VisualizationStep{
              .origin_stop = step.origin_stop,
              .destination_stop = step.destination_stop,
              .origin_time = step.origin_time,
              .destination_time = step.destination_time,
              .trip_description =
                  steps_from_gtfs.mapping.GetRouteDescFromTrip(step.origin_trip
                  ),
              .system_step =
                  IsBartTrip(steps_from_gtfs.mapping, step.origin_trip) &&
                  IsBartTrip(steps_from_gtfs.mapping, step.destination_trip),
          });
        }
        viz_paths.push_back(viz_path);
      }
      viz_path_groups.push_back(viz_paths);
    }
    viz.adjacent[stop_id.v] = viz_path_groups;
  }

  nlohmann::json j = viz;
  std::ofstream out("../data/visualization.json");
  out << j.dump(2) << std::endl;
  std::cout << "Wrote visualization to ../data/visualization.json" << std::endl;
  // std::cout << "Intermediate stops were (" << intermediate_stops.size() <<
  // "): "; for (const auto s : intermediate_stops) {
  //   std::cout << s.v << " ";
  // }
  // std::cout << "\n";
}

int main(int argc, char* argv[]) {
  const std::string gtfs_path = "../data/RG_20260109_BA_CT_SC_SM_AC";
  std::string load_state_path;

  // Parse command line arguments
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--load-state" && i + 1 < argc) {
      load_state_path = argv[++i];
    }
  }

  GtfsDay gtfs_day;
  StepsFromGtfs steps_from_gtfs;
  StepsAdjacencyList adjacency_list;
  std::unordered_set<StopId> bart_stops;
  PathsAdjacencyList minimal;

  if (!load_state_path.empty()) {
    std::cout << "Loading state from: " << load_state_path << std::endl;
    std::ifstream in(load_state_path);
    nlohmann::json j;
    in >> j;
    VisualizationToolState state = j.get<VisualizationToolState>();
    gtfs_day = std::move(state.gtfs_day);
    steps_from_gtfs = std::move(state.steps_from_gtfs);
    adjacency_list = std::move(state.adjacency_list);
    bart_stops = FromStopIdsJson(state.bart_stops_json);
    minimal = std::move(state.minimal);
  } else {
    std::cout << "Loading GTFS data from: " << gtfs_path << std::endl;
    gtfs_day = GtfsLoadDay(gtfs_path);

    std::cout << "Normalizing stops..." << std::endl;
    gtfs_day = GtfsNormalizeStops(gtfs_day);

    std::cout << "Getting steps..." << std::endl;
    steps_from_gtfs = GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});

    std::cout << "Making adjacency list..." << std::endl;
    adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);

    bart_stops =
        GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

    std::cout << "Reducing to minimal system steps..." << std::endl;
    minimal = ReduceToMinimalSystemPaths(adjacency_list, bart_stops);

    std::cout << "Saving state to ../data/visualization_state.json..."
              << std::endl;
    VisualizationToolState state{
        gtfs_day,
        steps_from_gtfs,
        adjacency_list,
        ToStopIdsJson(bart_stops),
        minimal
    };
    nlohmann::json state_j = state;
    std::ofstream state_out("../data/visualization_state.json");
    state_out << state_j.dump(2) << std::endl;
  }

  double snap_threshold_meters = 150;
  PathsAdjacencyList split = minimal;
  std::unordered_set<StopId> intermediate_stops = bart_stops;

  intermediate_stops.insert(steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
      GtfsStopId{"mtc:palo-alto-station"}
  ));
  intermediate_stops.insert(steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
      GtfsStopId{"mtc:san-jose-diridon-station"}
  ));
  intermediate_stops.insert(steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
      GtfsStopId{"mtc:santa-clara-caltrain"}
  ));
  intermediate_stops.insert(steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
      GtfsStopId{"mtc:mountain-view-station"}
  ));
  intermediate_stops.insert(steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
      GtfsStopId{"sunnyvale"}
  ));

  for (int i = 0; i < 50; ++i) {
    split = SplitPathsAt(split, intermediate_stops);
    split = AdjacencyListSnapToStops(
        steps_from_gtfs.mapping, snap_threshold_meters, split
    );
    split = SplitPathsAt(split, intermediate_stops);

    int edge_count = 0;
    for (const auto& [origin_stop, path_groups] : split.adjacent) {
      edge_count += path_groups.size();
    }
    std::cout << "Current edge count: " << edge_count << "\n";

    SaveVisualization(
        gtfs_day, steps_from_gtfs, bart_stops, intermediate_stops, split
    );

    auto candidate_intermediate_stops =
        SelectIntermediateStop(split, steps_from_gtfs.mapping);
    if (candidate_intermediate_stops.empty()) {
      break;
    }

    bool found_improvement = false;
    for (int j = 0; j < candidate_intermediate_stops.size(); ++j) {
      const StopId stop = candidate_intermediate_stops[j];
      std::unordered_set<StopId> new_intermediate_stops = intermediate_stops;
      new_intermediate_stops.insert(stop);

      PathsAdjacencyList new_split = split;
      new_split = SplitPathsAt(new_split, new_intermediate_stops);
      new_split = AdjacencyListSnapToStops(
          steps_from_gtfs.mapping, snap_threshold_meters, new_split
      );
      new_split = SplitPathsAt(new_split, new_intermediate_stops);

      int new_edge_count = 0;
      for (const auto& [origin_stop, path_groups] : new_split.adjacent) {
        new_edge_count += path_groups.size();
      }

      if (new_edge_count <= edge_count) {
        std::cout << "Selected " << j + 1 << " reducing from " << edge_count
                  << " to " << new_edge_count << "\n";
        intermediate_stops.insert(stop);
        found_improvement = true;
        break;
      }
    }

    if (!found_improvement) {
      std::cout << "No improvement found, stopping.\n";
      break;
      // intermediate_stops.insert(candidate_intermediate_stops[0]);
    }
  }

  return 0;
}

// Some older sets of intermediate stops:
// {"mtc:san-jose-diridon-station",
//         "mtc:mountain-view-station",
//         "mtc:palo-alto-station",
//         "mtc:salesforce-transit-center",
//         "mtc:santa-clara-caltrain",
//         "331100", // "SF Transit Center - BART shuttle connection"
//         "55777", // Uptown Transit Center
//         "52581", // 14th Street NB
//         "51111", // Broadway & 12th St (12th St BART)
//         "52050", //"7th St & Union St"
//         "50454", // 14th St & Martin Luther King Jr Way
//         "58808", // 159th Av & E 14th St
//         "56665", // Telegraph Av & 40th St
//         "59755", // San Pablo Av & Delaware St
//         "53335", // Broadway & 17th St
//        }

// {
//     "mtc:san-jose-diridon-station",
//     "331544", // Potrero Ave & 16th St
//     "58808", // 159th Av & E 14th St
//     "50454", // 14th St & Martin Luther King Jr Way
//     "50896", // 7th St & Mandela Pkwy (West Oakland BART)
//     "mtc:salesforce-transit-center", // Salesforce Transit Center
//     "52584", // City Center NB
//     "50958", // Broadway & 17th St (19th St BART)
//     "335620", // SFO Airport Terminal G-Lower Level
//     "331553", // Bayshore Blvd & Augusta Ave
//     "53335", // Broadway & 17th St
//     "52573", // Madison SB
//     "55562", // San Pablo Av & Carlson Blvd
//     "55555", // Shattuck Av & Allston Way
//     "53003", // San Pablo Av & Grayson St
//     "56665", // Telegraph Av & 40th St
//     "55335", // San Pablo Av & Stanford Av
//     "52494", // 67th Avenue
//     "335637", // SFO Airport Terminal A-Lower Level
//     "52050", // 7th St & Union St
//     "334087", // El Camino Real & McLellan Dr-South SF BART
//     "334079", // El Camino Real & BART-South SF
//     "331100", // SF Transit Center - BART shuttle connection
//     "52643", // 11th St & Jackson St
//     "55532", // Hayward BART
//     "mtc:palo-alto-station",
//     "51800", // 7th St & Adeline St
//     "58850", // 7th St & Market St
//   }
