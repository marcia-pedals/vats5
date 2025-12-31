#include <iostream>
#include <unordered_set>

#include "gtfs/gtfs.h"
#include "solver/data.h"
#include "solver/shortest_path.h"

using namespace vats5;

struct Visualization {
  // Key is actually a StopId but I don't know how to make the NLHOMANN macro recognize that
  // that is a number and can be serialized to an object key.
  // TODO: Figure out how to do that.
  std::unordered_map<int, GtfsStop> stops;

  // Similar thing about the key.
  std::unordered_map<int, std::vector<std::vector<Step>>> adjacent;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Visualization, stops, adjacent);

int main() {
  std::string gtfs_path = "../data/RG_20260109_BA_CT_SC_SM_AC";

  std::cout << "Loading GTFS data from: " << gtfs_path << std::endl;
  GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);

  std::cout << "Normalizing stops..." << std::endl;
  gtfs_day = GtfsNormalizeStops(gtfs_day);

  std::cout << "Getting steps..." << std::endl;
  StepsFromGtfs steps_from_gtfs =
      GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});

  std::cout << "Making adjacency list..." << std::endl;
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);

  std::unordered_set<StopId> bart_stops =
      GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

  std::cout << "Reducing to minimal system steps..." << std::endl;
  StepsAdjacencyList reduced =
      ReduceToMinimalSystemSteps(adjacency_list, bart_stops);

  std::cout << "BART stops count: " << bart_stops.size() << std::endl;
  std::cout << "Reduced adjacency list size: " << reduced.adjacent.size()
            << std::endl;

  Visualization viz;
  for (const GtfsStop& stop : gtfs_day.stops) {
    const StopId stop_id = steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(stop.stop_id);
    if (bart_stops.contains(stop_id)) {
      viz.stops[stop_id.v] = stop;
    }
  }

  for (const auto& [stop_id, step_groups] : reduced.adjacent) {
    viz.adjacent[stop_id.v] = step_groups;
  }

  nlohmann::json j = viz;
  std::cout << j.dump(2) << std::endl;

  return 0;
}
