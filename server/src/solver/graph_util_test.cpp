#include "solver/graph_util.h"

#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/steps_shortest_path.h"

using ::testing::UnorderedElementsAre;

namespace vats5 {

TEST(GraphUtilTest, ComputeExtremeStops_BART) {
  GtfsDay gtfs_day = GtfsNormalizeStops(GtfsLoadDay("../data/RG_20250718_BA"));
  StepsFromGtfs steps_from_gtfs =
      GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);

  // Get all BART stops
  std::unordered_set<StopId> bart_stops =
      GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

  StepPathsAdjacencyList minimal =
      ReduceToMinimalSystemPaths(adjacency_list, bart_stops);
  StepPathsAdjacencyList complete = ReduceToMinimalSystemPaths(
      MakeAdjacencyList(minimal.AllMergedSteps()), bart_stops, true
  );

  // Compute extreme stops
  std::unordered_set<StopId> extreme_stops =
      ComputeExtremeStops(complete, bart_stops, StopId{-1});

  // Convert to names for easier assertion
  std::vector<std::string> extreme_stop_names;
  for (const StopId& stop_id : extreme_stops) {
    extreme_stop_names.push_back(
        steps_from_gtfs.mapping.stop_id_to_stop_name.at(stop_id)
    );
  }

  // The endpoints of the BART system plus OAK (Oakland Airport connector) and
  // SFO. SFO is on a spur but there are direct trains to Millbrae that don't
  // require getting off at SFO, so SFO is not an "inner" stop.
  EXPECT_THAT(
      extreme_stop_names,
      UnorderedElementsAre(
          "Richmond BART/Amtrak",
          "Millbrae BART",
          "Berryessa / North San Jose",
          "Antioch",
          "Dublin / Pleasanton BART",
          "OAK",
          "SFO"
      )
  );
}

}  // namespace vats5
