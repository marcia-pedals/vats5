#include "solver/shortest_path.h"

#include <gtest/gtest.h>

#include <iostream>

#include "gtfs/gtfs.h"
#include "solver/data.h"

namespace vats5 {

TEST(ShortestPathTest, MakeAdjacencyListBasic) {
  std::vector<Step> steps = {
      Step{.origin_stop = StopId{1},
           .destination_stop = StopId{2},
           .origin_time = TimeSinceServiceStart{100},
           .destination_time = TimeSinceServiceStart{200},
           .origin_trip = TripId{1},
           .destination_trip = TripId{1}},
      Step{.origin_stop = StopId{1},
           .destination_stop = StopId{2},
           .origin_time = TimeSinceServiceStart{150},
           .destination_time = TimeSinceServiceStart{250},
           .origin_trip = TripId{2},
           .destination_trip = TripId{2}}};

  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps);

  EXPECT_EQ(adjacency_list.adjacent.size(), 1);
  EXPECT_NE(adjacency_list.adjacent.find(StopId{1}),
            adjacency_list.adjacent.end());
  EXPECT_EQ(adjacency_list.adjacent[StopId{1}].size(), 1);
  EXPECT_EQ(adjacency_list.adjacent[StopId{1}][0].size(), 2);
}

namespace {

struct ProcessedGtfsData {
  StepsFromGtfs steps_from_gtfs;
  StepsAdjacencyList adjacency_list;
};

ProcessedGtfsData LoadAndProcessGtfsData() {
  std::cout << "Loading GTFS data from ../data/RG_20250718_BA..." << std::endl;
  GtfsDay gtfs_day = GtfsLoadDay("../data/RG_20250718_BA");
  std::cout << "Loaded " << gtfs_day.stop_times.size() << " stop times"
            << std::endl;

  std::cout << "Normalizing stops..." << std::endl;
  gtfs_day = GtfsNormalizeStops(gtfs_day);
  std::cout << "After normalization: " << gtfs_day.stop_times.size()
            << " stop times" << std::endl;

  std::cout << "Converting GTFS data to steps..." << std::endl;
  StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(gtfs_day);
  std::cout << "Generated " << steps_from_gtfs.steps.size() << " steps"
            << std::endl;

  std::cout << "Creating adjacency list..." << std::endl;
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);
  std::cout << "Adjacency list has " << adjacency_list.adjacent.size()
            << " origin stops" << std::endl;

  return {std::move(steps_from_gtfs), std::move(adjacency_list)};
}

StopId FindStopId(const StepsFromGtfs& steps_from_gtfs,
                  const GtfsStopId& gtfs_id, const std::string& stop_name) {
  auto it = steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.find(gtfs_id);
  if (it != steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.end()) {
    std::cout << "Found " << stop_name << " stop: GTFS ID " << gtfs_id.v
              << " -> Internal ID " << it->second.v << std::endl;
    return it->second;
  } else {
    std::cout << stop_name << " stop " << gtfs_id.v << " not found in mapping"
              << std::endl;
    EXPECT_TRUE(false) << stop_name << " stop not found";
    return StopId{0};
  }
}

struct TestStops {
  StopId berryessa;
  StopId powell;
  StopId dublin;
  StopId bayfair;
  StopId antioch;
  StopId millbrae;
};

TestStops FindTestStops(const StepsFromGtfs& steps_from_gtfs) {
  TestStops stops;
  stops.berryessa =
      FindStopId(steps_from_gtfs, GtfsStopId{"909509"}, "Berryessa");
  stops.powell =
      FindStopId(steps_from_gtfs, GtfsStopId{"mtc:powell"}, "Powell");
  stops.dublin =
      FindStopId(steps_from_gtfs, GtfsStopId{"mtc:dublin-pleasanton-bart"},
                 "Dublin/Pleasanton");
  stops.bayfair = FindStopId(steps_from_gtfs, GtfsStopId{"902509"}, "Bay Fair");
  stops.antioch = FindStopId(steps_from_gtfs, GtfsStopId{"908309"}, "Antioch");
  stops.millbrae =
      FindStopId(steps_from_gtfs, GtfsStopId{"mtc:millbrae-bart"}, "Millbrae");
  return stops;
}

void VerifyPathResult(const std::unordered_map<StopId, Step>& shortest_paths,
                      const StepsFromGtfs& steps_from_gtfs,
                      StopId destination_stop,
                      const std::string& destination_name,
                      int expected_origin_time, int expected_destination_time) {
  auto path_it = shortest_paths.find(destination_stop);
  ASSERT_NE(path_it, shortest_paths.end())
      << destination_name << " path not found";

  const Step& step = path_it->second;
  auto gtfs_destination_id =
      steps_from_gtfs.mapping.stop_id_to_gtfs_stop_id.at(destination_stop);

  std::cout << "\nPath to " << gtfs_destination_id.v << " (internal ID "
            << destination_stop.v << "):" << std::endl;
  std::cout << "  " << step << std::endl;
  std::cout << "  Travel time: "
            << (step.destination_time.seconds - step.origin_time.seconds)
            << " seconds" << std::endl;

  EXPECT_EQ(step.origin_time.seconds, expected_origin_time)
      << destination_name << " departure time";
  EXPECT_EQ(step.destination_time.seconds, expected_destination_time)
      << destination_name << " arrival time";
}

void VerifyAllPaths(const std::unordered_map<StopId, Step>& shortest_paths,
                    const StepsFromGtfs& steps_from_gtfs,
                    const TestStops& stops) {
  std::cout << "\nResults:" << std::endl;
  std::cout << "Found " << shortest_paths.size() << " paths" << std::endl;

  EXPECT_EQ(shortest_paths.size(), 5);

  VerifyPathResult(shortest_paths, steps_from_gtfs, stops.powell, "Powell",
                   TimeSinceServiceStart::Parse("08:05:00").seconds,
                   TimeSinceServiceStart::Parse("09:11:00").seconds);
  VerifyPathResult(shortest_paths, steps_from_gtfs, stops.dublin, "Dublin",
                   TimeSinceServiceStart::Parse("08:05:00").seconds,
                   TimeSinceServiceStart::Parse("09:08:00").seconds);
  VerifyPathResult(shortest_paths, steps_from_gtfs, stops.bayfair, "Bay Fair",
                   TimeSinceServiceStart::Parse("08:05:00").seconds,
                   TimeSinceServiceStart::Parse("08:40:00").seconds);
  VerifyPathResult(shortest_paths, steps_from_gtfs, stops.antioch, "Antioch",
                   TimeSinceServiceStart::Parse("08:05:00").seconds,
                   TimeSinceServiceStart::Parse("10:14:00").seconds);
  VerifyPathResult(shortest_paths, steps_from_gtfs, stops.millbrae, "Millbrae",
                   TimeSinceServiceStart::Parse("08:05:00").seconds,
                   TimeSinceServiceStart::Parse("10:07:00").seconds);
}

}  // namespace

TEST(ShortestPathTest, FindShortestPathsAtTimeWithRealData) {
  try {
    auto processed_data = LoadAndProcessGtfsData();
    TestStops stops = FindTestStops(processed_data.steps_from_gtfs);

    TimeSinceServiceStart query_time = TimeSinceServiceStart::Parse("08:00:00");
    std::unordered_set<StopId> destinations = {stops.powell, stops.dublin,
                                               stops.bayfair, stops.antioch,
                                               stops.millbrae};

    std::cout << "\nQuerying shortest paths from Berryessa (stop "
              << stops.berryessa.v << ") at 8:00 AM..." << std::endl;

    auto shortest_paths =
        FindShortestPathsAtTime(processed_data.adjacency_list, query_time,
                                stops.berryessa, destinations);

    VerifyAllPaths(shortest_paths, processed_data.steps_from_gtfs, stops);

  } catch (const std::exception& e) {
    FAIL() << "Exception: " << e.what();
  }
}

}  // namespace vats5