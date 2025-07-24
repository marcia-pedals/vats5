#include "solver/shortest_path.h"

#include <gtest/gtest.h>

#include <iostream>

#include "gtfs/gtfs.h"
#include "solver/data.h"

namespace vats5 {

TEST(ShortestPathTest, MakeAdjacencyListBasic) {
  std::vector<Step> steps = {
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{100},
          .destination_time = TimeSinceServiceStart{200},
          .origin_trip = TripId{1},
          .destination_trip = TripId{1}
      },
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{150},
          .destination_time = TimeSinceServiceStart{250},
          .origin_trip = TripId{2},
          .destination_trip = TripId{2}
      }
  };

  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps);

  EXPECT_EQ(adjacency_list.adjacent.size(), 1);
  EXPECT_NE(
      adjacency_list.adjacent.find(StopId{1}), adjacency_list.adjacent.end()
  );
  EXPECT_EQ(adjacency_list.adjacent[StopId{1}].size(), 1);
  EXPECT_EQ(adjacency_list.adjacent[StopId{1}][0].size(), 2);
}

namespace {

void VerifyPathResult(
    const std::unordered_map<StopId, Step>& shortest_paths,
    const StepsFromGtfs& steps_from_gtfs,
    const std::string& destination_stop_name,
    const std::string& expected_origin_time_str,
    const std::string& expected_destination_time_str,
    const std::string& expected_origin_route_desc,
    const std::string& expected_destination_route_desc
) {
  StopId destination_stop =
      steps_from_gtfs.mapping.GetStopIdFromName(destination_stop_name);

  auto path_it = shortest_paths.find(destination_stop);
  ASSERT_NE(path_it, shortest_paths.end())
      << destination_stop_name << " path not found";

  const Step& step = path_it->second;

  EXPECT_EQ(step.origin_time.ToString(), expected_origin_time_str)
      << destination_stop_name << " departure time";
  EXPECT_EQ(step.destination_time.ToString(), expected_destination_time_str)
      << destination_stop_name << " arrival time";

  if (!expected_origin_route_desc.empty()) {
    std::string actual_origin_route_desc =
        steps_from_gtfs.mapping.GetRouteDescFromTrip(step.origin_trip);
    EXPECT_EQ(actual_origin_route_desc, expected_origin_route_desc)
        << destination_stop_name << " origin route desc";
  }

  if (!expected_destination_route_desc.empty()) {
    std::string actual_destination_route_desc =
        steps_from_gtfs.mapping.GetRouteDescFromTrip(step.destination_trip);
    EXPECT_EQ(actual_destination_route_desc, expected_destination_route_desc)
        << destination_stop_name << " destination route desc";
  }
}

struct ExpectedPath {
  std::string destination_stop_name;
  std::string expected_departure_time;
  std::string expected_arrival_time;
  std::string expected_origin_route_desc;
  std::string expected_destination_route_desc;
};

struct ShortestPathTestCase {
  std::string test_name;
  std::string gtfs_path;
  std::string origin_stop_name;
  std::string origin_time;
  std::vector<ExpectedPath> expected_paths;
};

class ShortestPathParameterizedTest
    : public ::testing::TestWithParam<ShortestPathTestCase> {};

}  // namespace

TEST_P(ShortestPathParameterizedTest, FindShortestPathsAtTime) {
  const auto& test_case = GetParam();

  try {
    GtfsDay gtfs_day = GtfsLoadDay(test_case.gtfs_path);
    gtfs_day = GtfsNormalizeStops(gtfs_day);
    StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(gtfs_day);
    StepsAdjacencyList adjacency_list =
        MakeAdjacencyList(steps_from_gtfs.steps);

    TimeSinceServiceStart query_time =
        TimeSinceServiceStart::Parse(test_case.origin_time);

    std::unordered_set<StopId> destinations;
    for (const auto& path : test_case.expected_paths) {
      destinations.insert(
          steps_from_gtfs.mapping.GetStopIdFromName(path.destination_stop_name)
      );
    }

    StopId origin_stop =
        steps_from_gtfs.mapping.GetStopIdFromName(test_case.origin_stop_name);
    auto shortest_paths = FindShortestPathsAtTime(
        adjacency_list, query_time, origin_stop, destinations
    );

    EXPECT_EQ(shortest_paths.size(), test_case.expected_paths.size());

    for (const auto& expected_path : test_case.expected_paths) {
      VerifyPathResult(
          shortest_paths,
          steps_from_gtfs,
          expected_path.destination_stop_name,
          expected_path.expected_departure_time,
          expected_path.expected_arrival_time,
          expected_path.expected_origin_route_desc,
          expected_path.expected_destination_route_desc
      );
    }

  } catch (const std::exception& e) {
    FAIL() << "Exception in test case " << test_case.test_name << ": "
           << e.what();
  }
}

INSTANTIATE_TEST_SUITE_P(
    ShortestPathRealDataTests,
    ShortestPathParameterizedTest,
    ::testing::Values(
        ShortestPathTestCase{
            .test_name = "BerryessaTo5Destinations",
            .gtfs_path = "../data/RG_20250718_BA",
            .origin_stop_name = "Berryessa / North San Jose",
            .origin_time = "08:00:00",
            .expected_paths =
                {{"Powell",
                  "08:05:00",
                  "09:11:00",
                  "Green-S South",
                  "Green-S South"},
                 {"Dublin / Pleasanton BART",
                  "08:05:00",
                  "09:08:00",
                  "Green-S South",
                  "Blue-N North"},
                 {"Bay Fair",
                  "08:05:00",
                  "08:40:00",
                  "Green-S South",
                  "Green-S South"},
                 {"Antioch",
                  "08:05:00",
                  "10:14:00",
                  "Green-S South",
                  "Yellow-N North"},
                 {"Millbrae BART",
                  "08:05:00",
                  "10:07:00",
                  "Green-S South",
                  "Red-S South"}}
        },
        ShortestPathTestCase{
            .test_name = "WarmSpringsToMillbraeWithCaltrain",
            .gtfs_path = "../data/RG_20250718_BA_CT_SC",
            .origin_stop_name = "Warm Springs South Fremont BART",
            .origin_time = "07:49:00",
            .expected_paths =
                {{"Millbrae BART",
                  "07:52:00",
                  "09:33:00",
                  "Orange-S South",
                  "Limited North"}}
        },
        ShortestPathTestCase{
            .test_name = "WalkToCaltrainMillbrae",
            .gtfs_path = "../data/RG_20250718_BA_CT_SC",
            .origin_stop_name = "Sunnyvale & Hendy",
            .origin_time = "08:00:00",
            .expected_paths =
                {{"Millbrae BART",
                  "08:00:00",
                  "08:54:00",
                  "Walk from Sunnyvale & Hendy to Sunnyvale",
                  "Local Weekday North"}}
        }
    ),
    [](const ::testing::TestParamInfo<ShortestPathTestCase>& info) {
      return info.param.test_name;
    }
);

}  // namespace vats5