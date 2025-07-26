#include "solver/shortest_path.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <iostream>
#include <random>
#include <unordered_map>

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

struct CachedTestData {
  StepsFromGtfs steps_from_gtfs;
  StepsAdjacencyList adjacency_list;
};

CachedTestData GetCachedTestData(const std::string& gtfs_path) {
  static std::unordered_map<std::string, CachedTestData> cache;

  auto it = cache.find(gtfs_path);
  if (it != cache.end()) {
    return it->second;
  }

  GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);
  gtfs_day = GtfsNormalizeStops(gtfs_day);
  StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(gtfs_day);
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);

  CachedTestData data{std::move(steps_from_gtfs), std::move(adjacency_list)};
  cache[gtfs_path] = data;
  return data;
}

}  // namespace

TEST_P(ShortestPathParameterizedTest, FindShortestPathsAtTime) {
  const auto& test_case = GetParam();

  try {
    auto cached_data = GetCachedTestData(test_case.gtfs_path);
    const auto& steps_from_gtfs = cached_data.steps_from_gtfs;
    const auto& adjacency_list = cached_data.adjacency_list;

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

TEST(ShortestPathTest, FlexTripWithRegularTripsAvailable) {
  // Create a scenario where a stop has both flex trip (walking) and regular
  // scheduled trips The bug is that when is_flex_trip is true, it only
  // considers the flex edge and skips regular edges But it should consider both
  // when has_flex_trip is true

  std::vector<Step> steps = {
      // Regular scheduled trip from stop 1 to stop 2
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{100},       // Departs at 100
          .destination_time = TimeSinceServiceStart{200},  // Arrives at 200
          .origin_trip = TripId{1},
          .destination_trip = TripId{1}
      },
      // Flex trip (walking) from stop 1 to stop 2 - should be first in group to
      // trigger bug
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time =
              TimeSinceServiceStart::FLEX_STEP_MARKER,  // Flex marker
          .destination_time = TimeSinceServiceStart{300
          },  // Duration of 300 seconds (5 minutes)
          .origin_trip = TripId{2},
          .destination_trip = TripId{2}
      }
  };

  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps);

  // Query at time 50 - before the scheduled trip departs
  // With the bug: only considers flex trip, arrives at 50+300=350
  // Without the bug: should consider both, and take scheduled trip arriving at
  // 200
  TimeSinceServiceStart query_time{50};
  StopId origin_stop{1};
  std::unordered_set<StopId> destinations{StopId{2}};

  auto shortest_paths = FindShortestPathsAtTime(
      adjacency_list, query_time, origin_stop, destinations
  );

  ASSERT_EQ(shortest_paths.size(), 1);
  const Step& result = shortest_paths[StopId{2}];

  // With the bug, this will be 350 (50 + 300 flex duration)
  // Without the bug, this should be 200 (scheduled trip arrival)
  EXPECT_EQ(
      result.destination_time.seconds, 200
  ) << "Should take scheduled trip arriving at 200, not flex trip arriving at "
    << result.destination_time.seconds;
}

TEST(ShortestPathTest, SuboptimalDepartureTimeExposure) {
  // This test exposes a case where `FindShortestPathsAtTime` does not find the
  // latest possible departure time from the origin.

  // When there are frequent connections A->B and infrequent connections B->C,
  // the algorithm might find a path that departs early but has to wait a long
  // time at B, when a later departure from A could result in a later departure
  // time.

  std::vector<Step> steps = {
      // Frequent trips from A (stop 1) to B (stop 2)
      // Trip 1: A->B departing at 100, arriving at 110
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{100},
          .destination_time = TimeSinceServiceStart{110},
          .origin_trip = TripId{1},
          .destination_trip = TripId{1}
      },
      // Trip 2: A->B departing at 120, arriving at 130
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{120},
          .destination_time = TimeSinceServiceStart{130},
          .origin_trip = TripId{2},
          .destination_trip = TripId{2}
      },
      // Trip 3: A->B departing at 180, arriving at 190
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{180},
          .destination_time = TimeSinceServiceStart{190},
          .origin_trip = TripId{3},
          .destination_trip = TripId{3}
      },

      // Infrequent trips from B (stop 2) to C (stop 3)
      // Only one trip: B->C departing at 200, arriving at 210
      Step{
          .origin_stop = StopId{2},
          .destination_stop = StopId{3},
          .origin_time = TimeSinceServiceStart{200},
          .destination_time = TimeSinceServiceStart{210},
          .origin_trip = TripId{4},
          .destination_trip = TripId{4}
      }
  };

  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps);

  // Query at time 90 (before any departures)
  TimeSinceServiceStart query_time{90};
  StopId origin_stop{1};                               // Stop A
  std::unordered_set<StopId> destinations{StopId{3}};  // Stop C

  auto shortest_paths = FindShortestPathsAtTime(
      adjacency_list, query_time, origin_stop, destinations
  );

  ASSERT_EQ(shortest_paths.size(), 1);
  const Step& result = shortest_paths[StopId{3}];

  // The algorithm will likely choose:
  // - Depart A at 100, arrive B at 110
  // - Wait at B from 110 to 200 (90 seconds of waiting!)
  // - Depart B at 200, arrive C at 210
  // Total travel time: 210 - 100 = 110 seconds (with 90 seconds waiting)
  //
  // But a better choice would be:
  // - Depart A at 180, arrive B at 190
  // - Wait at B from 190 to 200 (only 10 seconds of waiting)
  // - Depart B at 200, arrive C at 210
  // Total travel time: 210 - 180 = 30 seconds (with only 10 seconds waiting)
  //
  // However, once the algorithm reaches B at time 110, it won't consider
  // the later departure from A at 180.

  // Assert the "suboptimal" returned result.
  EXPECT_EQ(result.origin_time.seconds, 100);
  EXPECT_EQ(result.destination_time.seconds, 210);
}

}  // namespace vats5