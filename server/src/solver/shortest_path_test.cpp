#include "solver/shortest_path.h"

#include <gmock/gmock.h>
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
          .destination_trip = TripId{1},
          .is_flex = false
      },
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{150},
          .destination_time = TimeSinceServiceStart{250},
          .origin_trip = TripId{2},
          .destination_trip = TripId{2},
          .is_flex = false
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
    const std::string& expected_destination_route_desc,
    const bool expected_is_flex
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

  EXPECT_EQ(step.is_flex, expected_is_flex);
}

struct ExpectedPath {
  std::string destination_stop_name;
  std::string expected_departure_time;
  std::string expected_arrival_time;
  std::string expected_origin_route_desc;
  std::string expected_destination_route_desc;
  bool expected_is_flex;
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

  std::cout << "Loading for test: " << gtfs_path << "\n";
  GtfsDay gtfs_day = GtfsLoadDay(gtfs_path);
  std::cout << "Normalizing stops...\n";
  gtfs_day = GtfsNormalizeStops(gtfs_day);
  std::cout << "Getting steps...\n";
  StepsFromGtfs steps_from_gtfs =
      GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});
  std::cout << "Making adjacency list...\n";
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);
  std::cout << "Done computing test data.\n";

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
          expected_path.expected_destination_route_desc,
          expected_path.expected_is_flex
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
                  "Green-S South",
                  false},
                 {"Dublin / Pleasanton BART",
                  "08:05:00",
                  "09:08:00",
                  "Green-S South",
                  "Blue-N North",
                  false},
                 {"Bay Fair",
                  "08:05:00",
                  "08:40:00",
                  "Green-S South",
                  "Green-S South",
                  false},
                 {"Antioch",
                  "08:05:00",
                  "10:14:00",
                  "Green-S South",
                  "Yellow-N North",
                  false},
                 {"Millbrae BART",
                  "08:05:00",
                  "10:07:00",
                  "Green-S South",
                  "Red-S South",
                  false}}
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
                  "Limited North",
                  false}}
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
                  "Local Weekday North",
                  false}}
        },

        ShortestPathTestCase{
            .test_name = "WarmSpringsToMillbraeVERYEARLY",
            .gtfs_path = "../data/RG_20250718_BA_CT_SC",
            .origin_stop_name = "Berryessa / North San Jose",
            .origin_time = "00:00:00",
            .expected_paths =
                {{"Great Mall/Milpitas BART",
                  "00:00:00",
                  "01:23:21",
                  "Walk from Berryessa / North San Jose to Berryessa Transit "
                  "Center",
                  "Walk from Lundy & Trade Zone to Great Mall/Milpitas BART",
                  true}}
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
          .destination_trip = TripId{1},
          .is_flex = false
      },
      // Flex trip (walking) from stop 1 to stop 2 - should be first in group to
      // trigger bug
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{0},  // origin time for flex
          .destination_time = TimeSinceServiceStart{300
          },  // Duration of 300 seconds (5 minutes)
          .origin_trip = TripId{2},
          .destination_trip = TripId{2},
          .is_flex = true
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

MATCHER_P5(
    IsStep,
    origin_stop,
    destination_stop,
    origin_time,
    destination_time,
    is_flex,
    ""
) {
  return arg.origin_stop == origin_stop &&
         arg.destination_stop == destination_stop &&
         arg.origin_time.ToString() == origin_time &&
         arg.destination_time.ToString() == destination_time &&
         arg.is_flex == is_flex;
}

TEST(ShortestPathTest, FindMinimalPathSetMilpitasToFruitvale) {
  const auto test_data =
      GetCachedTestData("../data/RG_20250718_BA_CT_SC_SM_AC");
  StopId milpitas =
      test_data.steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
          GtfsStopId{"mtc:great-mall-milpitas-bart"}
      );
  StopId fruitvale =
      test_data.steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
          GtfsStopId{"mtc:fruitvale"}
      );

  std::vector<Step> minimal_path_set =
      FindMinimalPathSet(test_data.adjacency_list, milpitas, fruitvale);

  auto S = [&](const char* origin_time, const char* destination_time) {
    return IsStep(milpitas, fruitvale, origin_time, destination_time, false);
  };

  using ::testing::ElementsAre;
  EXPECT_THAT(
      minimal_path_set,
      ElementsAre(
          S("00:20:00", "05:00:00"),
          S("01:00:00", "05:16:00"),
          S("04:49:00", "05:31:00"),
          S("05:01:00", "05:43:00"),
          S("05:09:00", "05:51:00"),
          S("05:21:00", "06:03:00"),
          S("05:29:00", "06:11:00"),
          S("05:41:00", "06:23:00"),
          S("05:49:00", "06:31:00"),
          S("06:01:00", "06:43:00"),
          S("06:09:00", "06:51:00"),
          S("06:21:00", "07:03:00"),
          S("06:29:00", "07:11:00"),
          S("06:41:00", "07:23:00"),
          S("06:49:00", "07:31:00"),
          S("07:01:00", "07:43:00"),
          S("07:09:00", "07:51:00"),
          S("07:21:00", "08:03:00"),
          S("07:29:00", "08:11:00"),
          S("07:41:00", "08:23:00"),
          S("07:49:00", "08:31:00"),
          S("08:01:00", "08:43:00"),
          S("08:09:00", "08:51:00"),
          S("08:21:00", "09:03:00"),
          S("08:29:00", "09:11:00"),
          S("08:41:00", "09:23:00"),
          S("08:49:00", "09:31:00"),
          S("09:01:00", "09:43:00"),
          S("09:09:00", "09:51:00"),
          S("09:21:00", "10:03:00"),
          S("09:29:00", "10:11:00"),
          S("09:41:00", "10:23:00"),
          S("09:49:00", "10:31:00"),
          S("10:01:00", "10:43:00"),
          S("10:09:00", "10:51:00"),
          S("10:21:00", "11:03:00"),
          S("10:29:00", "11:11:00"),
          S("10:41:00", "11:23:00"),
          S("10:49:00", "11:31:00"),
          S("11:01:00", "11:43:00"),
          S("11:09:00", "11:51:00"),
          S("11:21:00", "12:03:00"),
          S("11:29:00", "12:11:00"),
          S("11:41:00", "12:23:00"),
          S("11:49:00", "12:31:00"),
          S("12:01:00", "12:43:00"),
          S("12:09:00", "12:51:00"),
          S("12:21:00", "13:03:00"),
          S("12:29:00", "13:11:00"),
          S("12:41:00", "13:23:00"),
          S("12:49:00", "13:31:00"),
          S("13:01:00", "13:43:00"),
          S("13:09:00", "13:51:00"),
          S("13:21:00", "14:03:00"),
          S("13:29:00", "14:11:00"),
          S("13:41:00", "14:23:00"),
          S("13:49:00", "14:31:00"),
          S("14:01:00", "14:43:00"),
          S("14:09:00", "14:51:00"),
          S("14:21:00", "15:03:00"),
          S("14:29:00", "15:11:00"),
          S("14:41:00", "15:23:00"),
          S("14:49:00", "15:31:00"),
          S("15:01:00", "15:43:00"),
          S("15:09:00", "15:51:00"),
          S("15:21:00", "16:03:00"),
          S("15:29:00", "16:11:00"),
          S("15:41:00", "16:23:00"),
          S("15:49:00", "16:31:00"),
          S("16:01:00", "16:43:00"),
          S("16:09:00", "16:51:00"),
          S("16:21:00", "17:03:00"),
          S("16:29:00", "17:11:00"),
          S("16:41:00", "17:23:00"),
          S("16:49:00", "17:31:00"),
          S("17:01:00", "17:43:00"),
          S("17:09:00", "17:51:00"),
          S("17:21:00", "18:03:00"),
          S("17:29:00", "18:11:00"),
          S("17:41:00", "18:23:00"),
          S("17:49:00", "18:31:00"),
          S("18:01:00", "18:43:00"),
          S("18:09:00", "18:51:00"),
          S("18:21:00", "19:03:00"),
          S("18:29:00", "19:11:00"),
          S("18:41:00", "19:23:00"),
          S("18:49:00", "19:31:00"),
          S("19:01:00", "19:43:00"),
          S("19:21:00", "20:03:00"),
          S("19:41:00", "20:23:00"),
          S("20:01:00", "20:43:00"),
          S("20:21:00", "21:03:00"),
          S("20:41:00", "21:23:00"),
          S("21:01:00", "21:43:00"),
          S("21:21:00", "22:03:00"),
          S("21:41:00", "22:23:00"),
          S("22:01:00", "22:43:00"),
          S("22:21:00", "23:03:00"),
          S("22:41:00", "23:23:00"),
          S("23:01:00", "23:43:00"),
          S("23:21:00", "24:03:00"),
          S("23:38:00", "24:20:00"),
          S("23:54:00", "24:36:00")
      )
  );
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
          .destination_trip = TripId{1},
          .is_flex = false
      },
      // Trip 2: A->B departing at 120, arriving at 130
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{120},
          .destination_time = TimeSinceServiceStart{130},
          .origin_trip = TripId{2},
          .destination_trip = TripId{2},
          .is_flex = false
      },
      // Trip 3: A->B departing at 180, arriving at 190
      Step{
          .origin_stop = StopId{1},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{180},
          .destination_time = TimeSinceServiceStart{190},
          .origin_trip = TripId{3},
          .destination_trip = TripId{3},
          .is_flex = false
      },

      // Infrequent trips from B (stop 2) to C (stop 3)
      // Only one trip: B->C departing at 200, arriving at 210
      Step{
          .origin_stop = StopId{2},
          .destination_stop = StopId{3},
          .origin_time = TimeSinceServiceStart{200},
          .destination_time = TimeSinceServiceStart{210},
          .origin_trip = TripId{4},
          .destination_trip = TripId{4},
          .is_flex = false
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