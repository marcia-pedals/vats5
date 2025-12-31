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
    const std::unordered_map<StopId, PathState>& shortest_paths,
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

  const Step& step = path_it->second.whole_step;

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
                  "08:07:09",
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

  const Step& result = shortest_paths[StopId{2}].whole_step;

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

MATCHER_P5(
    MergedStepIs,
    origin_stop,
    destination_stop,
    origin_time,
    destination_time,
    is_flex,
    ""
) {
  return ExplainMatchResult(
      IsStep(
          origin_stop, destination_stop, origin_time, destination_time, is_flex
      ),
      arg.merged_step,
      result_listener
  );
}

void PrintPaths(
    const std::vector<Path>& paths,
    const char* prefix,
    const DataGtfsMapping* mapping = nullptr
) {
  for (const auto& p : paths) {
    const auto& s = p.merged_step;
    std::cout << prefix;
    if (s.is_flex) {
      std::cout << "Flex";
    }
    std::cout << "(\"" << s.origin_time.ToString() << "\", \""
              << s.destination_time.ToString() << "\"),\n";
    if (mapping != nullptr) {
      for (const auto& step : p.steps) {
        std::cout << "  ";
        if (step.is_flex) {
          std::cout << "Flex ";
        }
        const auto& origin_name =
            mapping->stop_id_to_stop_name.at(step.origin_stop);
        const auto& dest_name =
            mapping->stop_id_to_stop_name.at(step.destination_stop);
        const auto& route_desc =
            mapping->GetRouteDescFromTrip(step.origin_trip);
        std::cout << "(" << origin_name << " -> " << dest_name << ", \""
                  << step.origin_time.ToString() << "\" -> \""
                  << step.destination_time.ToString() << "\", " << route_desc
                  << ")\n";
      }
    }
  }
}

TEST(ShortestPathTest, FindMinimalPathSetMilpitasToBerryessa) {
  const auto test_data =
      GetCachedTestData("../data/RG_20260109_BA_CT_SC_SM_AC");
  StopId milpitas =
      test_data.steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
          GtfsStopId{"mtc:great-mall-milpitas-bart"}
      );
  StopId berryessa =
      test_data.steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
          GtfsStopId{"PS_BERR"}
      );
  std::unordered_map<StopId, std::vector<Path>> minimal_path_set =
      FindMinimalPathSet(test_data.adjacency_list, milpitas, {berryessa});
  auto B = [&](const char* origin_time, const char* destination_time) {
    return MergedStepIs(
        milpitas, berryessa, origin_time, destination_time, false
    );
  };
  auto BFlex = [&](const char* origin_time, const char* destination_time) {
    return MergedStepIs(
        milpitas, berryessa, origin_time, destination_time, true
    );
  };
  PrintPaths(
      minimal_path_set[berryessa], "B", &test_data.steps_from_gtfs.mapping
  );

  using ::testing::ElementsAre;
  EXPECT_THAT(
      minimal_path_set[berryessa],
      ElementsAre(
          BFlex("01:47:01", "03:08:13"),
          B("00:02:00", "00:09:09"),
          B("00:22:00", "00:29:09"),
          B("00:42:00", "00:49:09"),
          B("01:02:00", "01:09:09"),
          B("01:22:00", "01:29:09"),
          B("01:47:00", "01:54:09"),
          B("05:09:00", "05:16:09"),
          B("05:29:00", "05:36:09"),
          B("05:49:00", "05:56:09"),
          B("06:00:00", "06:07:09"),
          B("06:09:00", "06:16:09"),
          B("06:20:00", "06:27:09"),
          B("06:29:00", "06:36:09"),
          B("06:41:00", "06:47:09"),
          B("06:49:00", "06:56:09"),
          B("07:01:00", "07:07:09"),
          B("07:09:00", "07:16:09"),
          B("07:21:00", "07:27:09"),
          B("07:29:00", "07:36:09"),
          B("07:41:00", "07:47:09"),
          B("07:49:00", "07:56:09"),
          B("08:01:00", "08:07:09"),
          B("08:09:00", "08:16:09"),
          B("08:21:00", "08:27:09"),
          B("08:29:00", "08:36:09"),
          B("08:41:00", "08:47:09"),
          B("08:49:00", "08:56:09"),
          B("09:01:00", "09:07:09"),
          B("09:09:00", "09:16:09"),
          B("09:21:00", "09:27:09"),
          B("09:29:00", "09:36:09"),
          B("09:41:00", "09:47:09"),
          B("09:49:00", "09:56:09"),
          B("10:01:00", "10:07:09"),
          B("10:09:00", "10:16:09"),
          B("10:21:00", "10:27:09"),
          B("10:29:00", "10:36:09"),
          B("10:41:00", "10:47:09"),
          B("10:49:00", "10:56:09"),
          B("11:00:00", "11:07:09"),
          B("11:09:00", "11:16:09"),
          B("11:20:00", "11:27:09"),
          B("11:29:00", "11:36:09"),
          B("11:40:00", "11:47:09"),
          B("11:49:00", "11:56:09"),
          B("12:00:00", "12:07:09"),
          B("12:09:00", "12:16:09"),
          B("12:20:00", "12:27:09"),
          B("12:29:00", "12:36:09"),
          B("12:40:00", "12:47:09"),
          B("12:49:00", "12:56:09"),
          B("13:00:00", "13:07:09"),
          B("13:09:00", "13:16:09"),
          B("13:20:00", "13:27:09"),
          B("13:29:00", "13:36:09"),
          B("13:40:00", "13:47:09"),
          B("13:49:00", "13:56:09"),
          B("14:00:00", "14:07:09"),
          B("14:09:00", "14:16:09"),
          B("14:20:00", "14:27:09"),
          B("14:29:00", "14:36:09"),
          B("14:40:00", "14:47:09"),
          B("14:49:00", "14:56:09"),
          B("15:00:00", "15:07:09"),
          B("15:09:00", "15:16:09"),
          B("15:20:00", "15:27:09"),
          B("15:29:00", "15:36:09"),
          B("15:40:00", "15:47:09"),
          B("15:49:00", "15:56:09"),
          B("16:00:00", "16:07:09"),
          B("16:09:00", "16:16:09"),
          B("16:20:00", "16:27:09"),
          B("16:29:00", "16:36:09"),
          B("16:40:00", "16:47:09"),
          B("16:49:00", "16:56:09"),
          B("17:00:00", "17:07:09"),
          B("17:09:00", "17:16:09"),
          B("17:20:00", "17:27:09"),
          B("17:29:00", "17:36:09"),
          B("17:40:00", "17:47:09"),
          B("17:49:00", "17:56:09"),
          B("18:00:00", "18:07:09"),
          B("18:09:00", "18:16:09"),
          B("18:20:00", "18:27:09"),
          B("18:29:00", "18:36:09"),
          B("18:40:00", "18:47:09"),
          B("18:49:00", "18:56:09"),
          B("19:00:00", "19:07:09"),
          B("19:09:00", "19:16:09"),
          B("19:20:00", "19:27:09"),
          B("19:29:00", "19:36:09"),
          B("19:40:00", "19:47:09"),
          B("19:49:00", "19:56:09"),
          B("20:00:00", "20:07:09"),
          B("20:09:00", "20:16:09"),
          B("20:20:00", "20:27:09"),
          B("20:29:00", "20:36:09"),
          B("20:40:00", "20:47:09"),
          B("20:49:00", "20:56:09"),
          B("21:02:00", "21:09:09"),
          B("21:22:00", "21:29:09"),
          B("21:42:00", "21:49:09"),
          B("22:02:00", "22:09:09"),
          B("22:22:00", "22:29:09"),
          B("22:42:00", "22:49:09"),
          B("22:43:00", "22:55:00"),
          B("23:02:00", "23:09:09"),
          B("23:22:00", "23:29:09"),
          B("23:42:00", "23:49:09"),
          B("23:43:00", "23:55:00")
      )
  );
}

TEST(ShortestPathTest, FindMinimalPathSetFromFruitvale) {
  const auto test_data =
      GetCachedTestData("../data/RG_20260109_BA_CT_SC_SM_AC");
  StopId fruitvale =
      test_data.steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
          GtfsStopId{"mtc:fruitvale"}
      );
  StopId lake_merritt =
      test_data.steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
          GtfsStopId{"902109"}
      );
  StopId coliseum =
      test_data.steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
          GtfsStopId{"mtc:oakland-coliseum-bart"}
      );
  StopId san_leandro =
      test_data.steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
          GtfsStopId{"902409"}
      );
  StopId west_oakland =
      test_data.steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.at(
          GtfsStopId{"901109"}
      );

  std::unordered_map<StopId, std::vector<Path>> minimal_path_set =
      FindMinimalPathSet(
          test_data.adjacency_list,
          fruitvale,
          {lake_merritt, coliseum, san_leandro, west_oakland}
      );

  PrintPaths(minimal_path_set[lake_merritt], "LM");
  PrintPaths(minimal_path_set[coliseum], "COLS");
  PrintPaths(minimal_path_set[san_leandro], "SL");
  PrintPaths(minimal_path_set[west_oakland], "WO");

  //   auto F = [&](const char* origin_time, const char* destination_time) {
  //     return IsStep(milpitas, fruitvale, origin_time, destination_time,
  //     false);
  //   };

  //   auto M = [&](const char* origin_time, const char* destination_time) {
  //     return IsStep(milpitas, millbrae, origin_time, destination_time,
  //     false);
  //   };

  //   auto B = [&](const char* origin_time, const char* destination_time) {
  //     return IsStep(milpitas, berryessa, origin_time, destination_time,
  //     false);
  //   };

  //   auto BFlex = [&](const char* origin_time, const char* destination_time) {
  //     return IsStep(milpitas, berryessa, origin_time, destination_time,
  //     true);
  //   };
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

  const Step& result = shortest_paths[StopId{3}].whole_step;

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