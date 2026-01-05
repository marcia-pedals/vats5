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

// GoogleTest printer for Step - makes EXPECT_THAT output readable
void PrintTo(const Step& step, std::ostream* os) {
  *os << "Step{origin_stop: " << step.origin_stop
      << ", destination_stop: " << step.destination_stop << ", origin_time: \""
      << step.origin_time.ToString() << "\""
      << ", destination_time: \"" << step.destination_time.ToString() << "\""
      << ", is_flex: " << (step.is_flex ? "true" : "false") << "}";
}

// GoogleTest printer for Path - makes EXPECT_THAT output readable
void PrintTo(const Path& path, std::ostream* os) {
  *os << "Path{merged_step: ";
  PrintTo(path.merged_step, os);
  *os << ", steps: [";
  for (size_t i = 0; i < path.steps.size(); ++i) {
    if (i > 0) *os << ", ";
    PrintTo(path.steps[i], os);
  }
  *os << "]}";
}

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
  EXPECT_EQ(adjacency_list.adjacent[StopId{1}][0].steps.size(), 2);
}

namespace {

void VerifyPathResult(
    const std::vector<PathState>& shortest_paths,
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

  ASSERT_TRUE(
      shortest_paths[destination_stop.v].step.destination_time.seconds !=
      std::numeric_limits<int>::max()
  ) << destination_stop_name
    << " path not found";

  std::vector<Step> path_steps =
      BacktrackPath(shortest_paths, destination_stop);
  ASSERT_FALSE(path_steps.empty()) << destination_stop_name << " path is empty";

  Step step = ComputeMergedStep(path_steps);

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
  GtfsDay gtfs_day;
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

  CachedTestData data{
      std::move(gtfs_day), std::move(steps_from_gtfs), std::move(adjacency_list)
  };
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
                  "Walk from Lundy & Fortune to Great Mall/Milpitas BART",
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

  const Step& result = shortest_paths[2].step;

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
              << s.destination_time.ToString() << "\")";
    if (mapping != nullptr) {
      const auto& trip_info = mapping->trip_id_to_trip_info.at(s.origin_trip).v;
      if (std::holds_alternative<GtfsTripId>(trip_info)) {
        std::cout << " trip=" << std::get<GtfsTripId>(trip_info).v;
      }
    }
    std::cout << ",\n";
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

  using ::testing::ElementsAre;
  EXPECT_THAT(
      minimal_path_set[berryessa],
      ElementsAre(
          BFlex("00:00:00", "01:21:12"),
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
          B("23:43:00", "23:55:00"),
          B("24:02:00", "24:09:09"),
          B("24:22:00", "24:29:09"),
          B("24:42:00", "24:49:09"),
          B("25:02:00", "25:09:09"),
          B("25:22:00", "25:29:09"),
          B("25:47:00", "25:54:09"),
          B("30:09:00", "30:16:09"),
          B("30:29:00", "30:36:09"),
          B("30:49:00", "30:56:09"),
          B("31:01:00", "31:07:09"),
          B("31:09:00", "31:16:09"),
          B("31:21:00", "31:27:09"),
          B("31:29:00", "31:36:09"),
          B("31:41:00", "31:47:09"),
          B("31:49:00", "31:56:09"),
          B("32:01:00", "32:07:09"),
          B("32:09:00", "32:16:09"),
          B("32:21:00", "32:27:09"),
          B("32:29:00", "32:36:09"),
          B("32:41:00", "32:47:09"),
          B("32:49:00", "32:56:09"),
          B("33:01:00", "33:07:09"),
          B("33:09:00", "33:16:09"),
          B("33:21:00", "33:27:09"),
          B("33:29:00", "33:36:09"),
          B("33:41:00", "33:47:09"),
          B("33:49:00", "33:56:09"),
          B("34:01:00", "34:07:09"),
          B("34:09:00", "34:16:09"),
          B("34:21:00", "34:27:09"),
          B("34:29:00", "34:36:09"),
          B("34:41:00", "34:47:09"),
          B("34:49:00", "34:56:09"),
          B("35:00:00", "35:07:09"),
          B("35:09:00", "35:16:09"),
          B("35:20:00", "35:27:09"),
          B("35:29:00", "35:36:09"),
          B("35:40:00", "35:47:09"),
          B("35:49:00", "35:56:09")
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

  auto LM = [&](const char* origin_time, const char* destination_time) {
    return MergedStepIs(
        fruitvale, lake_merritt, origin_time, destination_time, false
    );
  };
  auto COLS = [&](const char* origin_time, const char* destination_time) {
    return MergedStepIs(
        fruitvale, coliseum, origin_time, destination_time, false
    );
  };
  auto SL = [&](const char* origin_time, const char* destination_time) {
    return MergedStepIs(
        fruitvale, san_leandro, origin_time, destination_time, false
    );
  };
  auto WO = [&](const char* origin_time, const char* destination_time) {
    return MergedStepIs(
        fruitvale, west_oakland, origin_time, destination_time, false
    );
  };

  using ::testing::ElementsAre;
  EXPECT_THAT(
      minimal_path_set[lake_merritt],
      ElementsAre(
          LM("00:07:00", "00:11:00"),
          LM("00:15:00", "00:19:00"),
          LM("00:27:00", "00:31:00"),
          LM("00:37:00", "00:41:00"),
          LM("01:03:48", "01:24:53"),
          LM("01:07:41", "01:37:01"),
          LM("01:14:00", "01:41:46"),
          LM("01:33:48", "01:54:53"),
          LM("02:03:48", "02:24:53"),
          LM("02:07:41", "02:37:01"),
          LM("02:14:00", "02:41:46"),
          LM("02:33:48", "02:54:53"),
          LM("03:03:48", "03:24:53"),
          LM("03:07:41", "03:37:01"),
          LM("03:14:00", "03:41:46"),
          LM("03:33:48", "03:54:53"),
          LM("04:03:48", "04:24:53"),
          LM("04:07:41", "04:37:01"),
          LM("04:14:00", "04:41:46"),
          LM("04:33:48", "04:54:53"),
          LM("04:58:00", "05:02:00"),
          LM("05:08:00", "05:12:00"),
          LM("05:18:00", "05:22:00"),
          LM("05:25:00", "05:29:00"),
          LM("05:28:00", "05:32:00"),
          LM("05:38:00", "05:42:00"),
          LM("05:45:00", "05:49:00"),
          LM("05:48:00", "05:52:00"),
          LM("05:58:00", "06:02:00"),
          LM("06:05:00", "06:09:00"),
          LM("06:08:00", "06:12:00"),
          LM("06:18:00", "06:22:00"),
          LM("06:25:00", "06:29:00"),
          LM("06:28:00", "06:32:00"),
          LM("06:38:00", "06:42:00"),
          LM("06:45:00", "06:49:00"),
          LM("06:48:00", "06:52:00"),
          LM("06:58:00", "07:02:00"),
          LM("07:05:00", "07:09:00"),
          LM("07:08:00", "07:12:00"),
          LM("07:18:00", "07:22:00"),
          LM("07:25:00", "07:29:00"),
          LM("07:28:00", "07:32:00"),
          LM("07:38:00", "07:42:00"),
          LM("07:45:00", "07:49:00"),
          LM("07:48:00", "07:52:00"),
          LM("07:58:00", "08:02:00"),
          LM("08:05:00", "08:09:00"),
          LM("08:08:00", "08:12:00"),
          LM("08:18:00", "08:22:00"),
          LM("08:25:00", "08:29:00"),
          LM("08:28:00", "08:32:00"),
          LM("08:38:00", "08:42:00"),
          LM("08:45:00", "08:49:00"),
          LM("08:48:00", "08:52:00"),
          LM("08:58:00", "09:02:00"),
          LM("09:05:00", "09:09:00"),
          LM("09:08:00", "09:12:00"),
          LM("09:18:00", "09:22:00"),
          LM("09:25:00", "09:29:00"),
          LM("09:28:00", "09:32:00"),
          LM("09:38:00", "09:42:00"),
          LM("09:45:00", "09:49:00"),
          LM("09:48:00", "09:52:00"),
          LM("09:58:00", "10:02:00"),
          LM("10:05:00", "10:09:00"),
          LM("10:08:00", "10:12:00"),
          LM("10:18:00", "10:22:00"),
          LM("10:25:00", "10:29:00"),
          LM("10:28:00", "10:32:00"),
          LM("10:38:00", "10:42:00"),
          LM("10:45:00", "10:49:00"),
          LM("10:48:00", "10:52:00"),
          LM("10:58:00", "11:02:00"),
          LM("11:05:00", "11:09:00"),
          LM("11:08:00", "11:12:00"),
          LM("11:18:00", "11:22:00"),
          LM("11:25:00", "11:29:00"),
          LM("11:28:00", "11:32:00"),
          LM("11:38:00", "11:42:00"),
          LM("11:45:00", "11:49:00"),
          LM("11:48:00", "11:52:00"),
          LM("11:58:00", "12:02:00"),
          LM("12:05:00", "12:09:00"),
          LM("12:08:00", "12:12:00"),
          LM("12:18:00", "12:22:00"),
          LM("12:25:00", "12:29:00"),
          LM("12:28:00", "12:32:00"),
          LM("12:38:00", "12:42:00"),
          LM("12:45:00", "12:49:00"),
          LM("12:48:00", "12:52:00"),
          LM("12:58:00", "13:02:00"),
          LM("13:05:00", "13:09:00"),
          LM("13:08:00", "13:12:00"),
          LM("13:18:00", "13:22:00"),
          LM("13:25:00", "13:29:00"),
          LM("13:28:00", "13:32:00"),
          LM("13:38:00", "13:42:00"),
          LM("13:45:00", "13:49:00"),
          LM("13:48:00", "13:52:00"),
          LM("13:58:00", "14:02:00"),
          LM("14:05:00", "14:09:00"),
          LM("14:08:00", "14:12:00"),
          LM("14:18:00", "14:22:00"),
          LM("14:25:00", "14:29:00"),
          LM("14:28:00", "14:32:00"),
          LM("14:38:00", "14:42:00"),
          LM("14:45:00", "14:49:00"),
          LM("14:48:00", "14:52:00"),
          LM("14:58:00", "15:02:00"),
          LM("15:05:00", "15:09:00"),
          LM("15:08:00", "15:12:00"),
          LM("15:18:00", "15:22:00"),
          LM("15:25:00", "15:29:00"),
          LM("15:28:00", "15:32:00"),
          LM("15:38:00", "15:42:00"),
          LM("15:45:00", "15:49:00"),
          LM("15:48:00", "15:52:00"),
          LM("15:58:00", "16:02:00"),
          LM("16:05:00", "16:09:00"),
          LM("16:08:00", "16:12:00"),
          LM("16:18:00", "16:22:00"),
          LM("16:25:00", "16:29:00"),
          LM("16:28:00", "16:32:00"),
          LM("16:38:00", "16:42:00"),
          LM("16:45:00", "16:49:00"),
          LM("16:48:00", "16:52:00"),
          LM("16:58:00", "17:02:00"),
          LM("17:05:00", "17:09:00"),
          LM("17:08:00", "17:12:00"),
          LM("17:18:00", "17:22:00"),
          LM("17:25:00", "17:29:00"),
          LM("17:28:00", "17:32:00"),
          LM("17:38:00", "17:42:00"),
          LM("17:45:00", "17:49:00"),
          LM("17:48:00", "17:52:00"),
          LM("17:58:00", "18:02:00"),
          LM("18:05:00", "18:09:00"),
          LM("18:08:00", "18:12:00"),
          LM("18:18:00", "18:22:00"),
          LM("18:25:00", "18:29:00"),
          LM("18:28:00", "18:32:00"),
          LM("18:38:00", "18:42:00"),
          LM("18:45:00", "18:49:00"),
          LM("18:48:00", "18:52:00"),
          LM("18:58:00", "19:02:00"),
          LM("19:05:00", "19:09:00"),
          LM("19:08:00", "19:12:00"),
          LM("19:18:00", "19:22:00"),
          LM("19:25:00", "19:29:00"),
          LM("19:28:00", "19:32:00"),
          LM("19:38:00", "19:42:00"),
          LM("19:45:00", "19:49:00"),
          LM("19:48:00", "19:52:00"),
          LM("19:59:00", "20:03:00"),
          LM("20:07:00", "20:11:00"),
          LM("20:15:00", "20:19:00"),
          LM("20:27:00", "20:31:00"),
          LM("20:35:00", "20:39:00"),
          LM("20:47:00", "20:51:00"),
          LM("20:55:00", "20:59:00"),
          LM("21:07:00", "21:11:00"),
          LM("21:15:00", "21:19:00"),
          LM("21:27:00", "21:31:00"),
          LM("21:35:00", "21:39:00"),
          LM("21:47:00", "21:51:00"),
          LM("21:55:00", "21:59:00"),
          LM("22:07:00", "22:11:00"),
          LM("22:15:00", "22:19:00"),
          LM("22:27:00", "22:31:00"),
          LM("22:35:00", "22:39:00"),
          LM("22:47:00", "22:51:00"),
          LM("22:55:00", "22:59:00"),
          LM("23:07:00", "23:11:00"),
          LM("23:15:00", "23:19:00"),
          LM("23:27:00", "23:31:00"),
          LM("23:35:00", "23:39:00"),
          LM("23:47:00", "23:51:00"),
          LM("23:55:00", "23:59:00"),
          LM("24:07:00", "24:11:00"),
          LM("24:15:00", "24:19:00"),
          LM("24:27:00", "24:31:00"),
          LM("24:37:00", "24:41:00"),
          LM("25:03:48", "25:24:53"),
          LM("25:07:41", "25:37:01"),
          LM("25:14:00", "25:41:46"),
          LM("25:33:48", "25:54:53"),
          LM("26:03:48", "26:24:53"),
          LM("26:07:41", "26:37:01"),
          LM("26:14:00", "26:41:46"),
          LM("26:33:48", "26:54:53"),
          LM("27:03:48", "27:24:53"),
          LM("27:07:41", "27:37:01"),
          LM("27:14:00", "27:41:46"),
          LM("27:33:48", "27:54:53"),
          LM("28:03:48", "28:24:53"),
          LM("28:07:41", "28:37:01"),
          LM("28:14:00", "28:41:46"),
          LM("28:33:48", "28:54:53"),
          LM("29:03:48", "29:24:53"),
          LM("29:07:41", "29:37:01"),
          LM("29:24:00", "29:57:34"),
          LM("29:38:00", "29:59:40"),
          LM("29:58:00", "30:02:00"),
          LM("30:08:00", "30:12:00"),
          LM("30:18:00", "30:22:00"),
          LM("30:25:00", "30:29:00"),
          LM("30:28:00", "30:32:00"),
          LM("30:38:00", "30:42:00"),
          LM("30:45:00", "30:49:00"),
          LM("30:48:00", "30:52:00"),
          LM("30:58:00", "31:02:00"),
          LM("31:05:00", "31:09:00"),
          LM("31:08:00", "31:12:00"),
          LM("31:18:00", "31:22:00"),
          LM("31:25:00", "31:29:00"),
          LM("31:28:00", "31:32:00"),
          LM("31:38:00", "31:42:00"),
          LM("31:45:00", "31:49:00"),
          LM("31:48:00", "31:52:00"),
          LM("31:58:00", "32:02:00"),
          LM("32:05:00", "32:09:00"),
          LM("32:08:00", "32:12:00"),
          LM("32:18:00", "32:22:00"),
          LM("32:25:00", "32:29:00"),
          LM("32:28:00", "32:32:00"),
          LM("32:38:00", "32:42:00"),
          LM("32:45:00", "32:49:00"),
          LM("32:48:00", "32:52:00"),
          LM("32:58:00", "33:02:00"),
          LM("33:05:00", "33:09:00"),
          LM("33:08:00", "33:12:00"),
          LM("33:18:00", "33:22:00"),
          LM("33:25:00", "33:29:00"),
          LM("33:28:00", "33:32:00"),
          LM("33:38:00", "33:42:00"),
          LM("33:45:00", "33:49:00"),
          LM("33:48:00", "33:52:00"),
          LM("33:58:00", "34:02:00"),
          LM("34:05:00", "34:09:00"),
          LM("34:08:00", "34:12:00"),
          LM("34:18:00", "34:22:00"),
          LM("34:25:00", "34:29:00"),
          LM("34:28:00", "34:32:00"),
          LM("34:38:00", "34:42:00"),
          LM("34:45:00", "34:49:00"),
          LM("34:48:00", "34:52:00"),
          LM("34:58:00", "35:02:00"),
          LM("35:05:00", "35:09:00"),
          LM("35:08:00", "35:12:00"),
          LM("35:18:00", "35:22:00"),
          LM("35:25:00", "35:29:00"),
          LM("35:28:00", "35:32:00"),
          LM("35:38:00", "35:42:00"),
          LM("35:45:00", "35:49:00"),
          LM("35:48:00", "35:52:00"),
          LM("35:58:00", "36:02:00")
      )
  );
  EXPECT_THAT(
      minimal_path_set[coliseum],
      ElementsAre(
          COLS("00:08:00", "00:11:00"),
          COLS("00:19:00", "00:22:00"),
          COLS("00:28:00", "00:31:00"),
          COLS("00:39:00", "00:42:00"),
          COLS("00:47:00", "00:50:00"),
          COLS("01:04:00", "01:07:00"),
          COLS("01:23:55", "01:51:32"),
          COLS("01:53:55", "02:17:57"),
          COLS("02:23:55", "02:51:32"),
          COLS("02:53:55", "03:17:57"),
          COLS("03:23:55", "03:51:32"),
          COLS("03:53:55", "04:17:57"),
          COLS("04:23:55", "04:51:32"),
          COLS("04:53:55", "05:17:57"),
          COLS("05:17:00", "05:20:00"),
          COLS("05:26:00", "05:29:00"),
          COLS("05:37:00", "05:40:00"),
          COLS("05:43:00", "05:45:00"),
          COLS("05:46:00", "05:49:00"),
          COLS("05:57:00", "06:00:00"),
          COLS("06:03:00", "06:05:00"),
          COLS("06:06:00", "06:09:00"),
          COLS("06:17:00", "06:20:00"),
          COLS("06:23:00", "06:25:00"),
          COLS("06:26:00", "06:29:00"),
          COLS("06:37:00", "06:40:00"),
          COLS("06:43:00", "06:45:00"),
          COLS("06:46:00", "06:49:00"),
          COLS("06:57:00", "07:00:00"),
          COLS("07:03:00", "07:05:00"),
          COLS("07:06:00", "07:09:00"),
          COLS("07:17:00", "07:20:00"),
          COLS("07:23:00", "07:25:00"),
          COLS("07:26:00", "07:29:00"),
          COLS("07:37:00", "07:40:00"),
          COLS("07:43:00", "07:45:00"),
          COLS("07:46:00", "07:49:00"),
          COLS("07:57:00", "08:00:00"),
          COLS("08:03:00", "08:05:00"),
          COLS("08:06:00", "08:09:00"),
          COLS("08:17:00", "08:20:00"),
          COLS("08:23:00", "08:25:00"),
          COLS("08:26:00", "08:29:00"),
          COLS("08:37:00", "08:40:00"),
          COLS("08:43:00", "08:45:00"),
          COLS("08:46:00", "08:49:00"),
          COLS("08:57:00", "09:00:00"),
          COLS("09:03:00", "09:05:00"),
          COLS("09:06:00", "09:09:00"),
          COLS("09:17:00", "09:20:00"),
          COLS("09:23:00", "09:25:00"),
          COLS("09:26:00", "09:29:00"),
          COLS("09:37:00", "09:40:00"),
          COLS("09:43:00", "09:45:00"),
          COLS("09:46:00", "09:49:00"),
          COLS("09:57:00", "10:00:00"),
          COLS("10:03:00", "10:05:00"),
          COLS("10:06:00", "10:09:00"),
          COLS("10:17:00", "10:20:00"),
          COLS("10:23:00", "10:25:00"),
          COLS("10:26:00", "10:29:00"),
          COLS("10:37:00", "10:40:00"),
          COLS("10:43:00", "10:45:00"),
          COLS("10:46:00", "10:49:00"),
          COLS("10:57:00", "11:00:00"),
          COLS("11:03:00", "11:05:00"),
          COLS("11:06:00", "11:09:00"),
          COLS("11:17:00", "11:20:00"),
          COLS("11:23:00", "11:25:00"),
          COLS("11:26:00", "11:29:00"),
          COLS("11:37:00", "11:40:00"),
          COLS("11:43:00", "11:45:00"),
          COLS("11:46:00", "11:49:00"),
          COLS("11:57:00", "12:00:00"),
          COLS("12:03:00", "12:05:00"),
          COLS("12:06:00", "12:09:00"),
          COLS("12:17:00", "12:20:00"),
          COLS("12:23:00", "12:25:00"),
          COLS("12:26:00", "12:29:00"),
          COLS("12:37:00", "12:40:00"),
          COLS("12:43:00", "12:45:00"),
          COLS("12:46:00", "12:49:00"),
          COLS("12:57:00", "13:00:00"),
          COLS("13:03:00", "13:05:00"),
          COLS("13:06:00", "13:09:00"),
          COLS("13:17:00", "13:20:00"),
          COLS("13:23:00", "13:25:00"),
          COLS("13:26:00", "13:29:00"),
          COLS("13:37:00", "13:40:00"),
          COLS("13:43:00", "13:45:00"),
          COLS("13:46:00", "13:49:00"),
          COLS("13:57:00", "14:00:00"),
          COLS("14:03:00", "14:05:00"),
          COLS("14:06:00", "14:09:00"),
          COLS("14:17:00", "14:20:00"),
          COLS("14:23:00", "14:25:00"),
          COLS("14:26:00", "14:29:00"),
          COLS("14:37:00", "14:40:00"),
          COLS("14:43:00", "14:45:00"),
          COLS("14:46:00", "14:49:00"),
          COLS("14:57:00", "15:00:00"),
          COLS("15:03:00", "15:05:00"),
          COLS("15:06:00", "15:09:00"),
          COLS("15:17:00", "15:20:00"),
          COLS("15:23:00", "15:25:00"),
          COLS("15:26:00", "15:29:00"),
          COLS("15:37:00", "15:40:00"),
          COLS("15:43:00", "15:45:00"),
          COLS("15:46:00", "15:49:00"),
          COLS("15:57:00", "16:00:00"),
          COLS("16:03:00", "16:05:00"),
          COLS("16:06:00", "16:09:00"),
          COLS("16:17:00", "16:20:00"),
          COLS("16:23:00", "16:25:00"),
          COLS("16:26:00", "16:29:00"),
          COLS("16:37:00", "16:40:00"),
          COLS("16:43:00", "16:45:00"),
          COLS("16:46:00", "16:49:00"),
          COLS("16:57:00", "17:00:00"),
          COLS("17:03:00", "17:05:00"),
          COLS("17:06:00", "17:09:00"),
          COLS("17:17:00", "17:20:00"),
          COLS("17:23:00", "17:25:00"),
          COLS("17:26:00", "17:29:00"),
          COLS("17:37:00", "17:40:00"),
          COLS("17:43:00", "17:45:00"),
          COLS("17:46:00", "17:49:00"),
          COLS("17:57:00", "18:00:00"),
          COLS("18:03:00", "18:05:00"),
          COLS("18:06:00", "18:09:00"),
          COLS("18:17:00", "18:20:00"),
          COLS("18:23:00", "18:25:00"),
          COLS("18:26:00", "18:29:00"),
          COLS("18:37:00", "18:40:00"),
          COLS("18:43:00", "18:45:00"),
          COLS("18:46:00", "18:49:00"),
          COLS("18:57:00", "19:00:00"),
          COLS("19:03:00", "19:05:00"),
          COLS("19:06:00", "19:09:00"),
          COLS("19:17:00", "19:20:00"),
          COLS("19:23:00", "19:25:00"),
          COLS("19:26:00", "19:29:00"),
          COLS("19:37:00", "19:40:00"),
          COLS("19:43:00", "19:45:00"),
          COLS("19:46:00", "19:49:00"),
          COLS("19:57:00", "20:00:00"),
          COLS("20:03:00", "20:05:00"),
          COLS("20:06:00", "20:09:00"),
          COLS("20:19:00", "20:22:00"),
          COLS("20:28:00", "20:31:00"),
          COLS("20:39:00", "20:42:00"),
          COLS("20:48:00", "20:51:00"),
          COLS("20:59:00", "21:02:00"),
          COLS("21:08:00", "21:11:00"),
          COLS("21:19:00", "21:22:00"),
          COLS("21:28:00", "21:31:00"),
          COLS("21:39:00", "21:42:00"),
          COLS("21:48:00", "21:51:00"),
          COLS("21:59:00", "22:02:00"),
          COLS("22:08:00", "22:11:00"),
          COLS("22:19:00", "22:22:00"),
          COLS("22:28:00", "22:31:00"),
          COLS("22:39:00", "22:42:00"),
          COLS("22:48:00", "22:51:00"),
          COLS("22:59:00", "23:02:00"),
          COLS("23:08:00", "23:11:00"),
          COLS("23:19:00", "23:22:00"),
          COLS("23:28:00", "23:31:00"),
          COLS("23:39:00", "23:42:00"),
          COLS("23:48:00", "23:51:00"),
          COLS("23:59:00", "24:02:00"),
          COLS("24:08:00", "24:11:00"),
          COLS("24:19:00", "24:22:00"),
          COLS("24:28:00", "24:31:00"),
          COLS("24:39:00", "24:42:00"),
          COLS("24:47:00", "24:50:00"),
          COLS("25:04:00", "25:07:00"),
          COLS("25:23:55", "25:51:32"),
          COLS("25:53:55", "26:18:57"),
          COLS("26:23:55", "26:51:32"),
          COLS("26:53:55", "27:18:57"),
          COLS("27:23:55", "27:51:32"),
          COLS("27:53:55", "28:18:57"),
          COLS("28:23:55", "28:51:32"),
          COLS("28:53:55", "29:18:57"),
          COLS("29:23:55", "29:48:57"),
          COLS("29:53:55", "30:13:57"),
          COLS("30:17:00", "30:20:00"),
          COLS("30:26:00", "30:29:00"),
          COLS("30:37:00", "30:40:00"),
          COLS("30:43:00", "30:45:00"),
          COLS("30:46:00", "30:49:00"),
          COLS("30:57:00", "31:00:00"),
          COLS("31:03:00", "31:05:00"),
          COLS("31:06:00", "31:09:00"),
          COLS("31:17:00", "31:20:00"),
          COLS("31:23:00", "31:25:00"),
          COLS("31:26:00", "31:29:00"),
          COLS("31:37:00", "31:40:00"),
          COLS("31:43:00", "31:45:00"),
          COLS("31:46:00", "31:49:00"),
          COLS("31:57:00", "32:00:00"),
          COLS("32:03:00", "32:05:00"),
          COLS("32:06:00", "32:09:00"),
          COLS("32:17:00", "32:20:00"),
          COLS("32:23:00", "32:25:00"),
          COLS("32:26:00", "32:29:00"),
          COLS("32:37:00", "32:40:00"),
          COLS("32:43:00", "32:45:00"),
          COLS("32:46:00", "32:49:00"),
          COLS("32:57:00", "33:00:00"),
          COLS("33:03:00", "33:05:00"),
          COLS("33:06:00", "33:09:00"),
          COLS("33:17:00", "33:20:00"),
          COLS("33:23:00", "33:25:00"),
          COLS("33:26:00", "33:29:00"),
          COLS("33:37:00", "33:40:00"),
          COLS("33:43:00", "33:45:00"),
          COLS("33:46:00", "33:49:00"),
          COLS("33:57:00", "34:00:00"),
          COLS("34:03:00", "34:05:00"),
          COLS("34:06:00", "34:09:00"),
          COLS("34:17:00", "34:20:00"),
          COLS("34:23:00", "34:25:00"),
          COLS("34:26:00", "34:29:00"),
          COLS("34:37:00", "34:40:00"),
          COLS("34:43:00", "34:45:00"),
          COLS("34:46:00", "34:49:00"),
          COLS("34:57:00", "35:00:00"),
          COLS("35:03:00", "35:05:00"),
          COLS("35:06:00", "35:09:00"),
          COLS("35:17:00", "35:20:00"),
          COLS("35:23:00", "35:25:00"),
          COLS("35:26:00", "35:29:00"),
          COLS("35:37:00", "35:40:00"),
          COLS("35:43:00", "35:45:00"),
          COLS("35:46:00", "35:49:00"),
          COLS("35:57:00", "36:00:00")
      )
  );
  EXPECT_THAT(
      minimal_path_set[san_leandro],
      ElementsAre(
          SL("01:23:55", "01:50:55"),
          SL("01:53:55", "02:20:55"),
          SL("02:23:55", "02:50:55"),
          SL("02:53:55", "03:20:55"),
          SL("03:23:55", "03:50:55"),
          SL("03:53:55", "04:20:55"),
          SL("04:23:55", "04:50:55"),
          SL("04:53:55", "05:20:55"),
          SL("25:23:55", "25:50:55"),
          SL("25:53:55", "26:20:55"),
          SL("26:23:55", "26:50:55"),
          SL("26:53:55", "27:20:55"),
          SL("27:23:55", "27:50:55"),
          SL("27:53:55", "28:20:55"),
          SL("28:23:55", "28:50:55"),
          SL("28:53:55", "29:20:55"),
          SL("29:23:55", "29:50:55"),
          SL("29:53:55", "30:20:55")
      )
  );
  EXPECT_THAT(
      minimal_path_set[west_oakland],
      ElementsAre(
          WO("01:03:48", "01:56:30"),
          WO("01:14:00", "02:07:02"),
          WO("01:33:48", "02:26:30"),
          WO("02:03:48", "02:56:30"),
          WO("02:14:00", "03:07:02"),
          WO("02:33:48", "03:26:30"),
          WO("03:03:48", "03:56:30"),
          WO("03:14:00", "04:07:02"),
          WO("03:33:48", "04:26:30"),
          WO("04:03:48", "04:56:30"),
          WO("04:20:00", "05:01:58"),
          WO("25:03:48", "25:56:30"),
          WO("25:14:00", "26:07:02"),
          WO("25:33:48", "26:26:30"),
          WO("26:03:48", "26:56:30"),
          WO("26:14:00", "27:07:02"),
          WO("26:33:48", "27:26:30"),
          WO("27:03:48", "27:56:30"),
          WO("27:14:00", "28:07:02"),
          WO("27:33:48", "28:26:30"),
          WO("28:03:48", "28:56:30"),
          WO("28:14:00", "29:07:02"),
          WO("28:33:48", "29:26:30"),
          WO("29:03:48", "29:54:56")
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

  std::vector<Step> path_steps = BacktrackPath(shortest_paths, StopId{3});

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
  ASSERT_FALSE(path_steps.empty());
  EXPECT_EQ(path_steps.front().origin_time.seconds, 100);
  EXPECT_EQ(path_steps.back().destination_time.seconds, 210);
}

TEST(ShortestPathTest, ReduceToMinimalSystemSteps_BART_AlreadyMinimal) {
  const auto test_data = GetCachedTestData("../data/RG_20250718_BA");

  StepsAdjacencyList adjacency_list = test_data.adjacency_list;

  std::unordered_set<StopId> bart_stops = GetStopsForTripIdPrefix(
      test_data.gtfs_day, test_data.steps_from_gtfs.mapping, "BA:"
  );

  // Remove self-loops because these are not minimal.
  // (For some reason BART GTFS has trips with 2 adjacent SFO stops).
  for (auto& [stop_id, dest_groups] : adjacency_list.adjacent) {
    dest_groups.erase(
        std::remove_if(
            dest_groups.begin(),
            dest_groups.end(),
            [&](const StepGroup& group) {
              return !group.steps.empty() &&
                     group.steps[0].destination_stop == stop_id;
            }
        ),
        dest_groups.end()
    );
  }

  // TODO: Separate out a SplitPathsAt test. Calling it here with an empty
  // `intermediate_stops` is just a kludge to make sure it's working in the most
  // basic way: Not changing anything when there are no intermediate stops.
  StepsAdjacencyList reduced = AdjacentPathsToStepsList(
      SplitPathsAt(ReduceToMinimalSystemPaths(adjacency_list, bart_stops), {})
  );

  // Sort destination groups by destination stop ID for consistent comparison
  auto sort_by_dest = [](std::vector<StepGroup>& groups) {
    std::sort(
        groups.begin(),
        groups.end(),
        [](const StepGroup& a, const StepGroup& b) {
          StopId dest_a =
              a.steps.empty() ? StopId{0} : a.steps[0].destination_stop;
          StopId dest_b =
              b.steps.empty() ? StopId{0} : b.steps[0].destination_stop;
          return dest_a.v < dest_b.v;
        }
    );
  };

  for (auto& [stop_id, groups] : adjacency_list.adjacent) {
    sort_by_dest(groups);
  }
  for (auto& [stop_id, groups] : reduced.adjacent) {
    sort_by_dest(groups);
  }

  // Compare step groups element-by-element (comparing steps only)
  EXPECT_EQ(adjacency_list.adjacent.size(), reduced.adjacent.size());
  for (const auto& [stop_id, groups] : adjacency_list.adjacent) {
    auto it = reduced.adjacent.find(stop_id);
    ASSERT_NE(it, reduced.adjacent.end()) << "Missing stop_id " << stop_id.v;
    ASSERT_EQ(groups.size(), it->second.size());
    for (size_t i = 0; i < groups.size(); ++i) {
      EXPECT_EQ(groups[i].steps, it->second[i].steps);
    }
  }
}

// TEST(ShortestPathTest, ReduceToMinimalSystemSteps_BART) {
//   std::string gtfs_path = "../data/RG_20260109_BA_CT_SC_SM_AC";
//   GtfsDay gtfs_day = GtfsNormalizeStops(GtfsLoadDay(gtfs_path));
//   StepsFromGtfs steps_from_gtfs =
//       GetStepsFromGtfs(gtfs_day, GetStepsOptions{1000.0});
//   StepsAdjacencyList adjacency_list =
//   MakeAdjacencyList(steps_from_gtfs.steps);

//   std::unordered_set<StopId> bart_stops =
//       GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

//   StepsAdjacencyList reduced =
//       ReduceToMinimalSystemSteps(adjacency_list, bart_stops);

//   // TODO: add assertions
//   std::cout << "BART stops count: " << bart_stops.size() << std::endl;
//   std::cout << "Reduced adjacency list size: " << reduced.adjacent.size()
//             << std::endl;
// }

TEST(ShortestPathTest, SnapToStops_BasicSnapping) {
  // Create a mapping with stop positions.
  // Stop 1: (0, 0)
  // Stop 2: (100, 0)  - intermediate, will be snapped to Stop 3 (50m away)
  // Stop 3: (150, 0)
  // Stop 4: (1000, 0) - final destination, not snapped
  DataGtfsMapping mapping;
  mapping.stop_positions = {
      StopPosition{StopId{0}, 0, 0},    // placeholder for index 0
      StopPosition{StopId{1}, 0, 0},    // Stop 1 at origin
      StopPosition{StopId{2}, 100, 0},  // Stop 2 at (100, 0)
      StopPosition{StopId{3}, 150, 0},  // Stop 3 at (150, 0) - target snap stop
      StopPosition{StopId{4}, 1000, 0},  // Stop 4 at (1000, 0)
  };

  // Snap target stops: only Stop 1 and Stop 3.
  std::unordered_set<StopId> stops = {StopId{1}, StopId{3}};

  // Path: 1 -> 2 -> 4
  std::vector<Step> path = {
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
          .origin_stop = StopId{2},
          .destination_stop = StopId{4},
          .origin_time = TimeSinceServiceStart{200},
          .destination_time = TimeSinceServiceStart{300},
          .origin_trip = TripId{2},
          .destination_trip = TripId{2},
          .is_flex = false
      }
  };

  // threshold = 100 meters, so Stop 2 (at 100m) should snap to Stop 3 (at 150m,
  // 50m away). But origin of first step and destination of last step are not
  // snapped.
  std::vector<Step> result = SnapToStops(mapping, stops, 100.0f, path);

  ASSERT_EQ(result.size(), 2);
  // First step: 1 -> 3 (destination 2 snapped to 3, origin 1 not snapped)
  EXPECT_EQ(result[0].origin_stop, StopId{1});
  EXPECT_EQ(result[0].destination_stop, StopId{3});
  // Second step: 3 -> 4 (origin 2 snapped to 3, destination 4 not snapped)
  EXPECT_EQ(result[1].origin_stop, StopId{3});
  EXPECT_EQ(result[1].destination_stop, StopId{4});
}

TEST(ShortestPathTest, SnapToStops_DropSelfLoops) {
  DataGtfsMapping mapping;
  mapping.stop_positions = {
      StopPosition{StopId{0}, 0, 0},    // placeholder for index 0
      StopPosition{StopId{1}, 0, 0},    // Stop 1
      StopPosition{StopId{2}, 10, 0},   // Stop 2, 10m from Stop 1
      StopPosition{StopId{3}, 500, 0},  // Stop 3
  };

  // Only Stop 1 and Stop 3 are snap targets.
  std::unordered_set<StopId> stops = {StopId{1}, StopId{3}};

  // Path: 1 -> 2 -> 2 -> 3
  // The middle step (2 -> 2) is an intermediate step where both origin and
  // destination will be snapped to Stop 1, creating a self-loop that should be
  // dropped.
  std::vector<Step> path = {
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
          .origin_stop = StopId{2},
          .destination_stop = StopId{2},
          .origin_time = TimeSinceServiceStart{200},
          .destination_time = TimeSinceServiceStart{250},
          .origin_trip = TripId{2},
          .destination_trip = TripId{2},
          .is_flex = false
      },
      Step{
          .origin_stop = StopId{2},
          .destination_stop = StopId{3},
          .origin_time = TimeSinceServiceStart{250},
          .destination_time = TimeSinceServiceStart{300},
          .origin_trip = TripId{3},
          .destination_trip = TripId{3},
          .is_flex = false
      }
  };

  std::vector<Step> result = SnapToStops(mapping, stops, 50.0f, path);

  // First step: 1 -> 1 (destination 2 snaps to 1) -> self-loop, dropped
  // Second step: 1 -> 1 (both snap to 1) -> self-loop, dropped
  // Third step: 1 -> 3 (origin 2 snaps to 1, destination 3 not snapped)
  ASSERT_EQ(result.size(), 1);
  EXPECT_EQ(result[0].origin_stop, StopId{1});
  EXPECT_EQ(result[0].destination_stop, StopId{3});
}

TEST(ShortestPathTest, SnapToStops_NoSnapWhenTooFar) {
  DataGtfsMapping mapping;
  mapping.stop_positions = {
      StopPosition{StopId{0}, 0, 0},
      StopPosition{StopId{1}, 0, 0},
      StopPosition{StopId{2}, 200, 0},  // 200m away from Stop 1
  };

  std::unordered_set<StopId> stops = {StopId{1}};

  std::vector<Step> path = {Step{
      .origin_stop = StopId{1},
      .destination_stop = StopId{2},
      .origin_time = TimeSinceServiceStart{100},
      .destination_time = TimeSinceServiceStart{200},
      .origin_trip = TripId{1},
      .destination_trip = TripId{1},
      .is_flex = false
  }};

  // threshold = 100m, Stop 2 is 200m away, so no snapping.
  std::vector<Step> result = SnapToStops(mapping, stops, 100.0f, path);

  ASSERT_EQ(result.size(), 1);
  EXPECT_EQ(result[0].origin_stop, StopId{1});
  EXPECT_EQ(result[0].destination_stop, StopId{2});
}

TEST(ShortestPathTest, SnapToStops_EmptyPath) {
  DataGtfsMapping mapping;
  mapping.stop_positions = {StopPosition{StopId{0}, 0, 0}};

  std::unordered_set<StopId> stops = {StopId{1}};
  std::vector<Step> path = {};

  std::vector<Step> result = SnapToStops(mapping, stops, 100.0f, path);

  EXPECT_TRUE(result.empty());
}

}  // namespace vats5