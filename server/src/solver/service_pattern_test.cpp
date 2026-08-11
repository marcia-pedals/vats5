#include "solver/service_pattern.h"

#include <GUnit.h>

#include <filesystem>
#include <fstream>

using namespace vats5;

namespace {

// A day with three trips of its own and one pulled in from each of the two
// service days after it.
GtfsDay MakeGtfsDay() {
  GtfsDay day;
  day.trips = {
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0},
          GtfsTripId{"T1"},
          GtfsServiceId{"weekday"},
          "North"
      },
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R2"}, 0},
          GtfsTripId{"T2"},
          GtfsServiceId{"weekday"},
          "East"
      },
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0},
          GtfsTripId{"T3"},
          GtfsServiceId{"latenight"},
          "North"
      },
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0},
          GtfsTripId{"T1:next-sd-1"},
          GtfsServiceId{"holiday"},
          "North"
      },
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R2"}, 0},
          GtfsTripId{"T2:next-sd-2"},
          GtfsServiceId{"sunday"},
          "East"
      },
  };
  return day;
}

// How many service days MakeGtfsDay() spans: the date itself and two after it.
constexpr int kNumServiceDays = 3;

// Maps the trips of MakeGtfsDay() to TripIds 1..5, plus a walking trip 6.
DataGtfsMapping MakeMapping() {
  DataGtfsMapping mapping;
  mapping.trip_id_to_trip_info[TripId{1}] = TripInfo{GtfsTripId{"T1"}};
  mapping.trip_id_to_trip_info[TripId{2}] = TripInfo{GtfsTripId{"T2"}};
  mapping.trip_id_to_trip_info[TripId{3}] = TripInfo{GtfsTripId{"T3"}};
  mapping.trip_id_to_trip_info[TripId{4}] =
      TripInfo{GtfsTripId{"T1:next-sd-1"}};
  mapping.trip_id_to_trip_info[TripId{5}] =
      TripInfo{GtfsTripId{"T2:next-sd-2"}};
  mapping.trip_id_to_trip_info[TripId{6}] =
      TripInfo{FlexTrip{StopId{1}, StopId{2}, 60}};
  return mapping;
}

// One path per given trip, all between the same two stops.
StepPathsAdjacencyList MakePaths(const std::vector<TripId>& trips) {
  std::vector<Path> paths;
  for (TripId trip : trips) {
    Step step = Step::PrimitiveScheduled(
        StopId{1},
        StopId{2},
        TimeSinceServiceStart{100},
        TimeSinceServiceStart{200},
        trip
    );
    paths.push_back(Path{step, {step}});
  }
  StepPathsAdjacencyList result;
  result.adjacent[StopId{1}] = {paths};
  return result;
}

using ServicesByDay = std::vector<std::vector<std::string>>;

ServicesByDay Values(const ActiveServices& services) {
  ServicesByDay result;
  for (const std::vector<GtfsServiceId>& day : services.by_service_day) {
    std::vector<std::string> day_values;
    for (const GtfsServiceId& id : day) {
      day_values.push_back(id.v);
    }
    result.push_back(std::move(day_values));
  }
  return result;
}

// An ActiveServices holding the given service ids, one vector per service day.
ActiveServices Services(const ServicesByDay& by_service_day) {
  ActiveServices result;
  for (const std::vector<std::string>& day : by_service_day) {
    std::vector<GtfsServiceId> day_ids;
    for (const std::string& id : day) {
      day_ids.push_back(GtfsServiceId{id});
    }
    result.by_service_day.push_back(std::move(day_ids));
  }
  return result;
}

}  // namespace

GTEST("GetAllActiveServices splits by service day and deduplicates") {
  ActiveServices services =
      GetAllActiveServices(MakeGtfsDay(), kNumServiceDays);

  EXPECT_EQ(
      Values(services),
      (ServicesByDay{{"latenight", "weekday"}, {"holiday"}, {"sunday"}})
  );
}

GTEST("GetAllActiveServices throws on a trip beyond the days it spans") {
  EXPECT_THROW(
      GetAllActiveServices(MakeGtfsDay(), /*num_service_days=*/2),
      std::runtime_error
  );
}

GTEST("GetAllActiveServices keeps an entry for a day with no services") {
  GtfsDay day = MakeGtfsDay();
  std::erase_if(day.trips, [](const GtfsTrip& trip) {
    return trip.trip_id.v == "T1:next-sd-1";
  });

  ActiveServices services = GetAllActiveServices(day, kNumServiceDays);

  EXPECT_EQ(
      Values(services),
      (ServicesByDay{{"latenight", "weekday"}, {}, {"sunday"}})
  );
}

GTEST("GetPrunedActiveServices only reports services of the given paths") {
  ActiveServices services = GetPrunedActiveServices(
      MakePaths({TripId{2}, TripId{5}}),
      MakeMapping(),
      MakeGtfsDay(),
      kNumServiceDays
  );

  EXPECT_EQ(Values(services), (ServicesByDay{{"weekday"}, {}, {"sunday"}}));
}

GTEST("GetPrunedActiveServices ignores flex trips") {
  ActiveServices services = GetPrunedActiveServices(
      MakePaths({TripId{6}}), MakeMapping(), MakeGtfsDay(), kNumServiceDays
  );

  EXPECT_EQ(Values(services), (ServicesByDay{{}, {}, {}}));
}

GTEST("GetPrunedActiveServices throws on a trip with no info") {
  EXPECT_THROW(
      GetPrunedActiveServices(
          MakePaths({TripId{99}}), MakeMapping(), MakeGtfsDay(), kNumServiceDays
      ),
      std::runtime_error
  );
}

GTEST("ActiveServicesPath replaces the extension") {
  EXPECT_EQ(
      ActiveServicesPath("/tmp/a.b/problem_state.json"),
      "/tmp/a.b/problem_state-active-services.json"
  );
  EXPECT_EQ(ActiveServicesPath("state"), "state-active-services.json");
}

GTEST("Service patterns match only on identical service sets") {
  ServicePattern solved{
      .service_date = "20250708",
      .all_services = Services({{"weekday"}, {}}),
      .pruned_services = Services({{"weekday"}, {}}),
      .duplicate_of = std::nullopt,
  };
  // An already-solved date that exited early records no pruned services.
  ServicePattern early{
      .service_date = "20250709",
      .all_services = Services({{"holiday"}, {}}),
      .pruned_services = std::nullopt,
      .duplicate_of = std::nullopt,
  };
  std::vector<ServicePattern> solved_patterns{solved, early};

  const ServicePattern* all_match =
      FindMatchingAllServices(solved_patterns, Services({{"weekday"}, {}}));
  ASSERT_TRUE(all_match != nullptr);
  EXPECT_EQ(all_match->service_date, "20250708");

  EXPECT_TRUE(
      FindMatchingAllServices(
          solved_patterns, Services({{"weekday"}, {"holiday"}})
      ) == nullptr
  );

  const ServicePattern* pruned_match =
      FindMatchingPrunedServices(solved_patterns, Services({{"weekday"}, {}}));
  ASSERT_TRUE(pruned_match != nullptr);
  EXPECT_EQ(pruned_match->service_date, "20250708");

  EXPECT_TRUE(
      FindMatchingPrunedServices(
          solved_patterns, Services({{"holiday"}, {}})
      ) == nullptr
  );
}

GTEST("Service patterns spanning different day counts never match") {
  std::vector<ServicePattern> solved_patterns{ServicePattern{
      .service_date = "20250708",
      .all_services = Services({{"weekday"}, {"weekday"}}),
      .pruned_services = Services({{"weekday"}, {"weekday"}}),
      .duplicate_of = std::nullopt,
  }};

  // Same services, but spanning a third day: a different problem.
  EXPECT_TRUE(
      FindMatchingAllServices(
          solved_patterns, Services({{"weekday"}, {"weekday"}, {}})
      ) == nullptr
  );
  EXPECT_TRUE(
      FindMatchingPrunedServices(
          solved_patterns, Services({{"weekday"}, {"weekday"}, {}})
      ) == nullptr
  );
}

GTEST("Active services are keyed by service day in JSON") {
  nlohmann::json j = Services({{"weekday"}, {"holiday"}, {}});

  EXPECT_EQ(
      j.dump(),
      R"({"cur_sd":["weekday"],"next_sd_1":["holiday"],"next_sd_2":[]})"
  );
  EXPECT_TRUE(
      j.get<ActiveServices>() == Services({{"weekday"}, {"holiday"}, {}})
  );
}

GTEST("Active services with a gap in its service days is rejected") {
  nlohmann::json j = {{"cur_sd", {"weekday"}}, {"next_sd_2", {"holiday"}}};

  EXPECT_THROW(j.get<ActiveServices>(), std::runtime_error);
}

GTEST("Service patterns round trip through JSON") {
  ServicePattern pattern{
      .service_date = "20250708",
      .all_services = Services({{"weekday"}, {"holiday"}, {"sunday"}}),
      .pruned_services = Services({{"weekday"}, {}, {}}),
      .duplicate_of = DuplicateOf{"20250707", "pruned_services"},
  };

  std::filesystem::path path =
      std::filesystem::temp_directory_path() / "service_pattern_test.json";
  WriteServicePattern(pattern, path.string());

  // LoadServicePatterns reads a list, which is how run.py feeds them back in.
  nlohmann::json list = nlohmann::json::array({nlohmann::json(pattern)});
  std::ofstream(path.string()) << list.dump();
  std::vector<ServicePattern> loaded = LoadServicePatterns(path.string());

  ASSERT_EQ(loaded.size(), 1u);
  EXPECT_EQ(loaded[0].service_date, "20250708");
  EXPECT_TRUE(loaded[0].all_services == pattern.all_services);
  ASSERT_TRUE(loaded[0].pruned_services.has_value());
  EXPECT_TRUE(*loaded[0].pruned_services == *pattern.pruned_services);
  ASSERT_TRUE(loaded[0].duplicate_of.has_value());
  EXPECT_EQ(loaded[0].duplicate_of->service_date, "20250707");
  EXPECT_EQ(loaded[0].duplicate_of->matched_on, "pruned_services");

  std::filesystem::remove(path);
}
