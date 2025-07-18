#include "gtfs/gtfs.h"

#include <GUnit.h>
#include <unordered_set>

using namespace vats5;
using ::testing::AllOf;
using ::testing::Contains;
using ::testing::DoubleNear;
using ::testing::Eq;
using ::testing::Field;

// Load GTFS data once for all tests
static Gtfs* getGlobalGtfs() {
  static Gtfs* gtfs = nullptr;
  if (!gtfs) {
    std::string gtfs_directory_path = "../data/RG";
    gtfs = new Gtfs(GtfsLoad(gtfs_directory_path));
  }
  return gtfs;
}

GTEST("GtfsLoad should work") {
  const Gtfs& gtfs = *getGlobalGtfs();
  
  // Check stops
  EXPECT_EQ(gtfs.stops.size(), 21011);
  EXPECT_THAT(gtfs.stops, Contains(AllOf(
    Field(&GtfsStop::stop_id, Field(&GtfsStopId::v, Eq("sunnyvale"))),
    Field(&GtfsStop::stop_name, Eq("Sunnyvale")),
    Field(&GtfsStop::stop_lat, DoubleNear(37.37893, 1e-6)),
    Field(&GtfsStop::stop_lon, DoubleNear(-122.0315, 1e-6)),
    Field(&GtfsStop::parent_station, Eq(std::nullopt))
  )));

  // Check trips
  EXPECT_EQ(gtfs.trips.size(), 100391);
  EXPECT_THAT(gtfs.trips, Contains(AllOf(
    Field(&GtfsTrip::route_direction_id, AllOf(
      Field(&GtfsRouteDirectionId::route_id, Field(&GtfsRouteId::v, Eq("SR:3"))),
      Field(&GtfsRouteDirectionId::direction_id, Eq(0))
    )),
    Field(&GtfsTrip::trip_id, Field(&GtfsTripId::v, Eq("SR:198"))),
    Field(&GtfsTrip::service_id, Field(&GtfsServiceId::v, Eq("SR:79233")))
  )));

  // Check calendar
  EXPECT_EQ(gtfs.calendar.size(), 384);
  EXPECT_THAT(gtfs.calendar, Contains(AllOf(
    Field(&GtfsCalendar::service_id, Field(&GtfsServiceId::v, Eq("SR:79276"))),
    Field(&GtfsCalendar::monday, Eq(true)),
    Field(&GtfsCalendar::tuesday, Eq(true)),
    Field(&GtfsCalendar::wednesday, Eq(true)),
    Field(&GtfsCalendar::thursday, Eq(true)),
    Field(&GtfsCalendar::friday, Eq(true)),
    Field(&GtfsCalendar::saturday, Eq(false)),
    Field(&GtfsCalendar::sunday, Eq(false)),
    Field(&GtfsCalendar::start_date, Eq("20241117")),
    Field(&GtfsCalendar::end_date, Eq("20991231"))
  )));

  // Check stop times (excluding entries with empty arrival/departure times)
  EXPECT_EQ(gtfs.stop_times.size(), 3448431);
  EXPECT_THAT(gtfs.stop_times, Contains(AllOf(
    Field(&GtfsStopTime::trip_id, Field(&GtfsTripId::v, Eq("SR:198"))),
    Field(&GtfsStopTime::stop_id, Field(&GtfsStopId::v, Eq("80100"))),
    Field(&GtfsStopTime::stop_sequence, Eq(0)),
    Field(&GtfsStopTime::arrival_time, Field(&GtfsTimeSinceServiceStart::seconds, Eq(25200))),
    Field(&GtfsStopTime::departure_time, Field(&GtfsTimeSinceServiceStart::seconds, Eq(25200)))
  )));

  // Check routes
  EXPECT_EQ(gtfs.routes.size(), 604);
  EXPECT_THAT(gtfs.routes, Contains(AllOf(
    Field(&GtfsRoute::route_id, Field(&GtfsRouteId::v, Eq("SR:3"))),
    Field(&GtfsRoute::route_short_name, Eq("3")),
    Field(&GtfsRoute::route_long_name, Eq("Santa Rosa Ave"))
  )));

  // Check directions
  EXPECT_EQ(gtfs.directions.size(), 1074);
  EXPECT_THAT(gtfs.directions, Contains(AllOf(
    Field(&GtfsDirection::route_direction_id, AllOf(
      Field(&GtfsRouteDirectionId::route_id, Field(&GtfsRouteId::v, Eq("SR:3"))),
      Field(&GtfsRouteDirectionId::direction_id, Eq(0))
    )),
    Field(&GtfsDirection::direction, Eq("Loop"))
  )));
}

GTEST("GtfsFilterByDate should filter for weekday") {
  const Gtfs& gtfs = *getGlobalGtfs();
  
  // Test with a weekday (Tuesday, July 8, 2025)
  GtfsDay weekday_gtfs = GtfsFilterByDate(gtfs, "20250708");
  
  // Should have some trips on a weekday
  EXPECT_GT(weekday_gtfs.trips.size(), 0);
  EXPECT_GT(weekday_gtfs.stop_times.size(), 0);
  EXPECT_GT(weekday_gtfs.stops.size(), 0);
  EXPECT_GT(weekday_gtfs.routes.size(), 0);
  EXPECT_GT(weekday_gtfs.directions.size(), 0);
  
  // Check for specific Caltrain routes
  std::unordered_set<std::string> expected_caltrain_routes = {
    "CT:Limited", "CT:South County", "CT:Local Weekday", "CT:Express"
  };
  std::unordered_set<std::string> found_caltrain_routes;
  
  for (const auto& route : weekday_gtfs.routes) {
    if (route.route_id.v.substr(0, 3) == "CT:") {
      found_caltrain_routes.insert(route.route_id.v);
    }
  }
  
  // Verify expected routes are present
  for (const auto& expected_route : expected_caltrain_routes) {
    EXPECT_TRUE(found_caltrain_routes.count(expected_route)) 
      << "Expected Caltrain route " << expected_route << " not found";
  }
  
  // Verify CT:Local Weekend is NOT present
  EXPECT_FALSE(found_caltrain_routes.count("CT:Local Weekend"))
    << "CT:Local Weekend should not be present on weekdays";
  
  // Verify trip data is consistent - all trips should have corresponding stop times
  std::unordered_set<std::string> trip_ids_from_trips;
  for (const auto& trip : weekday_gtfs.trips) {
    trip_ids_from_trips.insert(trip.trip_id.v);
  }
  
  for (const auto& stop_time : weekday_gtfs.stop_times) {
    EXPECT_TRUE(trip_ids_from_trips.count(stop_time.trip_id.v));
  }
}

GTEST("GtfsFilterByDate should filter for weekend") {
  const Gtfs& gtfs = *getGlobalGtfs();
  
  // Test with a weekend day (Saturday, July 12, 2025)
  GtfsDay weekend_gtfs = GtfsFilterByDate(gtfs, "20250712");
  
  // Should have some trips on weekend (though likely fewer than weekday)
  EXPECT_GT(weekend_gtfs.trips.size(), 0);
  EXPECT_GT(weekend_gtfs.stop_times.size(), 0);
  EXPECT_GT(weekend_gtfs.stops.size(), 0);
  EXPECT_GT(weekend_gtfs.routes.size(), 0);
  EXPECT_GT(weekend_gtfs.directions.size(), 0);
  
  // Check for specific Caltrain routes on weekend
  std::unordered_set<std::string> expected_weekend_caltrain_routes = {
    "CT:Local Weekend"
  };
  std::unordered_set<std::string> weekday_only_routes = {
    "CT:Limited", "CT:South County", "CT:Local Weekday", "CT:Express"
  };
  std::unordered_set<std::string> found_caltrain_routes;
  
  for (const auto& route : weekend_gtfs.routes) {
    if (route.route_id.v.substr(0, 3) == "CT:") {
      found_caltrain_routes.insert(route.route_id.v);
    }
  }
  
  // Verify weekend routes are present
  for (const auto& expected_route : expected_weekend_caltrain_routes) {
    EXPECT_TRUE(found_caltrain_routes.count(expected_route)) 
      << "Expected weekend Caltrain route " << expected_route << " not found";
  }
  
  // Verify weekday-only routes are NOT present
  for (const auto& weekday_route : weekday_only_routes) {
    EXPECT_FALSE(found_caltrain_routes.count(weekday_route))
      << "Weekday-only route " << weekday_route << " should not be present on weekends";
  }
  
  // Verify trip data is consistent
  std::unordered_set<std::string> trip_ids_from_trips;
  for (const auto& trip : weekend_gtfs.trips) {
    trip_ids_from_trips.insert(trip.trip_id.v);
  }
  
  for (const auto& stop_time : weekend_gtfs.stop_times) {
    EXPECT_TRUE(trip_ids_from_trips.count(stop_time.trip_id.v));
  }
}

GTEST("GtfsFilterByDate should handle dates outside service period") {
  const Gtfs& gtfs = *getGlobalGtfs();
  
  // Test with a date far in the past (before any service starts)
  GtfsDay no_service_gtfs = GtfsFilterByDate(gtfs, "20200101");
  
  // Should have no trips, stop times, etc.
  EXPECT_EQ(no_service_gtfs.trips.size(), 0);
  EXPECT_EQ(no_service_gtfs.stop_times.size(), 0);
  EXPECT_EQ(no_service_gtfs.routes.size(), 0);
  EXPECT_EQ(no_service_gtfs.directions.size(), 0);
  // Stops might still be included as they're not date-dependent
}
