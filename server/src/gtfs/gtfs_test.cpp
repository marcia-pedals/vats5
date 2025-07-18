#include "gtfs/gtfs.h"

#include <GUnit.h>
#include <unordered_set>

using namespace vats5;
using ::testing::AllOf;
using ::testing::Contains;
using ::testing::DoubleNear;
using ::testing::Eq;
using ::testing::Field;
using ::testing::UnorderedElementsAre;

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

GTEST("GtfsFilterByTrips should filter by specific trip") {
  const Gtfs& gtfs = *getGlobalGtfs();
  GtfsDay gtfs_day = GtfsFilterByDate(gtfs, "20250718");
  const GtfsTripId trip_id{"CT:507"};
  const GtfsRouteId route_id{"CT:Express"};
  const GtfsServiceId service_id{"CT:72982"};
  
  // Child stop IDs (northbound platforms) in order of the trip
  const GtfsStopId san_jose_diridon_northbound{"70261"};
  const GtfsStopId sunnyvale_northbound{"70221"};
  const GtfsStopId mountain_view_northbound{"70211"};
  const GtfsStopId palo_alto_northbound{"70171"};
  const GtfsStopId redwood_city_northbound{"70141"};
  const GtfsStopId hillsdale_northbound{"70111"};
  const GtfsStopId san_mateo_northbound{"70091"};
  const GtfsStopId millbrae_northbound{"70061"};
  const GtfsStopId south_sf_northbound{"70041"};
  const GtfsStopId sf_22nd_street_northbound{"70021"};
  const GtfsStopId sf_4th_king_northbound{"70011"};
  
  // Parent station IDs
  const GtfsStopId caltrain_4th_king{"mtc:caltrain-4th-&-king"};
  const GtfsStopId palo_alto_station{"mtc:palo-alto-station"};
  const GtfsStopId millbrae_bart{"mtc:millbrae-bart"};
  const GtfsStopId mountain_view_station{"mtc:mountain-view-station"};
  const GtfsStopId san_jose_diridon_station{"mtc:san-jose-diridon-station"};
  const GtfsStopId sf_22nd_street{"22nd_street"};
  const GtfsStopId hillsdale{"hillsdale"};
  const GtfsStopId redwood_city{"redwood_city"};
  const GtfsStopId san_mateo{"san_mateo"};
  const GtfsStopId south_sf{"south_sf"};
  const GtfsStopId sunnyvale{"sunnyvale"};
  
  std::unordered_set<GtfsTripId> trips_set = {trip_id};
  GtfsDay filtered = GtfsFilterByTrips(gtfs_day, trips_set);
  
  // Check trips
  std::vector<GtfsTrip> expected_trips = {
    GtfsTrip{
      GtfsRouteDirectionId{route_id, 0},
      trip_id,
      service_id
    }
  };
  EXPECT_EQ(filtered.trips, expected_trips);
  
  // Check routes
  std::vector<GtfsRoute> expected_routes = {
    GtfsRoute{route_id, "Express", ""}
  };
  EXPECT_EQ(filtered.routes, expected_routes);
  
  // Check directions
  std::vector<GtfsDirection> expected_directions = {
    GtfsDirection{GtfsRouteDirectionId{route_id, 0}, "North"}
  };
  EXPECT_EQ(filtered.directions, expected_directions);
  
  // Check stops (all expected stops with floating point tolerance)
  EXPECT_THAT(filtered.stops, UnorderedElementsAre(
    // Northbound platform stops
    AllOf(
      Field(&GtfsStop::stop_id, Eq(sf_4th_king_northbound)),
      Field(&GtfsStop::stop_name, Eq("San Francisco Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.7764, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.395, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(caltrain_4th_king))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(palo_alto_northbound)),
      Field(&GtfsStop::stop_name, Eq("Palo Alto Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.4434, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.165, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(palo_alto_station))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(mountain_view_northbound)),
      Field(&GtfsStop::stop_name, Eq("Mountain View Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.3945, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.076, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(mountain_view_station))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(san_jose_diridon_northbound)),
      Field(&GtfsStop::stop_name, Eq("San Jose Diridon Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.3292, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-121.903, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(san_jose_diridon_station))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(millbrae_northbound)),
      Field(&GtfsStop::stop_name, Eq("Millbrae Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.5999, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.387, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(millbrae_bart))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(sf_22nd_street_northbound)),
      Field(&GtfsStop::stop_name, Eq("22nd Street Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.7576, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.392, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(sf_22nd_street))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(hillsdale_northbound)),
      Field(&GtfsStop::stop_name, Eq("Hillsdale Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.5426, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.302, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(hillsdale))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(redwood_city_northbound)),
      Field(&GtfsStop::stop_name, Eq("Redwood City Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.4862, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.232, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(redwood_city))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(san_mateo_northbound)),
      Field(&GtfsStop::stop_name, Eq("San Mateo Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.5681, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.324, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(san_mateo))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(south_sf_northbound)),
      Field(&GtfsStop::stop_name, Eq("South San Francisco Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.6559, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.405, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(south_sf))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(sunnyvale_northbound)),
      Field(&GtfsStop::stop_name, Eq("Sunnyvale Caltrain Station Northbound")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.3789, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.031, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(sunnyvale))
    ),
    // Parent station stops
    AllOf(
      Field(&GtfsStop::stop_id, Eq(caltrain_4th_king)),
      Field(&GtfsStop::stop_name, Eq("Caltrain 4th & King")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.7765, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.395, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(palo_alto_station)),
      Field(&GtfsStop::stop_name, Eq("Palo Alto Station")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.4432, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.164, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(millbrae_bart)),
      Field(&GtfsStop::stop_name, Eq("Millbrae BART")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.6001, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.387, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(mountain_view_station)),
      Field(&GtfsStop::stop_name, Eq("Mountain View Station")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.3943, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.076, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(san_jose_diridon_station)),
      Field(&GtfsStop::stop_name, Eq("San Jose Diridon Station")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.3299, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-121.903, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(sf_22nd_street)),
      Field(&GtfsStop::stop_name, Eq("22nd Street")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.757, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.392, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(hillsdale)),
      Field(&GtfsStop::stop_name, Eq("Hillsdale")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.5424, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.302, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(redwood_city)),
      Field(&GtfsStop::stop_name, Eq("Redwood City")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.4859, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.231, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(san_mateo)),
      Field(&GtfsStop::stop_name, Eq("San Mateo")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.5682, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.324, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(south_sf)),
      Field(&GtfsStop::stop_name, Eq("South San Francisco Caltrain Station")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.6559, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.405, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    ),
    AllOf(
      Field(&GtfsStop::stop_id, Eq(sunnyvale)),
      Field(&GtfsStop::stop_name, Eq("Sunnyvale")),
      Field(&GtfsStop::stop_lat, DoubleNear(37.3789, 1e-4)),
      Field(&GtfsStop::stop_lon, DoubleNear(-122.031, 1e-3)),
      Field(&GtfsStop::parent_station, Eq(std::nullopt))
    )
  ));
  
  // Check stop times
  std::vector<GtfsStopTime> expected_stop_times = {
    GtfsStopTime{trip_id, san_jose_diridon_northbound, 1, ParseGtfsTime("07:22:00"), ParseGtfsTime("07:22:00")},
    GtfsStopTime{trip_id, sunnyvale_northbound, 2, ParseGtfsTime("07:32:00"), ParseGtfsTime("07:32:00")},
    GtfsStopTime{trip_id, mountain_view_northbound, 3, ParseGtfsTime("07:36:00"), ParseGtfsTime("07:36:00")},
    GtfsStopTime{trip_id, palo_alto_northbound, 4, ParseGtfsTime("07:43:00"), ParseGtfsTime("07:43:00")},
    GtfsStopTime{trip_id, redwood_city_northbound, 5, ParseGtfsTime("07:49:00"), ParseGtfsTime("07:49:00")},
    GtfsStopTime{trip_id, hillsdale_northbound, 6, ParseGtfsTime("07:56:00"), ParseGtfsTime("07:56:00")},
    GtfsStopTime{trip_id, san_mateo_northbound, 7, ParseGtfsTime("07:59:00"), ParseGtfsTime("07:59:00")},
    GtfsStopTime{trip_id, millbrae_northbound, 8, ParseGtfsTime("08:04:00"), ParseGtfsTime("08:04:00")},
    GtfsStopTime{trip_id, south_sf_northbound, 9, ParseGtfsTime("08:09:00"), ParseGtfsTime("08:09:00")},
    GtfsStopTime{trip_id, sf_22nd_street_northbound, 10, ParseGtfsTime("08:16:00"), ParseGtfsTime("08:16:00")},
    GtfsStopTime{trip_id, sf_4th_king_northbound, 11, ParseGtfsTime("08:22:00"), ParseGtfsTime("08:22:00")}
  };
  EXPECT_EQ(filtered.stop_times, expected_stop_times);
}
