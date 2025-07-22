#include "gtfs/gtfs.h"

#include <GUnit.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
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
  EXPECT_THAT(
      gtfs.stops,
      Contains(AllOf(
          Field(&GtfsStop::stop_id, Field(&GtfsStopId::v, Eq("sunnyvale"))),
          Field(&GtfsStop::stop_name, Eq("Sunnyvale")),
          Field(&GtfsStop::stop_lat, DoubleNear(37.37893, 1e-6)),
          Field(&GtfsStop::stop_lon, DoubleNear(-122.0315, 1e-6)),
          Field(&GtfsStop::parent_station, Eq(std::nullopt)))));

  // Check trips
  EXPECT_EQ(gtfs.trips.size(), 100391);
  EXPECT_THAT(
      gtfs.trips,
      Contains(
          AllOf(Field(&GtfsTrip::route_direction_id,
                      AllOf(Field(&GtfsRouteDirectionId::route_id,
                                  Field(&GtfsRouteId::v, Eq("SR:3"))),
                            Field(&GtfsRouteDirectionId::direction_id, Eq(0)))),
                Field(&GtfsTrip::trip_id, Field(&GtfsTripId::v, Eq("SR:198"))),
                Field(&GtfsTrip::service_id,
                      Field(&GtfsServiceId::v, Eq("SR:79233"))))));

  // Check calendar
  EXPECT_EQ(gtfs.calendar.size(), 384);
  EXPECT_THAT(gtfs.calendar,
              Contains(AllOf(Field(&GtfsCalendar::service_id,
                                   Field(&GtfsServiceId::v, Eq("SR:79276"))),
                             Field(&GtfsCalendar::monday, Eq(true)),
                             Field(&GtfsCalendar::tuesday, Eq(true)),
                             Field(&GtfsCalendar::wednesday, Eq(true)),
                             Field(&GtfsCalendar::thursday, Eq(true)),
                             Field(&GtfsCalendar::friday, Eq(true)),
                             Field(&GtfsCalendar::saturday, Eq(false)),
                             Field(&GtfsCalendar::sunday, Eq(false)),
                             Field(&GtfsCalendar::start_date, Eq("20241117")),
                             Field(&GtfsCalendar::end_date, Eq("20991231")))));

  // Check stop times (excluding entries with empty arrival/departure times)
  EXPECT_EQ(gtfs.stop_times.size(), 3448431);
  EXPECT_THAT(
      gtfs.stop_times,
      Contains(AllOf(
          Field(&GtfsStopTime::trip_id, Field(&GtfsTripId::v, Eq("SR:198"))),
          Field(&GtfsStopTime::stop_id, Field(&GtfsStopId::v, Eq("80100"))),
          Field(&GtfsStopTime::stop_sequence, Eq(0)),
          Field(&GtfsStopTime::arrival_time,
                Field(&GtfsTimeSinceServiceStart::seconds, Eq(25200))),
          Field(&GtfsStopTime::departure_time,
                Field(&GtfsTimeSinceServiceStart::seconds, Eq(25200))))));

  // Check routes
  EXPECT_EQ(gtfs.routes.size(), 604);
  EXPECT_THAT(
      gtfs.routes,
      Contains(
          AllOf(Field(&GtfsRoute::route_id, Field(&GtfsRouteId::v, Eq("SR:3"))),
                Field(&GtfsRoute::route_short_name, Eq("3")),
                Field(&GtfsRoute::route_long_name, Eq("Santa Rosa Ave")))));

  // Check directions
  EXPECT_EQ(gtfs.directions.size(), 1074);
  EXPECT_THAT(
      gtfs.directions,
      Contains(
          AllOf(Field(&GtfsDirection::route_direction_id,
                      AllOf(Field(&GtfsRouteDirectionId::route_id,
                                  Field(&GtfsRouteId::v, Eq("SR:3"))),
                            Field(&GtfsRouteDirectionId::direction_id, Eq(0)))),
                Field(&GtfsDirection::direction, Eq("Loop")))));
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
      "CT:Limited", "CT:South County", "CT:Local Weekday", "CT:Express"};
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

  // Verify trip data is consistent - all trips should have corresponding stop
  // times
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
      "CT:Local Weekend"};
  std::unordered_set<std::string> weekday_only_routes = {
      "CT:Limited", "CT:South County", "CT:Local Weekday", "CT:Express"};
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
        << "Weekday-only route " << weekday_route
        << " should not be present on weekends";
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

GTEST("GtfsSave round-trip should preserve data") {
  const Gtfs& gtfs = *getGlobalGtfs();

  // Filter by a specific weekday
  GtfsDay original_gtfs_day = GtfsFilterByDate(gtfs, "20250708");

  // Check that everything is non-empty so that this test actually tests
  // something.
  EXPECT_GT(original_gtfs_day.stops.size(), 100);
  EXPECT_GT(original_gtfs_day.trips.size(), 100);
  EXPECT_GT(original_gtfs_day.stop_times.size(), 100);
  EXPECT_GT(original_gtfs_day.routes.size(), 100);
  EXPECT_GT(original_gtfs_day.directions.size(), 100);

  // Save to a temporary directory
  std::string temp_dir =
      std::filesystem::temp_directory_path() / "gtfs_test_save";
  std::filesystem::remove_all(temp_dir);  // Clean up any existing directory

  GtfsSave(original_gtfs_day, temp_dir);

  // Load back from the saved directory
  Gtfs reloaded_gtfs = GtfsLoad(temp_dir);

  // Compare the relevant fields (GtfsDay contains the same fields as Gtfs
  // except calendar)
  EXPECT_EQ(original_gtfs_day.stops.size(), reloaded_gtfs.stops.size());
  EXPECT_EQ(original_gtfs_day.trips.size(), reloaded_gtfs.trips.size());
  EXPECT_EQ(original_gtfs_day.stop_times.size(),
            reloaded_gtfs.stop_times.size());
  EXPECT_EQ(original_gtfs_day.routes.size(), reloaded_gtfs.routes.size());
  EXPECT_EQ(original_gtfs_day.directions.size(),
            reloaded_gtfs.directions.size());

  // Sort both vectors for reliable comparison (file order might differ)
  auto sort_stops = [](std::vector<GtfsStop>& stops) {
    std::sort(stops.begin(), stops.end(),
              [](const GtfsStop& a, const GtfsStop& b) {
                return a.stop_id.v < b.stop_id.v;
              });
  };

  auto sort_trips = [](std::vector<GtfsTrip>& trips) {
    std::sort(trips.begin(), trips.end(),
              [](const GtfsTrip& a, const GtfsTrip& b) {
                return a.trip_id.v < b.trip_id.v;
              });
  };

  auto sort_stop_times = [](std::vector<GtfsStopTime>& stop_times) {
    std::sort(stop_times.begin(), stop_times.end(),
              [](const GtfsStopTime& a, const GtfsStopTime& b) {
                if (a.trip_id.v != b.trip_id.v)
                  return a.trip_id.v < b.trip_id.v;
                return a.stop_sequence < b.stop_sequence;
              });
  };

  auto sort_routes = [](std::vector<GtfsRoute>& routes) {
    std::sort(routes.begin(), routes.end(),
              [](const GtfsRoute& a, const GtfsRoute& b) {
                return a.route_id.v < b.route_id.v;
              });
  };

  auto sort_directions = [](std::vector<GtfsDirection>& directions) {
    std::sort(directions.begin(), directions.end(),
              [](const GtfsDirection& a, const GtfsDirection& b) {
                if (a.route_direction_id.route_id.v !=
                    b.route_direction_id.route_id.v) {
                  return a.route_direction_id.route_id.v <
                         b.route_direction_id.route_id.v;
                }
                return a.route_direction_id.direction_id <
                       b.route_direction_id.direction_id;
              });
  };

  // Create copies for sorting
  std::vector<GtfsStop> original_stops = original_gtfs_day.stops;
  std::vector<GtfsStop> reloaded_stops = reloaded_gtfs.stops;
  sort_stops(original_stops);
  sort_stops(reloaded_stops);

  std::vector<GtfsTrip> original_trips = original_gtfs_day.trips;
  std::vector<GtfsTrip> reloaded_trips = reloaded_gtfs.trips;
  sort_trips(original_trips);
  sort_trips(reloaded_trips);

  std::vector<GtfsStopTime> original_stop_times = original_gtfs_day.stop_times;
  std::vector<GtfsStopTime> reloaded_stop_times = reloaded_gtfs.stop_times;
  sort_stop_times(original_stop_times);
  sort_stop_times(reloaded_stop_times);

  std::vector<GtfsRoute> original_routes = original_gtfs_day.routes;
  std::vector<GtfsRoute> reloaded_routes = reloaded_gtfs.routes;
  sort_routes(original_routes);
  sort_routes(reloaded_routes);

  std::vector<GtfsDirection> original_directions = original_gtfs_day.directions;
  std::vector<GtfsDirection> reloaded_directions = reloaded_gtfs.directions;
  sort_directions(original_directions);
  sort_directions(reloaded_directions);

  // Compare the sorted data
  EXPECT_EQ(original_stops, reloaded_stops);
  EXPECT_EQ(original_trips, reloaded_trips);
  EXPECT_EQ(original_stop_times, reloaded_stop_times);
  EXPECT_EQ(original_routes, reloaded_routes);
  EXPECT_EQ(original_directions, reloaded_directions);

  // Verify calendar.txt exists but is empty (just headers)
  EXPECT_EQ(reloaded_gtfs.calendar.size(), 0);

  // Clean up
  std::filesystem::remove_all(temp_dir);
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
      GtfsTrip{GtfsRouteDirectionId{route_id, 0}, trip_id, service_id}};
  EXPECT_EQ(filtered.trips, expected_trips);

  // Check routes
  std::vector<GtfsRoute> expected_routes = {GtfsRoute{route_id, "Express", ""}};
  EXPECT_EQ(filtered.routes, expected_routes);

  // Check directions
  std::vector<GtfsDirection> expected_directions = {
      GtfsDirection{GtfsRouteDirectionId{route_id, 0}, "North"}};
  EXPECT_EQ(filtered.directions, expected_directions);

  // Check stops (all expected stops with floating point tolerance)
  EXPECT_THAT(
      filtered.stops,
      UnorderedElementsAre(
          // Northbound platform stops
          AllOf(Field(&GtfsStop::stop_id, Eq(sf_4th_king_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("San Francisco Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.7764, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.395, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(caltrain_4th_king))),
          AllOf(Field(&GtfsStop::stop_id, Eq(palo_alto_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("Palo Alto Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.4434, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.165, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(palo_alto_station))),
          AllOf(Field(&GtfsStop::stop_id, Eq(mountain_view_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("Mountain View Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.3945, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.076, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(mountain_view_station))),
          AllOf(Field(&GtfsStop::stop_id, Eq(san_jose_diridon_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("San Jose Diridon Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.3292, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-121.903, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(san_jose_diridon_station))),
          AllOf(Field(&GtfsStop::stop_id, Eq(millbrae_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("Millbrae Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.5999, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.387, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(millbrae_bart))),
          AllOf(Field(&GtfsStop::stop_id, Eq(sf_22nd_street_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("22nd Street Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.7576, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.392, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(sf_22nd_street))),
          AllOf(Field(&GtfsStop::stop_id, Eq(hillsdale_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("Hillsdale Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.5426, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.302, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(hillsdale))),
          AllOf(Field(&GtfsStop::stop_id, Eq(redwood_city_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("Redwood City Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.4862, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.232, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(redwood_city))),
          AllOf(Field(&GtfsStop::stop_id, Eq(san_mateo_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("San Mateo Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.5681, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.324, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(san_mateo))),
          AllOf(Field(&GtfsStop::stop_id, Eq(south_sf_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("South San Francisco Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.6559, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.405, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(south_sf))),
          AllOf(Field(&GtfsStop::stop_id, Eq(sunnyvale_northbound)),
                Field(&GtfsStop::stop_name,
                      Eq("Sunnyvale Caltrain Station Northbound")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.3789, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.031, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(sunnyvale))),
          // Parent station stops
          AllOf(Field(&GtfsStop::stop_id, Eq(caltrain_4th_king)),
                Field(&GtfsStop::stop_name, Eq("Caltrain 4th & King")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.7765, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.395, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt))),
          AllOf(Field(&GtfsStop::stop_id, Eq(palo_alto_station)),
                Field(&GtfsStop::stop_name, Eq("Palo Alto Station")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.4432, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.164, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt))),
          AllOf(Field(&GtfsStop::stop_id, Eq(millbrae_bart)),
                Field(&GtfsStop::stop_name, Eq("Millbrae BART")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.6001, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.387, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt))),
          AllOf(Field(&GtfsStop::stop_id, Eq(mountain_view_station)),
                Field(&GtfsStop::stop_name, Eq("Mountain View Station")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.3943, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.076, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt))),
          AllOf(Field(&GtfsStop::stop_id, Eq(san_jose_diridon_station)),
                Field(&GtfsStop::stop_name, Eq("San Jose Diridon Station")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.3299, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-121.903, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt))),
          AllOf(Field(&GtfsStop::stop_id, Eq(sf_22nd_street)),
                Field(&GtfsStop::stop_name, Eq("22nd Street")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.757, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.392, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt))),
          AllOf(Field(&GtfsStop::stop_id, Eq(hillsdale)),
                Field(&GtfsStop::stop_name, Eq("Hillsdale")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.5424, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.302, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt))),
          AllOf(Field(&GtfsStop::stop_id, Eq(redwood_city)),
                Field(&GtfsStop::stop_name, Eq("Redwood City")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.4859, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.231, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt))),
          AllOf(Field(&GtfsStop::stop_id, Eq(san_mateo)),
                Field(&GtfsStop::stop_name, Eq("San Mateo")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.5682, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.324, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt))),
          AllOf(Field(&GtfsStop::stop_id, Eq(south_sf)),
                Field(&GtfsStop::stop_name,
                      Eq("South San Francisco Caltrain Station")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.6559, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.405, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt))),
          AllOf(Field(&GtfsStop::stop_id, Eq(sunnyvale)),
                Field(&GtfsStop::stop_name, Eq("Sunnyvale")),
                Field(&GtfsStop::stop_lat, DoubleNear(37.3789, 1e-4)),
                Field(&GtfsStop::stop_lon, DoubleNear(-122.031, 1e-3)),
                Field(&GtfsStop::parent_station, Eq(std::nullopt)))));

  // Check stop times
  std::vector<GtfsStopTime> expected_stop_times = {
      GtfsStopTime{trip_id, san_jose_diridon_northbound, 1,
                   ParseGtfsTime("07:22:00"), ParseGtfsTime("07:22:00")},
      GtfsStopTime{trip_id, sunnyvale_northbound, 2, ParseGtfsTime("07:32:00"),
                   ParseGtfsTime("07:32:00")},
      GtfsStopTime{trip_id, mountain_view_northbound, 3,
                   ParseGtfsTime("07:36:00"), ParseGtfsTime("07:36:00")},
      GtfsStopTime{trip_id, palo_alto_northbound, 4, ParseGtfsTime("07:43:00"),
                   ParseGtfsTime("07:43:00")},
      GtfsStopTime{trip_id, redwood_city_northbound, 5,
                   ParseGtfsTime("07:49:00"), ParseGtfsTime("07:49:00")},
      GtfsStopTime{trip_id, hillsdale_northbound, 6, ParseGtfsTime("07:56:00"),
                   ParseGtfsTime("07:56:00")},
      GtfsStopTime{trip_id, san_mateo_northbound, 7, ParseGtfsTime("07:59:00"),
                   ParseGtfsTime("07:59:00")},
      GtfsStopTime{trip_id, millbrae_northbound, 8, ParseGtfsTime("08:04:00"),
                   ParseGtfsTime("08:04:00")},
      GtfsStopTime{trip_id, south_sf_northbound, 9, ParseGtfsTime("08:09:00"),
                   ParseGtfsTime("08:09:00")},
      GtfsStopTime{trip_id, sf_22nd_street_northbound, 10,
                   ParseGtfsTime("08:16:00"), ParseGtfsTime("08:16:00")},
      GtfsStopTime{trip_id, sf_4th_king_northbound, 11,
                   ParseGtfsTime("08:22:00"), ParseGtfsTime("08:22:00")}};
  EXPECT_EQ(filtered.stop_times, expected_stop_times);
}

// Helper function to create stop matcher
auto StopMatcher(const std::string& stop_id, const std::string& stop_name) {
  return AllOf(Field(&GtfsStop::stop_id, Field(&GtfsStopId::v, Eq(stop_id))),
               Field(&GtfsStop::stop_name, Eq(stop_name)));
}

GTEST("GtfsNormalizeStops should replace child stops with parents") {
  // Load the RG_20250718_BA data as requested
  std::string gtfs_directory_path = "../data/RG_20250718_BA";
  GtfsDay gtfs_day = GtfsLoadDay(gtfs_directory_path);

  GtfsDay normalized = GtfsNormalizeStops(gtfs_day);
  EXPECT_EQ(normalized.trips.size(), gtfs_day.trips.size());
  EXPECT_EQ(normalized.routes.size(), gtfs_day.routes.size());
  EXPECT_EQ(normalized.directions.size(), gtfs_day.directions.size());
  EXPECT_EQ(normalized.stop_times.size(), gtfs_day.stop_times.size());

  EXPECT_THAT(
      normalized.stops,
      UnorderedElementsAre(
          StopMatcher("mtc:powell", "Powell"),
          StopMatcher("mtc:fruitvale", "Fruitvale"),
          StopMatcher("mtc:warm-springs-south-fremont-bart",
                      "Warm Springs South Fremont BART"),
          StopMatcher("mtc:el-cerrito-del-norte-bart",
                      "El Cerrito Del Norte BART"),
          StopMatcher("mtc:walnut-creek-bart", "Walnut Creek BART"),
          StopMatcher("mtc:richmond-bart-amtrak", "Richmond BART/Amtrak"),
          StopMatcher("mtc:macarthur-bart", "MacArthur BART"),
          StopMatcher("mtc:oak", "OAK"),
          StopMatcher("mtc:millbrae-bart", "Millbrae BART"),
          StopMatcher("mtc:pleasant-hill-bart", "Pleasant Hill BART"),
          StopMatcher("mtc:daly-city-bart", "Daly City BART"),
          StopMatcher("mtc:oakland-coliseum-bart", "Oakland Coliseum BART"),
          StopMatcher("mtc:12th-st-oakland-city-center-bart",
                      "12th St Oakland City Center BART"),
          StopMatcher("mtc:19th-st-oakland-bart", "19TH St Oakland BART"),
          StopMatcher("mtc:embarcadero-bart", "Embarcadero BART"),
          StopMatcher("mtc:civic-center-bart", "Civic Center BART"),
          StopMatcher("mtc:montgomery-bart", "Montgomery BART"),
          StopMatcher("mtc:sfo", "SFO"),
          StopMatcher("mtc:union-city-bart", "Union City BART"),
          StopMatcher("mtc:dublin-pleasanton-bart", "Dublin / Pleasanton BART"),
          StopMatcher("mtc:great-mall-milpitas-bart",
                      "Great Mall/Milpitas BART"),
          StopMatcher("901509", "16th Street / Mission"),
          StopMatcher("901609", "24th Street / Mission"),
          StopMatcher("908309", "Antioch"), StopMatcher("904109", "Ashby"),
          StopMatcher("901809", "Balboa Park"),
          StopMatcher("902509", "Bay Fair"),
          StopMatcher("909509", "Berryessa / North San Jose"),
          StopMatcher("905109", "Castro Valley"),
          StopMatcher("906109", "Colma"), StopMatcher("903609", "Concord"),
          StopMatcher("904209", "Downtown Berkeley"),
          StopMatcher("904409", "El Cerrito Plaza"),
          StopMatcher("902909", "Fremont"), StopMatcher("901709", "Glen Park"),
          StopMatcher("902609", "Hayward"), StopMatcher("903309", "Lafayette"),
          StopMatcher("902109", "Lake Merritt"),
          StopMatcher("904309", "North Berkeley"),
          StopMatcher("903709", "North Concord / Martinez"),
          StopMatcher("903209", "Orinda"),
          StopMatcher("903809", "Pittsburg / Bay Point"),
          StopMatcher("908209", "Pittsburg Center"),
          StopMatcher("903109", "Rockridge"),
          StopMatcher("906309", "San Bruno"),
          StopMatcher("902409", "San Leandro"),
          StopMatcher("902709", "South Hayward"),
          StopMatcher("906209", "South San Francisco"),
          StopMatcher("905209", "West Dublin / Pleasanton"),
          StopMatcher("901109", "West Oakland")));

  // Verify that all stop_times refer to stops that have been kept
  std::unordered_set<std::string> normalized_stop_ids;
  for (const auto& stop : normalized.stops) {
    normalized_stop_ids.insert(stop.stop_id.v);
  }

  for (const auto& stop_time : normalized.stop_times) {
    EXPECT_TRUE(normalized_stop_ids.count(stop_time.stop_id.v))
        << "Stop time refers to stop " << stop_time.stop_id.v
        << " which is not in the normalized stops list";
  }
}
