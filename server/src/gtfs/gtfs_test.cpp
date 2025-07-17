#include "gtfs.h"

#include <GUnit.h>

using namespace vats5;
using ::testing::AllOf;
using ::testing::Contains;
using ::testing::DoubleNear;
using ::testing::Eq;
using ::testing::Field;

GTEST("GtfsLoadStops should work") {
  std::string stops_file_path = "../data/RG/stops.txt";
  std::vector<Stop> stops = GtfsLoadStops(stops_file_path);
  EXPECT_EQ(stops.size(), 21011);

  // Check that the Sunnyvale stop exists
  EXPECT_THAT(stops, Contains(AllOf(
    Field(&Stop::stop_id, Field(&StopId::v, Eq("sunnyvale"))),
    Field(&Stop::stop_name, Eq("Sunnyvale")),
    Field(&Stop::stop_lat, DoubleNear(37.37893, 1e-6)),
    Field(&Stop::stop_lon, DoubleNear(-122.0315, 1e-6)),
    Field(&Stop::parent_station, Eq(std::nullopt))
  )));

}

GTEST("GtfsLoadTrips should work") {
  std::string trips_file_path = "../data/RG/trips.txt";
  std::vector<Trip> trips = GtfsLoadTrips(trips_file_path);
  EXPECT_EQ(trips.size(), 100391);

  // Check that a specific trip exists
  EXPECT_THAT(trips, Contains(AllOf(
    Field(&Trip::route_direction_id, AllOf(
      Field(&RouteDirectionId::route_id, Field(&RouteId::v, Eq("SR:3"))),
      Field(&RouteDirectionId::direction_id, Eq(0))
    )),
    Field(&Trip::trip_id, Field(&TripId::v, Eq("SR:198"))),
    Field(&Trip::service_id, Field(&ServiceId::v, Eq("SR:79233")))
  )));
}

GTEST("GtfsLoadCalendar should work") {
  std::string calendar_file_path = "../data/RG/calendar.txt";
  std::vector<Calendar> calendars = GtfsLoadCalendar(calendar_file_path);
  EXPECT_EQ(calendars.size(), 384);

  // Check that a specific calendar entry exists
  EXPECT_THAT(calendars, Contains(AllOf(
    Field(&Calendar::service_id, Field(&ServiceId::v, Eq("SR:79276"))),
    Field(&Calendar::monday, Eq(true)),
    Field(&Calendar::tuesday, Eq(true)),
    Field(&Calendar::wednesday, Eq(true)),
    Field(&Calendar::thursday, Eq(true)),
    Field(&Calendar::friday, Eq(true)),
    Field(&Calendar::saturday, Eq(false)),
    Field(&Calendar::sunday, Eq(false)),
    Field(&Calendar::start_date, Eq("20241117")),
    Field(&Calendar::end_date, Eq("20991231"))
  )));
}

GTEST("GtfsLoadStopTimes should work") {
  std::string stop_times_file_path = "../data/RG/stop_times.txt";
  std::vector<StopTime> stop_times = GtfsLoadStopTimes(stop_times_file_path);
  EXPECT_EQ(stop_times.size(), 3489224);

  // Check that a specific stop time exists
  EXPECT_THAT(stop_times, Contains(AllOf(
    Field(&StopTime::trip_id, Field(&TripId::v, Eq("SR:198"))),
    Field(&StopTime::stop_id, Field(&StopId::v, Eq("80100"))),
    Field(&StopTime::stop_sequence, Eq(0)),
    Field(&StopTime::arrival_time, Eq("07:00:00")),
    Field(&StopTime::departure_time, Eq("07:00:00"))
  )));
}

GTEST("GtfsLoadRoutes should work") {
  std::string routes_file_path = "../data/RG/routes.txt";
  std::vector<Route> routes = GtfsLoadRoutes(routes_file_path);
  EXPECT_EQ(routes.size(), 604);

  // Check that a specific route exists
  EXPECT_THAT(routes, Contains(AllOf(
    Field(&Route::route_id, Field(&RouteId::v, Eq("SR:3"))),
    Field(&Route::route_short_name, Eq("3")),
    Field(&Route::route_long_name, Eq("Santa Rosa Ave"))
  )));
}

GTEST("GtfsLoadDirections should work") {
  std::string directions_file_path = "../data/RG/directions.txt";
  std::vector<Direction> directions = GtfsLoadDirections(directions_file_path);
  EXPECT_EQ(directions.size(), 1074);

  // Check that a specific direction exists
  EXPECT_THAT(directions, Contains(AllOf(
    Field(&Direction::route_direction_id, AllOf(
      Field(&RouteDirectionId::route_id, Field(&RouteId::v, Eq("SR:3"))),
      Field(&RouteDirectionId::direction_id, Eq(0))
    )),
    Field(&Direction::direction, Eq("Loop"))
  )));
}
