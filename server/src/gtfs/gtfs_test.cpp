#include "gtfs.h"

#include <GUnit.h>

using namespace vats5;
using ::testing::AllOf;
using ::testing::Contains;
using ::testing::DoubleNear;
using ::testing::Eq;
using ::testing::Field;

GTEST("GtfsLoad should work") {
  std::string gtfs_directory_path = "../data/RG";
  Gtfs gtfs = GtfsLoad(gtfs_directory_path);
  
  // Check stops
  EXPECT_EQ(gtfs.stops.size(), 21011);
  EXPECT_THAT(gtfs.stops, Contains(AllOf(
    Field(&Stop::stop_id, Field(&StopId::v, Eq("sunnyvale"))),
    Field(&Stop::stop_name, Eq("Sunnyvale")),
    Field(&Stop::stop_lat, DoubleNear(37.37893, 1e-6)),
    Field(&Stop::stop_lon, DoubleNear(-122.0315, 1e-6)),
    Field(&Stop::parent_station, Eq(std::nullopt))
  )));

  // Check trips
  EXPECT_EQ(gtfs.trips.size(), 100391);
  EXPECT_THAT(gtfs.trips, Contains(AllOf(
    Field(&Trip::route_direction_id, AllOf(
      Field(&RouteDirectionId::route_id, Field(&RouteId::v, Eq("SR:3"))),
      Field(&RouteDirectionId::direction_id, Eq(0))
    )),
    Field(&Trip::trip_id, Field(&TripId::v, Eq("SR:198"))),
    Field(&Trip::service_id, Field(&ServiceId::v, Eq("SR:79233")))
  )));

  // Check calendar
  EXPECT_EQ(gtfs.calendar.size(), 384);
  EXPECT_THAT(gtfs.calendar, Contains(AllOf(
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

  // Check stop times
  EXPECT_EQ(gtfs.stop_times.size(), 3489224);
  EXPECT_THAT(gtfs.stop_times, Contains(AllOf(
    Field(&StopTime::trip_id, Field(&TripId::v, Eq("SR:198"))),
    Field(&StopTime::stop_id, Field(&StopId::v, Eq("80100"))),
    Field(&StopTime::stop_sequence, Eq(0)),
    Field(&StopTime::arrival_time, Eq("07:00:00")),
    Field(&StopTime::departure_time, Eq("07:00:00"))
  )));

  // Check routes
  EXPECT_EQ(gtfs.routes.size(), 604);
  EXPECT_THAT(gtfs.routes, Contains(AllOf(
    Field(&Route::route_id, Field(&RouteId::v, Eq("SR:3"))),
    Field(&Route::route_short_name, Eq("3")),
    Field(&Route::route_long_name, Eq("Santa Rosa Ave"))
  )));

  // Check directions
  EXPECT_EQ(gtfs.directions.size(), 1074);
  EXPECT_THAT(gtfs.directions, Contains(AllOf(
    Field(&Direction::route_direction_id, AllOf(
      Field(&RouteDirectionId::route_id, Field(&RouteId::v, Eq("SR:3"))),
      Field(&RouteDirectionId::direction_id, Eq(0))
    )),
    Field(&Direction::direction, Eq("Loop"))
  )));
}
