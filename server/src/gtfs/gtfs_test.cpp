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
