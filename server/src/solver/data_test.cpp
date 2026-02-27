#include "solver/data.h"

#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <unordered_set>
#include <variant>

#include "gtfs/gtfs.h"

using ::testing::AllOf;
using ::testing::Contains;
using ::testing::DoubleNear;
using ::testing::Eq;
using ::testing::Field;
using ::testing::UnorderedElementsAre;

namespace vats5 {

MATCHER_P3(WalkingStep, origin, destination, seconds, "") {
  return arg.origin.stop.v == origin && arg.destination.stop.v == destination &&
         arg.is_flex && (arg.destination.time.seconds > seconds - 5) &&
         (arg.destination.time.seconds < seconds + 5);
}

TEST(DataTest, GetStepsFromGtfs) {
  // Load pre-filtered GTFS data (contains all CT: trips from 20250718)
  std::string gtfs_directory_path = "../data/RG_20250718_CT";
  GtfsDay gtfs_day = GtfsNormalizeStops(GtfsLoadDay(gtfs_directory_path));

  // Filter to just CT:507 for this specific test
  const GtfsTripId trip_id{"CT:507"};
  std::unordered_set<GtfsTripId> trips_set = {trip_id};
  GtfsDay filtered = GtfsDayFilterByTrips(gtfs_day, trips_set);

  StepsFromGtfs result = GetStepsFromGtfs(filtered, GetStepsOptions{500.0});

  // Trip CT:507 has 11 stops, so there should be 10 steps between them
  EXPECT_EQ(result.steps.size(), 10);

  // Check that mappings are populated
  EXPECT_EQ(
      result.mapping.gtfs_stop_id_to_stop_id.size(),
      11
  );  // 11 unique stops
  EXPECT_EQ(result.mapping.stop_id_to_gtfs_stop_id.size(), 11);
  EXPECT_EQ(result.mapping.gtfs_trip_id_to_trip_id.size(), 1);  // 1 unique trip
  EXPECT_EQ(result.mapping.trip_id_to_trip_info.size(), 1);

  // Get the mapped IDs for constructing expected steps
  const GtfsStopId san_jose_diridon{"mtc:san-jose-diridon-station"};
  const GtfsStopId sunnyvale{"sunnyvale"};
  const GtfsStopId mountain_view{"mtc:mountain-view-station"};
  const GtfsStopId palo_alto{"mtc:palo-alto-station"};
  const GtfsStopId redwood_city{"redwood_city"};
  const GtfsStopId hillsdale{"hillsdale"};
  const GtfsStopId san_mateo{"san_mateo"};
  const GtfsStopId millbrae{"mtc:millbrae-bart"};
  const GtfsStopId south_sf{"south_sf"};
  const GtfsStopId sf_22nd_street{"22nd_street"};
  const GtfsStopId sf_4th_king{"mtc:caltrain-4th-&-king"};

  // Look up the mapped stop IDs
  StopId<> san_jose_diridon_id =
      result.mapping.gtfs_stop_id_to_stop_id.at(san_jose_diridon);
  StopId<> sunnyvale_id = result.mapping.gtfs_stop_id_to_stop_id.at(sunnyvale);
  StopId<> mountain_view_id =
      result.mapping.gtfs_stop_id_to_stop_id.at(mountain_view);
  StopId<> palo_alto_id = result.mapping.gtfs_stop_id_to_stop_id.at(palo_alto);
  StopId<> redwood_city_id =
      result.mapping.gtfs_stop_id_to_stop_id.at(redwood_city);
  StopId<> hillsdale_id = result.mapping.gtfs_stop_id_to_stop_id.at(hillsdale);
  StopId<> san_mateo_id = result.mapping.gtfs_stop_id_to_stop_id.at(san_mateo);
  StopId<> millbrae_id = result.mapping.gtfs_stop_id_to_stop_id.at(millbrae);
  StopId<> south_sf_id = result.mapping.gtfs_stop_id_to_stop_id.at(south_sf);
  StopId<> sf_22nd_street_id =
      result.mapping.gtfs_stop_id_to_stop_id.at(sf_22nd_street);
  StopId<> sf_4th_king_id =
      result.mapping.gtfs_stop_id_to_stop_id.at(sf_4th_king);

  // Look up the mapped trip ID
  TripId mapped_trip_id = result.mapping.gtfs_trip_id_to_trip_id.at(trip_id);

  // Construct expected steps with hardcoded times from gtfs_test.cpp
  std::vector<Step> expected_steps = {
      // San Jose Diridon (07:22:00) -> Sunnyvale (07:32:00)
      Step::PrimitiveScheduled(
          san_jose_diridon_id,
          sunnyvale_id,
          TimeSinceServiceStart{ParseGtfsTime("07:22:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("07:32:00").seconds},
          mapped_trip_id
      ),

      // Sunnyvale (07:32:00) -> Mountain View (07:36:00)
      Step::PrimitiveScheduled(
          sunnyvale_id,
          mountain_view_id,
          TimeSinceServiceStart{ParseGtfsTime("07:32:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("07:36:00").seconds},
          mapped_trip_id
      ),

      // Mountain View (07:36:00) -> Palo Alto (07:43:00)
      Step::PrimitiveScheduled(
          mountain_view_id,
          palo_alto_id,
          TimeSinceServiceStart{ParseGtfsTime("07:36:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("07:43:00").seconds},
          mapped_trip_id
      ),

      // Palo Alto (07:43:00) -> Redwood City (07:49:00)
      Step::PrimitiveScheduled(
          palo_alto_id,
          redwood_city_id,
          TimeSinceServiceStart{ParseGtfsTime("07:43:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("07:49:00").seconds},
          mapped_trip_id
      ),

      // Redwood City (07:49:00) -> Hillsdale (07:56:00)
      Step::PrimitiveScheduled(
          redwood_city_id,
          hillsdale_id,
          TimeSinceServiceStart{ParseGtfsTime("07:49:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("07:56:00").seconds},
          mapped_trip_id
      ),

      // Hillsdale (07:56:00) -> San Mateo (07:59:00)
      Step::PrimitiveScheduled(
          hillsdale_id,
          san_mateo_id,
          TimeSinceServiceStart{ParseGtfsTime("07:56:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("07:59:00").seconds},
          mapped_trip_id
      ),

      // San Mateo (07:59:00) -> Millbrae (08:04:00)
      Step::PrimitiveScheduled(
          san_mateo_id,
          millbrae_id,
          TimeSinceServiceStart{ParseGtfsTime("07:59:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("08:04:00").seconds},
          mapped_trip_id
      ),

      // Millbrae (08:04:00) -> South SF (08:09:00)
      Step::PrimitiveScheduled(
          millbrae_id,
          south_sf_id,
          TimeSinceServiceStart{ParseGtfsTime("08:04:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("08:09:00").seconds},
          mapped_trip_id
      ),

      // South SF (08:09:00) -> 22nd Street (08:16:00)
      Step::PrimitiveScheduled(
          south_sf_id,
          sf_22nd_street_id,
          TimeSinceServiceStart{ParseGtfsTime("08:09:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("08:16:00").seconds},
          mapped_trip_id
      ),

      // 22nd Street (08:16:00) -> 4th & King (08:22:00)
      Step::PrimitiveScheduled(
          sf_22nd_street_id,
          sf_4th_king_id,
          TimeSinceServiceStart{ParseGtfsTime("08:16:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("08:22:00").seconds},
          mapped_trip_id
      )
  };

  // Assert that the actual steps match the expected steps
  EXPECT_EQ(result.steps, expected_steps);
}

TEST(DataTest, WalkingStepsGrid3x3) {
  // Create a 3x3 grid of stops with 300m spacing
  // With 500m max walking distance, stops should connect to adjacent neighbors
  // (including diagonals) Adjacent: 300m, Diagonal: ~424m (both under 500m
  // limit)

  GtfsDay gtfs_day;

  const double BASE_LAT = 37.0;
  const double BASE_LON = -122.0;
  const double LAT_SPACING_DEGREES =
      300.0 / 111000.0;  // ~300m in degrees for latitude
  const double LON_SPACING_DEGREES =
      300.0 /
      (111000.0 * std::cos(
                      BASE_LAT * M_PI / 180.0
                  ));  // ~300m in degrees for longitude at this latitude

  // Create 3x3 stops
  int stop_id = 1;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      GtfsStop stop;
      stop.stop_id = GtfsStopId{std::to_string(stop_id++)};
      stop.stop_name =
          "Stop_" + std::to_string(row) + "_" + std::to_string(col);
      stop.stop_lat = BASE_LAT + row * LAT_SPACING_DEGREES;
      stop.stop_lon = BASE_LON + col * LON_SPACING_DEGREES;
      gtfs_day.stops.push_back(stop);
    }
  }

  // Generate steps
  StepsFromGtfs result = GetStepsFromGtfs(gtfs_day, GetStepsOptions{500.0});

  EXPECT_THAT(
      result.steps,
      UnorderedElementsAre(
          // From stop 1 (top-left): can walk to 2, 4, 5
          WalkingStep(1, 2, 300),
          WalkingStep(1, 4, 300),
          WalkingStep(1, 5, 424),

          // From stop 2 (top-center): can walk to 1, 3, 4, 5, 6
          WalkingStep(2, 1, 300),
          WalkingStep(2, 3, 300),
          WalkingStep(2, 4, 424),
          WalkingStep(2, 5, 300),
          WalkingStep(2, 6, 424),

          // From stop 3 (top-right): can walk to 2, 5, 6
          WalkingStep(3, 2, 300),
          WalkingStep(3, 5, 424),
          WalkingStep(3, 6, 300),

          // From stop 4 (middle-left): can walk to 1, 2, 5, 7, 8
          WalkingStep(4, 1, 300),
          WalkingStep(4, 2, 424),
          WalkingStep(4, 5, 300),
          WalkingStep(4, 7, 300),
          WalkingStep(4, 8, 424),

          // From stop 5 (middle-center): can walk to 1, 2, 3, 4, 6, 7, 8, 9
          WalkingStep(5, 1, 424),
          WalkingStep(5, 2, 300),
          WalkingStep(5, 3, 424),
          WalkingStep(5, 4, 300),
          WalkingStep(5, 6, 300),
          WalkingStep(5, 7, 424),
          WalkingStep(5, 8, 300),
          WalkingStep(5, 9, 424),

          // From stop 6 (middle-right): can walk to 2, 3, 5, 8, 9
          WalkingStep(6, 2, 424),
          WalkingStep(6, 3, 300),
          WalkingStep(6, 5, 300),
          WalkingStep(6, 8, 424),
          WalkingStep(6, 9, 300),

          // From stop 7 (bottom-left): can walk to 4, 5, 8
          WalkingStep(7, 4, 300),
          WalkingStep(7, 5, 424),
          WalkingStep(7, 8, 300),

          // From stop 8 (bottom-center): can walk to 4, 5, 6, 7, 9
          WalkingStep(8, 4, 424),
          WalkingStep(8, 5, 300),
          WalkingStep(8, 6, 424),
          WalkingStep(8, 7, 300),
          WalkingStep(8, 9, 300),

          // From stop 9 (bottom-right): can walk to 5, 6, 8
          WalkingStep(9, 5, 424),
          WalkingStep(9, 6, 300),
          WalkingStep(9, 8, 300)
      )
  );
}

TEST(DataTest, GetStopsForTripIdPrefix_BART) {
  std::string gtfs_directory_path = "../data/RG_20250718_BA_CT_SC";
  GtfsDay gtfs_day = GtfsNormalizeStops(GtfsLoadDay(gtfs_directory_path));

  StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(gtfs_day);

  std::unordered_set<StopId<>> bart_stops =
      GetStopsForTripIdPrefix(gtfs_day, steps_from_gtfs.mapping, "BA:");

  std::vector<std::string> bart_stop_names;
  for (const StopId<>& stop_id : bart_stops) {
    bart_stop_names.push_back(
        steps_from_gtfs.mapping.stop_id_to_stop_name.at(stop_id)
    );
  }

  EXPECT_THAT(
      bart_stop_names,
      UnorderedElementsAre(
          "Berryessa / North San Jose",
          "Great Mall/Milpitas BART",
          "Warm Springs South Fremont BART",
          "Fremont",
          "Union City BART",
          "South Hayward",
          "Hayward",
          "Millbrae BART",
          "Ashby",
          "Downtown Berkeley",
          "North Berkeley",
          "El Cerrito Plaza",
          "El Cerrito Del Norte BART",
          "Richmond BART/Amtrak",
          "OAK",
          "Antioch",
          "Pittsburg Center",
          "Pittsburg / Bay Point",
          "North Concord / Martinez",
          "Concord",
          "Pleasant Hill BART",
          "Civic Center BART",
          "Powell",
          "Montgomery BART",
          "Rockridge",
          "West Oakland",
          "Embarcadero BART",
          "Orinda",
          "Lake Merritt",
          "Oakland Coliseum BART",
          "Fruitvale",
          "San Leandro",
          "Bay Fair",
          "Castro Valley",
          "Walnut Creek BART",
          "West Dublin / Pleasanton",
          "Dublin / Pleasanton BART",
          "South San Francisco",
          "16th Street / Mission",
          "24th Street / Mission",
          "Glen Park",
          "Balboa Park",
          "Daly City BART",
          "Lafayette",
          "SFO",
          "San Bruno",
          "Colma",
          "12th St Oakland City Center BART",
          "19TH St Oakland BART",
          "MacArthur BART"
      )
  );
}

}  // namespace vats5
