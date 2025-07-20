#include <gtest/gtest.h>
#include "solver/data.h"
#include "gtfs/gtfs.h"
#include <unordered_set>

namespace vats5 {

TEST(DataTest, GetStepsFromGtfs) {
    // Load pre-filtered GTFS data (contains all CT: trips from 20250718)
    std::string gtfs_directory_path = "../data/RG_20250718_CT";
    Gtfs gtfs = GtfsLoad(gtfs_directory_path);
    
    // Convert Gtfs to GtfsDay for GetStepsFromGtfs (data is already filtered)
    GtfsDay gtfs_day;
    gtfs_day.stops = gtfs.stops;
    gtfs_day.trips = gtfs.trips;
    gtfs_day.stop_times = gtfs.stop_times;
    gtfs_day.routes = gtfs.routes;
    gtfs_day.directions = gtfs.directions;
    
    // Filter to just CT:507 for this specific test
    const GtfsTripId trip_id{"CT:507"};
    std::unordered_set<GtfsTripId> trips_set = {trip_id};
    GtfsDay filtered = GtfsFilterByTrips(gtfs_day, trips_set);
    
    StepsFromGtfs result = GetStepsFromGtfs(filtered);
    
    // Trip CT:507 has 11 stops, so there should be 10 steps between them
    EXPECT_EQ(result.steps.size(), 10);
    
    // Check that mappings are populated
    EXPECT_EQ(result.mapping.gtfs_stop_id_to_stop_id.size(), 11);  // 11 unique stops
    EXPECT_EQ(result.mapping.stop_id_to_gtfs_stop_id.size(), 11);
    EXPECT_EQ(result.mapping.gtfs_trip_id_to_trip_id.size(), 1);   // 1 unique trip
    EXPECT_EQ(result.mapping.trip_id_to_gtfs_trip_id.size(), 1);
    
    // Get the mapped IDs for constructing expected steps
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
    
    // Look up the mapped stop IDs
    StopId san_jose_diridon_id = result.mapping.gtfs_stop_id_to_stop_id.at(san_jose_diridon_northbound);
    StopId sunnyvale_id = result.mapping.gtfs_stop_id_to_stop_id.at(sunnyvale_northbound);
    StopId mountain_view_id = result.mapping.gtfs_stop_id_to_stop_id.at(mountain_view_northbound);
    StopId palo_alto_id = result.mapping.gtfs_stop_id_to_stop_id.at(palo_alto_northbound);
    StopId redwood_city_id = result.mapping.gtfs_stop_id_to_stop_id.at(redwood_city_northbound);
    StopId hillsdale_id = result.mapping.gtfs_stop_id_to_stop_id.at(hillsdale_northbound);
    StopId san_mateo_id = result.mapping.gtfs_stop_id_to_stop_id.at(san_mateo_northbound);
    StopId millbrae_id = result.mapping.gtfs_stop_id_to_stop_id.at(millbrae_northbound);
    StopId south_sf_id = result.mapping.gtfs_stop_id_to_stop_id.at(south_sf_northbound);
    StopId sf_22nd_street_id = result.mapping.gtfs_stop_id_to_stop_id.at(sf_22nd_street_northbound);
    StopId sf_4th_king_id = result.mapping.gtfs_stop_id_to_stop_id.at(sf_4th_king_northbound);
    
    // Look up the mapped trip ID
    TripId mapped_trip_id = result.mapping.gtfs_trip_id_to_trip_id.at(trip_id);
    
    // Construct expected steps with hardcoded times from gtfs_test.cpp
    std::vector<Step> expected_steps = {
        // San Jose Diridon (07:22:00) -> Sunnyvale (07:32:00)
        Step{san_jose_diridon_id, sunnyvale_id, 
             TimeSinceServiceStart{ParseGtfsTime("07:22:00").seconds}, 
             TimeSinceServiceStart{ParseGtfsTime("07:32:00").seconds}, 
             mapped_trip_id, mapped_trip_id},
        
        // Sunnyvale (07:32:00) -> Mountain View (07:36:00)
        Step{sunnyvale_id, mountain_view_id, 
             TimeSinceServiceStart{ParseGtfsTime("07:32:00").seconds}, 
             TimeSinceServiceStart{ParseGtfsTime("07:36:00").seconds}, 
             mapped_trip_id, mapped_trip_id},
        
        // Mountain View (07:36:00) -> Palo Alto (07:43:00)
        Step{mountain_view_id, palo_alto_id, 
             TimeSinceServiceStart{ParseGtfsTime("07:36:00").seconds}, 
             TimeSinceServiceStart{ParseGtfsTime("07:43:00").seconds}, 
             mapped_trip_id, mapped_trip_id},
        
        // Palo Alto (07:43:00) -> Redwood City (07:49:00)
        Step{palo_alto_id, redwood_city_id, 
             TimeSinceServiceStart{ParseGtfsTime("07:43:00").seconds}, 
             TimeSinceServiceStart{ParseGtfsTime("07:49:00").seconds}, 
             mapped_trip_id, mapped_trip_id},
        
        // Redwood City (07:49:00) -> Hillsdale (07:56:00)
        Step{redwood_city_id, hillsdale_id, 
             TimeSinceServiceStart{ParseGtfsTime("07:49:00").seconds}, 
             TimeSinceServiceStart{ParseGtfsTime("07:56:00").seconds}, 
             mapped_trip_id, mapped_trip_id},
        
        // Hillsdale (07:56:00) -> San Mateo (07:59:00)
        Step{hillsdale_id, san_mateo_id, 
             TimeSinceServiceStart{ParseGtfsTime("07:56:00").seconds}, 
             TimeSinceServiceStart{ParseGtfsTime("07:59:00").seconds}, 
             mapped_trip_id, mapped_trip_id},
        
        // San Mateo (07:59:00) -> Millbrae (08:04:00)
        Step{san_mateo_id, millbrae_id, 
             TimeSinceServiceStart{ParseGtfsTime("07:59:00").seconds}, 
             TimeSinceServiceStart{ParseGtfsTime("08:04:00").seconds}, 
             mapped_trip_id, mapped_trip_id},
        
        // Millbrae (08:04:00) -> South SF (08:09:00)
        Step{millbrae_id, south_sf_id, 
             TimeSinceServiceStart{ParseGtfsTime("08:04:00").seconds}, 
             TimeSinceServiceStart{ParseGtfsTime("08:09:00").seconds}, 
             mapped_trip_id, mapped_trip_id},
        
        // South SF (08:09:00) -> 22nd Street (08:16:00)
        Step{south_sf_id, sf_22nd_street_id, 
             TimeSinceServiceStart{ParseGtfsTime("08:09:00").seconds}, 
             TimeSinceServiceStart{ParseGtfsTime("08:16:00").seconds}, 
             mapped_trip_id, mapped_trip_id},
        
        // 22nd Street (08:16:00) -> 4th & King (08:22:00)
        Step{sf_22nd_street_id, sf_4th_king_id, 
             TimeSinceServiceStart{ParseGtfsTime("08:16:00").seconds}, 
             TimeSinceServiceStart{ParseGtfsTime("08:22:00").seconds}, 
             mapped_trip_id, mapped_trip_id}
    };
    
    // Assert that the actual steps match the expected steps
    EXPECT_EQ(result.steps, expected_steps);
}

}  // namespace vats5