#include "solver/shortest_path.h"
#include "solver/data.h"
#include "gtfs/gtfs.h"

#include <gtest/gtest.h>
#include <iostream>

namespace vats5 {

TEST(ShortestPathTest, MakeAdjacencyListBasic) {
  std::vector<Step> steps = {
    Step{
      .origin_stop = StopId{1},
      .destination_stop = StopId{2},
      .origin_time = TimeSinceServiceStart{100},
      .destination_time = TimeSinceServiceStart{200},
      .origin_trip = TripId{1},
      .destination_trip = TripId{1}
    },
    Step{
      .origin_stop = StopId{1},
      .destination_stop = StopId{2},
      .origin_time = TimeSinceServiceStart{150},
      .destination_time = TimeSinceServiceStart{250},
      .origin_trip = TripId{2},
      .destination_trip = TripId{2}
    }
  };
  
  StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps);
  
  EXPECT_EQ(adjacency_list.adjacent.size(), 1);
  EXPECT_NE(adjacency_list.adjacent.find(StopId{1}), adjacency_list.adjacent.end());
  EXPECT_EQ(adjacency_list.adjacent[StopId{1}].size(), 1);
  EXPECT_EQ(adjacency_list.adjacent[StopId{1}][0].size(), 2);
}

TEST(ShortestPathTest, FindShortestPathsAtTimeWithRealData) {
  try {
    // Load GTFS data
    std::cout << "Loading GTFS data from ../data/RG_20250718_BA..." << std::endl;
    GtfsDay gtfs_day = GtfsLoadDay("../data/RG_20250718_BA");
    std::cout << "Loaded " << gtfs_day.stop_times.size() << " stop times" << std::endl;
    
    // Normalize stops to handle parent-child relationships
    std::cout << "Normalizing stops..." << std::endl;
    gtfs_day = GtfsNormalizeStops(gtfs_day);
    std::cout << "After normalization: " << gtfs_day.stop_times.size() << " stop times" << std::endl;
    
    // Convert to steps
    std::cout << "Converting GTFS data to steps..." << std::endl;
    StepsFromGtfs steps_from_gtfs = GetStepsFromGtfs(gtfs_day);
    std::cout << "Generated " << steps_from_gtfs.steps.size() << " steps" << std::endl;
    
    // Create adjacency list
    std::cout << "Creating adjacency list..." << std::endl;
    StepsAdjacencyList adjacency_list = MakeAdjacencyList(steps_from_gtfs.steps);
    std::cout << "Adjacency list has " << adjacency_list.adjacent.size() << " origin stops" << std::endl;
    
    // Find stop IDs for the query
    StopId berryessa_stop_id;
    StopId powell_stop_id;
    StopId dublin_stop_id;
    
    // Look for Berryessa stop (909509)
    auto berryessa_gtfs_id = GtfsStopId{"909509"};
    auto berryessa_it = steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.find(berryessa_gtfs_id);
    if (berryessa_it != steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.end()) {
      berryessa_stop_id = berryessa_it->second;
      std::cout << "Found Berryessa stop: GTFS ID 909509 -> Internal ID " << berryessa_stop_id.v << std::endl;
    } else {
      std::cout << "Berryessa stop 909509 not found in mapping" << std::endl;
      FAIL() << "Berryessa stop not found";
    }
    
    // Look for Powell stop
    auto powell_gtfs_id = GtfsStopId{"mtc:powell"};
    auto powell_it = steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.find(powell_gtfs_id);
    if (powell_it != steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.end()) {
      powell_stop_id = powell_it->second;
      std::cout << "Found Powell stop: GTFS ID mtc:powell -> Internal ID " << powell_stop_id.v << std::endl;
    } else {
      std::cout << "Powell stop mtc:powell not found in mapping" << std::endl;
      FAIL() << "Powell stop not found";
    }
    
    // Look for Dublin/Pleasanton stop
    auto dublin_gtfs_id = GtfsStopId{"mtc:dublin-pleasanton-bart"};
    auto dublin_it = steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.find(dublin_gtfs_id);
    if (dublin_it != steps_from_gtfs.mapping.gtfs_stop_id_to_stop_id.end()) {
      dublin_stop_id = dublin_it->second;
      std::cout << "Found Dublin/Pleasanton stop: GTFS ID mtc:dublin-pleasanton-bart -> Internal ID " << dublin_stop_id.v << std::endl;
    } else {
      std::cout << "Dublin/Pleasanton stop mtc:dublin-pleasanton-bart not found in mapping" << std::endl;
      FAIL() << "Dublin/Pleasanton stop not found";
    }
    
    // Query shortest paths at 8:00 AM (8 * 3600 = 28800 seconds)
    TimeSinceServiceStart query_time{28800};
    std::unordered_set<StopId> destinations = {powell_stop_id, dublin_stop_id};
    
    std::cout << "\nQuerying shortest paths from Berryessa (stop " << berryessa_stop_id.v 
              << ") at 8:00 AM (28800 seconds)..." << std::endl;
    
    auto shortest_paths = FindShortestPathsAtTime(
        adjacency_list, 
        query_time, 
        berryessa_stop_id, 
        destinations
    );
    
    std::cout << "\nResults:" << std::endl;
    std::cout << "Found " << shortest_paths.size() << " paths" << std::endl;
    
    for (const auto& [destination_stop, step] : shortest_paths) {
      auto gtfs_destination_id = steps_from_gtfs.mapping.stop_id_to_gtfs_stop_id[destination_stop];
      std::cout << "\nPath to " << gtfs_destination_id.v << " (internal ID " << destination_stop.v << "):" << std::endl;
      std::cout << "  " << step << std::endl;
      std::cout << "  Travel time: " << (step.destination_time.seconds - step.origin_time.seconds) << " seconds" << std::endl;
    }
    
  } catch (const std::exception& e) {
    FAIL() << "Exception: " << e.what();
  }
}

}  // namespace vats5