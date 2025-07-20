#include <gtest/gtest.h>
#include <algorithm>
#include <rapidcheck.h>
#include "solver/step_merge.h"
#include "gtfs/gtfs.h"

namespace vats5 {

static Gtfs* getGlobalGtfs() {
    static Gtfs* gtfs = nullptr;
    if (!gtfs) {
        std::string gtfs_directory_path = "../data/RG";
        gtfs = new Gtfs(GtfsLoad(gtfs_directory_path));
    }
    return gtfs;
}

TEST(StepMergeTest, CheckSortedAndMinimalEmptyVector) {
    std::vector<Step> empty_steps;
    EXPECT_TRUE(CheckSortedAndMinimal(empty_steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalSingleStep) {
    std::vector<Step> steps = {
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}, TripId{1}}
    };
    EXPECT_TRUE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalSortedByOriginTime) {
    std::vector<Step> steps = {
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}, TripId{1}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{250}, TripId{2}, TripId{2}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{200}, TimeSinceServiceStart{300}, TripId{3}, TripId{3}}
    };
    EXPECT_TRUE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalNotSortedByOriginTime) {
    std::vector<Step> steps = {
        {StopId{1}, StopId{2}, TimeSinceServiceStart{200}, TimeSinceServiceStart{250}, TripId{1}, TripId{1}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{2}, TripId{2}}
    };
    EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalNotSortedByDestinationTime) {
    std::vector<Step> steps = {
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{1}, TripId{1}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{200}, TripId{2}, TripId{2}}
    };
    EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalEqualOriginTimes) {
    std::vector<Step> steps = {
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}, TripId{1}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{250}, TripId{2}, TripId{2}}
    };
    EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalEqualDestinationTimes) {
    std::vector<Step> steps = {
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}, TripId{1}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{200}, TripId{2}, TripId{2}}
    };
    EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, MergeStepsTest) {
    const Gtfs& gtfs = *getGlobalGtfs();
    
    // Test with a weekday (Tuesday, July 8, 2025)
    GtfsDay weekday_gtfs = GtfsFilterByDate(gtfs, "20250708");
    
    // Get steps from GTFS
    StepsFromGtfs result = GetStepsFromGtfs(weekday_gtfs);
    
    // Define the three stops
    const GtfsStopId san_jose_diridon_northbound{"70261"};
    const GtfsStopId sunnyvale_northbound{"70221"};
    const GtfsStopId mountain_view_northbound{"70211"};
    
    // Get mapped stop IDs
    StopId stop_a = result.mapping.gtfs_stop_id_to_stop_id.at(san_jose_diridon_northbound);
    StopId stop_b = result.mapping.gtfs_stop_id_to_stop_id.at(sunnyvale_northbound);
    StopId stop_c = result.mapping.gtfs_stop_id_to_stop_id.at(mountain_view_northbound);
    
    // Construct ab vector (steps from A to B)
    std::vector<Step> ab;
    for (const auto& step : result.steps) {
        if (step.origin_stop == stop_a && step.destination_stop == stop_b) {
            ab.push_back(step);
        }
    }
    
    // Construct bc vector (steps from B to C)
    std::vector<Step> bc;
    for (const auto& step : result.steps) {
        if (step.origin_stop == stop_b && step.destination_stop == stop_c) {
            bc.push_back(step);
        }
    }
    
    // Sort by origin_time ascending
    SortByOriginAndDestinationTime(ab);
    SortByOriginAndDestinationTime(bc);
    
    // Call MergeSteps
    std::vector<Step> merged_steps = MergeSteps(ab, bc);
    
    // No expectations for now, as requested
}

TEST(StepMergeTest, SortByOriginAndDestinationTimeWithSecondarySort) {
    // Test case with same origin times but different destination times
    std::vector<Step> steps = {
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{1}, TripId{1}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{2}, TripId{2}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{400}, TripId{3}, TripId{3}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{250}, TripId{4}, TripId{4}}
    };
    
    SortByOriginAndDestinationTime(steps);
    
    // Verify primary sort by origin_time ascending
    for (size_t i = 1; i < steps.size(); ++i) {
        EXPECT_LE(steps[i-1].origin_time.seconds, steps[i].origin_time.seconds);
    }
    
    // Verify secondary sort by destination_time descending when origin times are equal
    EXPECT_EQ(steps[0].origin_time.seconds, 100);
    EXPECT_EQ(steps[0].destination_time.seconds, 400); // highest destination time first
    EXPECT_EQ(steps[1].origin_time.seconds, 100);
    EXPECT_EQ(steps[1].destination_time.seconds, 300);
    EXPECT_EQ(steps[2].origin_time.seconds, 100);
    EXPECT_EQ(steps[2].destination_time.seconds, 200); // lowest destination time last
    EXPECT_EQ(steps[3].origin_time.seconds, 150);
}

TEST(StepMergeTest, MakeMinimalCoverSimpleTest) {
    // Simple test case: step 2 dominates step 1 (departs later, arrives earlier)
    std::vector<Step> steps = {
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{1}, TripId{1}}, // dominated
        {StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{250}, TripId{2}, TripId{2}}  // dominates step 1
    };
    
    MakeMinimalCover(steps);
    
    EXPECT_EQ(steps.size(), 1);
    EXPECT_EQ(steps[0].origin_time.seconds, 150);
    EXPECT_EQ(steps[0].destination_time.seconds, 250);
}

TEST(StepMergeTest, MakeMinimalCoverTest) {
    // Test case where only the last step (earliest arrival) should remain
    std::vector<Step> steps = {
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{1}, TripId{1}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{120}, TimeSinceServiceStart{350}, TripId{2}, TripId{2}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{250}, TripId{3}, TripId{3}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{180}, TimeSinceServiceStart{280}, TripId{4}, TripId{4}},
        {StopId{1}, StopId{2}, TimeSinceServiceStart{200}, TimeSinceServiceStart{240}, TripId{5}, TripId{5}}
    };
    
    MakeMinimalCover(steps);
    
    // Only the last step should remain (arrives earliest at 240)
    EXPECT_EQ(steps.size(), 1);
    EXPECT_EQ(steps[0].origin_time.seconds, 200);
    EXPECT_EQ(steps[0].destination_time.seconds, 240);
}

TEST(StepMergeTest, MakeMinimalCoverEmptyAndSingle) {
    // Empty vector
    std::vector<Step> empty_steps;
    MakeMinimalCover(empty_steps);
    EXPECT_TRUE(empty_steps.empty());
    
    // Single step
    std::vector<Step> single_step = {
        {StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}, TripId{1}}
    };
    MakeMinimalCover(single_step);
    EXPECT_EQ(single_step.size(), 1);
}

TEST(StepMergeTest, RapidCheckMakeMinimalCoverTest) {
    rc::check("MakeMinimalCover satisfies all 3 properties", [](std::vector<int> origin_times, std::vector<int> destination_times) {
        RC_PRE(origin_times.size() == destination_times.size());
        RC_PRE(!origin_times.empty());
        
        // Create Step vector with random times
        std::vector<Step> original_steps;
        for (size_t i = 0; i < origin_times.size(); ++i) {
            original_steps.push_back({
                StopId{1}, 
                StopId{2}, 
                TimeSinceServiceStart{origin_times[i]}, 
                TimeSinceServiceStart{destination_times[i]}, 
                TripId{static_cast<int>(i)}, 
                TripId{static_cast<int>(i)}
            });
        }
        
        // Sort as precondition
        SortByOriginAndDestinationTime(original_steps);
        
        // Make a copy for the minimal cover
        std::vector<Step> minimal_cover = original_steps;
        MakeMinimalCover(minimal_cover);
        
        // Property 1: minimal cover is a subset of original steps
        for (const auto& step : minimal_cover) {
            bool found = false;
            for (const auto& orig_step : original_steps) {
                if (step.origin_time.seconds == orig_step.origin_time.seconds &&
                    step.destination_time.seconds == orig_step.destination_time.seconds &&
                    step.origin_trip.v == orig_step.origin_trip.v) {
                    found = true;
                    break;
                }
            }
            RC_ASSERT(found);
        }
        
        // Property 2: satisfies CheckSortedAndMinimal
        RC_ASSERT(CheckSortedAndMinimal(minimal_cover));
        
        // Property 3: for any original step, there is a step in minimal cover with
        // origin_time no later and destination_time no later
        for (const auto& orig_step : original_steps) {
            bool dominated_or_kept = false;
            for (const auto& cover_step : minimal_cover) {
                if (cover_step.origin_time.seconds <= orig_step.origin_time.seconds &&
                    cover_step.destination_time.seconds <= orig_step.destination_time.seconds) {
                    dominated_or_kept = true;
                    break;
                }
            }
            RC_ASSERT(dominated_or_kept);
        }
    });
}

TEST(StepMergeTest, RapidCheckSortByOriginAndDestinationTimeTest) {
    rc::check("SortByOriginAndDestinationTime sorts vector by origin_time ascending", [](std::vector<int> time_values) {
        // Create Step vector with random origin times
        std::vector<Step> steps;
        for (size_t i = 0; i < time_values.size(); ++i) {
            steps.push_back({
                StopId{1}, 
                StopId{2}, 
                TimeSinceServiceStart{time_values[i]}, 
                TimeSinceServiceStart{time_values[i] + 100}, 
                TripId{static_cast<int>(i)}, 
                TripId{static_cast<int>(i)}
            });
        }
        
        // Sort using our function
        SortByOriginAndDestinationTime(steps);
        
        // Verify it's sorted by origin_time
        for (size_t i = 1; i < steps.size(); ++i) {
            RC_ASSERT(steps[i-1].origin_time.seconds <= steps[i].origin_time.seconds);
        }
    });
}

}  // namespace vats5