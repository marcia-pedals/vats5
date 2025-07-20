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
    std::sort(ab.begin(), ab.end(), [](const Step& a, const Step& b) {
        return a.origin_time.seconds < b.origin_time.seconds;
    });
    std::sort(bc.begin(), bc.end(), [](const Step& a, const Step& b) {
        return a.origin_time.seconds < b.origin_time.seconds;
    });
    
    // Call MergeSteps
    std::vector<Step> merged_steps = MergeSteps(ab, bc);
    
    // No expectations for now, as requested
}

TEST(StepMergeTest, RapidCheckExampleTest) {
    rc::check("sorted vectors remain sorted after CheckSortedAndMinimal", [](std::vector<int> values) {
        std::sort(values.begin(), values.end());
        
        // Convert to Step vector for testing CheckSortedAndMinimal
        std::vector<Step> steps;
        for (size_t i = 0; i < values.size(); ++i) {
            steps.push_back({
                StopId{1}, 
                StopId{2}, 
                TimeSinceServiceStart{values[i]}, 
                TimeSinceServiceStart{values[i] + 100}, 
                TripId{static_cast<int>(i)}, 
                TripId{static_cast<int>(i)}
            });
        }
        
        // If we have duplicate times, the function should return false
        // Otherwise it should return true for properly sorted steps
        bool has_duplicates = false;
        for (size_t i = 1; i < values.size(); ++i) {
            if (values[i] == values[i-1]) {
                has_duplicates = true;
                break;
            }
        }
        
        RC_ASSERT(CheckSortedAndMinimal(steps) == !has_duplicates);
    });
}

}  // namespace vats5