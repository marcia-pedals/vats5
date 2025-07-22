#include "solver/step_merge.h"

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <algorithm>
#include <unordered_set>

#include "gtfs/gtfs.h"

// Phantom type allowing us to specify fixed Origin and Destination values in
// properties.
template <int Origin, int Destination>
struct StepFromTo : public vats5::Step {};

namespace rc {

template <>
struct Arbitrary<vats5::StopId> {
  static Gen<vats5::StopId> arbitrary() {
    return gen::map(gen::inRange(1, 100), [](int v) {
      return vats5::StopId{v};
    });
  }
};

template <>
struct Arbitrary<vats5::TimeSinceServiceStart> {
  static Gen<vats5::TimeSinceServiceStart> arbitrary() {
    return gen::map(gen::inRange(-1, 3600), [](int v) {
      return vats5::TimeSinceServiceStart{v};
    });
  }
};

template <>
struct Arbitrary<vats5::TripId> {
  static Gen<vats5::TripId> arbitrary() {
    return gen::map(gen::inRange(0, 50), [](int v) {
      return vats5::TripId{v};
    });
  }
};

template <int Origin, int Destination>
struct Arbitrary<StepFromTo<Origin, Destination>> {
  static Gen<StepFromTo<Origin, Destination>> arbitrary() {
    return gen::apply(
        [](vats5::TimeSinceServiceStart origin_time, int duration_offset) {
          vats5::TimeSinceServiceStart dest_time{
              origin_time == vats5::TimeSinceServiceStart::FLEX_STEP_MARKER
                  ? duration_offset
                  : origin_time.seconds + duration_offset
          };
          return StepFromTo<Origin, Destination>{
              Origin,
              Destination,
              origin_time,
              dest_time,
              vats5::TripId{0},
              vats5::TripId{0}
          };
        },
        gen::arbitrary<vats5::TimeSinceServiceStart>(),
        gen::inRange(1, 3600)
    );
  }
};

}  // namespace rc

namespace vats5 {

TEST(StepMergeTest, CheckSortedAndMinimalEmptyVector) {
  std::vector<Step> empty_steps;
  EXPECT_TRUE(CheckSortedAndMinimal(empty_steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalSingleStep) {
  std::vector<Step> steps = {
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{200},
       TripId{1},
       TripId{1}}
  };
  EXPECT_TRUE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalSortedByOriginTime) {
  std::vector<Step> steps = {
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{200},
       TripId{1},
       TripId{1}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{150},
       TimeSinceServiceStart{250},
       TripId{2},
       TripId{2}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{200},
       TimeSinceServiceStart{300},
       TripId{3},
       TripId{3}}
  };
  EXPECT_TRUE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalNotSortedByOriginTime) {
  std::vector<Step> steps = {
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{200},
       TimeSinceServiceStart{250},
       TripId{1},
       TripId{1}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{300},
       TripId{2},
       TripId{2}}
  };
  EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalNotSortedByDestinationTime) {
  std::vector<Step> steps = {
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{300},
       TripId{1},
       TripId{1}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{150},
       TimeSinceServiceStart{200},
       TripId{2},
       TripId{2}}
  };
  EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalEqualOriginTimes) {
  std::vector<Step> steps = {
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{200},
       TripId{1},
       TripId{1}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{250},
       TripId{2},
       TripId{2}}
  };
  EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalEqualDestinationTimes) {
  std::vector<Step> steps = {
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{200},
       TripId{1},
       TripId{1}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{150},
       TimeSinceServiceStart{200},
       TripId{2},
       TripId{2}}
  };
  EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, MergeStepsTest) {
  // Load pre-filtered GTFS data (contains all CT: trips from 20250718)
  std::string gtfs_directory_path = "../data/RG_20250718_CT";
  GtfsDay gtfs_day = GtfsLoadDay(gtfs_directory_path);

  // Get steps from GTFS
  StepsFromGtfs result = GetStepsFromGtfs(gtfs_day);

  // Define the three stops
  const GtfsStopId san_jose_diridon_northbound{"70261"};
  const GtfsStopId sunnyvale_northbound{"70221"};
  const GtfsStopId mountain_view_northbound{"70211"};

  // Get mapped stop IDs
  StopId stop_a =
      result.mapping.gtfs_stop_id_to_stop_id.at(san_jose_diridon_northbound);
  StopId stop_b =
      result.mapping.gtfs_stop_id_to_stop_id.at(sunnyvale_northbound);
  StopId stop_c =
      result.mapping.gtfs_stop_id_to_stop_id.at(mountain_view_northbound);

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

  // These should be minimal because Caltrain doesn't have overtakes.
  EXPECT_TRUE(CheckSortedAndMinimal(ab));
  EXPECT_TRUE(CheckSortedAndMinimal(bc));

  // Call MergeSteps
  std::vector<Step> ac = MergeSteps(ab, bc);

  TripId ct503 =
      result.mapping.gtfs_trip_id_to_trip_id.at(GtfsTripId{"CT:503"});
  TripId ct507 =
      result.mapping.gtfs_trip_id_to_trip_id.at(GtfsTripId{"CT:507"});
  TripId ct511 =
      result.mapping.gtfs_trip_id_to_trip_id.at(GtfsTripId{"CT:511"});
  TripId ct515 =
      result.mapping.gtfs_trip_id_to_trip_id.at(GtfsTripId{"CT:515"});
  TripId ct519 =
      result.mapping.gtfs_trip_id_to_trip_id.at(GtfsTripId{"CT:519"});
  TripId ct523 =
      result.mapping.gtfs_trip_id_to_trip_id.at(GtfsTripId{"CT:523"});
  TripId ct527 =
      result.mapping.gtfs_trip_id_to_trip_id.at(GtfsTripId{"CT:527"});

  std::vector<Step> expected_ac = {
      Step{
          stop_a,
          stop_c,
          TimeSinceServiceStart{ParseGtfsTime("06:22:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("06:36:00").seconds},
          ct503,
          ct503
      },
      Step{
          stop_a,
          stop_c,
          TimeSinceServiceStart{ParseGtfsTime("07:22:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("07:36:00").seconds},
          ct507,
          ct507
      },
      Step{
          stop_a,
          stop_c,
          TimeSinceServiceStart{ParseGtfsTime("08:22:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("08:36:00").seconds},
          ct511,
          ct511
      },
      Step{
          stop_a,
          stop_c,
          TimeSinceServiceStart{ParseGtfsTime("15:22:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("15:36:00").seconds},
          ct515,
          ct515
      },
      Step{
          stop_a,
          stop_c,
          TimeSinceServiceStart{ParseGtfsTime("16:22:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("16:36:00").seconds},
          ct519,
          ct519
      },
      Step{
          stop_a,
          stop_c,
          TimeSinceServiceStart{ParseGtfsTime("17:22:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("17:36:00").seconds},
          ct523,
          ct523
      },
      Step{
          stop_a,
          stop_c,
          TimeSinceServiceStart{ParseGtfsTime("18:22:00").seconds},
          TimeSinceServiceStart{ParseGtfsTime("18:36:00").seconds},
          ct527,
          ct527
      },
  };

  EXPECT_EQ(ac, expected_ac);
}

TEST(StepMergeTest, SortByOriginAndDestinationTimeWithSecondarySort) {
  // Test case with same origin times but different destination times
  std::vector<Step> steps = {
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{300},
       TripId{1},
       TripId{1}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{200},
       TripId{2},
       TripId{2}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{400},
       TripId{3},
       TripId{3}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{150},
       TimeSinceServiceStart{250},
       TripId{4},
       TripId{4}}
  };

  SortByOriginAndDestinationTime(steps);

  // Verify primary sort by origin_time ascending
  for (size_t i = 1; i < steps.size(); ++i) {
    EXPECT_LE(steps[i - 1].origin_time.seconds, steps[i].origin_time.seconds);
  }

  // Verify secondary sort by destination_time descending when origin times are
  // equal
  EXPECT_EQ(steps[0].origin_time.seconds, 100);
  EXPECT_EQ(
      steps[0].destination_time.seconds,
      400
  );  // highest destination time first
  EXPECT_EQ(steps[1].origin_time.seconds, 100);
  EXPECT_EQ(steps[1].destination_time.seconds, 300);
  EXPECT_EQ(steps[2].origin_time.seconds, 100);
  EXPECT_EQ(
      steps[2].destination_time.seconds,
      200
  );  // lowest destination time last
  EXPECT_EQ(steps[3].origin_time.seconds, 150);
}

TEST(StepMergeTest, MakeMinimalCoverSimpleTest) {
  // Simple test case: step 2 dominates step 1 (departs later, arrives earlier)
  std::vector<Step> steps = {
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{300},
       TripId{1},
       TripId{1}},  // dominated
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{150},
       TimeSinceServiceStart{250},
       TripId{2},
       TripId{2}}  // dominates step 1
  };

  MakeMinimalCover(steps);

  EXPECT_EQ(steps.size(), 1);
  EXPECT_EQ(steps[0].origin_time.seconds, 150);
  EXPECT_EQ(steps[0].destination_time.seconds, 250);
}

TEST(StepMergeTest, MakeMinimalCoverTest) {
  // Test case where only the last step (earliest arrival) should remain
  std::vector<Step> steps = {
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{300},
       TripId{1},
       TripId{1}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{120},
       TimeSinceServiceStart{350},
       TripId{2},
       TripId{2}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{150},
       TimeSinceServiceStart{250},
       TripId{3},
       TripId{3}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{180},
       TimeSinceServiceStart{280},
       TripId{4},
       TripId{4}},
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{200},
       TimeSinceServiceStart{240},
       TripId{5},
       TripId{5}}
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
      {StopId{1},
       StopId{2},
       TimeSinceServiceStart{100},
       TimeSinceServiceStart{200},
       TripId{1},
       TripId{1}}
  };
  MakeMinimalCover(single_step);
  EXPECT_EQ(single_step.size(), 1);
}

RC_GTEST_PROP(
    StepMergeTest,
    MakeMinimalCoverProperties,
    (std::vector<StepFromTo<1, 2>> phantom_steps)
) {
  std::vector<Step> steps(phantom_steps.begin(), phantom_steps.end());

  // Sort as precondition
  SortByOriginAndDestinationTime(steps);
  RC_LOG() << "Sorted steps: " << rc::toString(steps) << "\n";

  // Make a copy for the minimal cover
  std::vector<Step> minimal_cover = steps;
  MakeMinimalCover(minimal_cover);
  RC_LOG() << "Minimal cover: " << rc::toString(minimal_cover) << "\n";

  // Property 1: minimal cover is a subset of original steps
  std::unordered_set<Step> original_steps_set(steps.begin(), steps.end());
  for (const auto& step : minimal_cover) {
    RC_ASSERT(original_steps_set.count(step) > 0);
  }

  // Property 2: satisfies CheckSortedAndMinimal
  RC_ASSERT(CheckSortedAndMinimal(minimal_cover));

  // Property 3: for any original step, there is a step in minimal cover that
  // dominates it.
  for (const auto& orig_step : steps) {
    bool dominated_or_kept = false;
    for (const auto& cover_step : minimal_cover) {
      if (cover_step.origin_time == TimeSinceServiceStart::FLEX_STEP_MARKER) {
        // Flex steps dominate flex steps by duration.
        if (orig_step.origin_time == TimeSinceServiceStart::FLEX_STEP_MARKER &&
            orig_step.destination_time.seconds >=
                cover_step.destination_time.seconds) {
          dominated_or_kept = true;
          break;
        }

        // Flex steps dominate non-flex steps by duration.
        if (orig_step.origin_time != TimeSinceServiceStart::FLEX_STEP_MARKER &&
            orig_step.destination_time.seconds -
                    orig_step.origin_time.seconds >=
                cover_step.destination_time.seconds) {
          dominated_or_kept = true;
          break;
        }
        continue;
      }

      if (cover_step.origin_time.seconds >= orig_step.origin_time.seconds &&
          cover_step.destination_time.seconds <=
              orig_step.destination_time.seconds) {
        dominated_or_kept = true;
        break;
      }
    }
    RC_ASSERT(dominated_or_kept);
  }
}

RC_GTEST_PROP(
    StepMergeTest,
    SortByOriginAndDestinationTimeProperty,
    (std::vector<StepFromTo<1, 2>> phantom_steps)
) {
  std::vector<Step> steps(phantom_steps.begin(), phantom_steps.end());

  // Sort using our function
  SortByOriginAndDestinationTime(steps);

  // Verify it's sorted by origin_time ascending
  for (size_t i = 1; i < steps.size(); ++i) {
    RC_ASSERT(steps[i - 1].origin_time.seconds <= steps[i].origin_time.seconds);
  }

  // Verify secondary sort by destination_time descending for equal origin times
  for (size_t i = 1; i < steps.size(); ++i) {
    if (steps[i - 1].origin_time.seconds == steps[i].origin_time.seconds) {
      RC_ASSERT(
          steps[i - 1].destination_time.seconds >=
          steps[i].destination_time.seconds
      );
    }
  }
}

RC_GTEST_PROP(
    StepMergeTest,
    MergeStepsProperty,
    (std::vector<StepFromTo<1, 2>> phantom_steps_12,
     std::vector<StepFromTo<2, 3>> phantom_steps_23)
) {
  std::vector<Step> steps_12(phantom_steps_12.begin(), phantom_steps_12.end());
  std::vector<Step> steps_23(phantom_steps_23.begin(), phantom_steps_23.end());

  // Sort both vectors
  SortByOriginAndDestinationTime(steps_12);
  SortByOriginAndDestinationTime(steps_23);

  // Get minimal covers
  MakeMinimalCover(steps_12);
  MakeMinimalCover(steps_23);
  RC_LOG() << "Minimal steps 1->2: " << rc::toString(steps_12) << "\n";
  RC_LOG() << "Minimal steps 2->3: " << rc::toString(steps_23) << "\n";

  // Merge the steps
  std::vector<Step> merged_steps = MergeSteps(steps_12, steps_23);
  RC_LOG() << "Merged steps: " << rc::toString(merged_steps) << "\n";

  // Property 1: the result satisfies CheckSortedAndMinimal
  RC_ASSERT(CheckSortedAndMinimal(merged_steps));

  // Property 2: for every valid connection (where transfer is possible),
  // there is a merged step that dominates or equals the connection
  for (const auto& step_12 : steps_12) {
    for (const auto& step_23 : steps_23) {
      bool step_12_flex =
          step_12.origin_time == TimeSinceServiceStart::FLEX_STEP_MARKER;
      bool step_23_flex =
          step_23.origin_time == TimeSinceServiceStart::FLEX_STEP_MARKER;

      // If both steps are flex, check that the first step of the result is a
      // flex combining them.
      if (step_12_flex && step_23_flex) {
        RC_ASSERT(
            merged_steps[0] == (Step{
                                   step_12.origin_stop,
                                   step_23.destination_stop,
                                   TimeSinceServiceStart::FLEX_STEP_MARKER,
                                   TimeSinceServiceStart{
                                       step_12.destination_time.seconds +
                                       step_23.destination_time.seconds
                                   },
                                   step_12.origin_trip,
                                   step_23.destination_trip,
                               })
        );
        continue;
      }

      // If this is an invalid connection, don't need to check that it is
      // dominated.
      if (!step_12_flex && !step_23_flex &&
          step_12.destination_time.seconds > step_23.origin_time.seconds) {
        continue;
      }

      // Compute the origin and destination times for the pair.
      int origin_time_seconds =
          step_12_flex
              ? step_23.origin_time.seconds - step_12.destination_time.seconds
              : step_12.origin_time.seconds;
      int destination_time_seconds = step_23_flex
                                         ? step_12.destination_time.seconds +
                                               step_23.destination_time.seconds
                                         : step_23.destination_time.seconds;

      // There should be a merged step that dominates this connection
      bool found_dominating_step = false;
      for (const auto& merged_step : merged_steps) {
        if (merged_step.origin_time.seconds >= origin_time_seconds &&
            merged_step.destination_time.seconds <= destination_time_seconds) {
          found_dominating_step = true;
          break;
        }
      }
      if (!found_dominating_step) {
        RC_LOG() << "No merged step dominates the pair " << step_12 << ", "
                 << step_23;
      }
      RC_ASSERT(found_dominating_step);
    }
  }

  // Property 3: every merged step has origin_ fields from a single steps_12
  // and destination_ fields from a single steps_23, with valid transfer time
  for (const auto& merged_step : merged_steps) {
    if (merged_step.origin_time == TimeSinceServiceStart::FLEX_STEP_MARKER) {
      RC_ASSERT(
          merged_step.destination_time.seconds ==
          steps_12[0].destination_time.seconds +
              steps_23[0].destination_time.seconds
      );
    }

    // Find the corresponding steps_12 and steps_23 that this merged step is
    // based on
    const Step* source_step_12 = nullptr;
    const Step* source_step_23 = nullptr;

    for (const auto& step_12 : steps_12) {
      if (step_12.origin_stop == merged_step.origin_stop &&
          step_12.origin_time == merged_step.origin_time &&
          step_12.origin_trip == merged_step.origin_trip) {
        source_step_12 = &step_12;
        break;
      }
    }

    for (const auto& step_23 : steps_23) {
      if (step_23.destination_stop == merged_step.destination_stop &&
          step_23.destination_time == merged_step.destination_time &&
          step_23.destination_trip == merged_step.destination_trip) {
        source_step_23 = &step_23;
        break;
      }
    }

    if (source_step_12 == nullptr && source_step_23 == nullptr) {
      RC_FAIL("both source steps nullptr");
    } else if (source_step_12 == nullptr) {
      RC_ASSERT(
          merged_step.origin_time.seconds +
              steps_12[0].destination_time.seconds ==
          source_step_23->origin_time.seconds
      );
    } else if (source_step_23 == nullptr) {
      RC_ASSERT(
          source_step_12->destination_time.seconds +
              steps_23[0].destination_time.seconds ==
          merged_step.destination_time.seconds
      );
    } else {
      // Check that the destination_time of steps_12 <= origin_time of steps_23
      RC_ASSERT(
          source_step_12->destination_time.seconds <=
          source_step_23->origin_time.seconds
      );
    }
  }
}

}  // namespace vats5