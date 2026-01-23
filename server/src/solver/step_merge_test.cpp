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

// A utility for assigining distinct trip ids to multiple vectors of trips.
struct DistinctTripIds {
  int cur = 1;
  void Assign(std::vector<vats5::Step>& steps) {
    for (auto& step : steps) {
      step.origin.trip.v = cur;
      step.destination.trip.v = cur;
      cur += 1;
    }
  }
};

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
          bool is_flex =
              (origin_time.seconds == -1);  // Use -1 as marker for flex
          vats5::TimeSinceServiceStart actual_origin_time =
              is_flex ? vats5::TimeSinceServiceStart{0} : origin_time;
          vats5::TimeSinceServiceStart dest_time{
              is_flex ? duration_offset : origin_time.seconds + duration_offset
          };
          StepFromTo<Origin, Destination> step{
              vats5::StepEndpoint{vats5::StopId{Origin}, is_flex, vats5::StepPartitionId::NONE, actual_origin_time, vats5::TripId{0}},
              vats5::StepEndpoint{vats5::StopId{Destination}, is_flex, vats5::StepPartitionId::NONE, dest_time, vats5::TripId{0}},
              is_flex
          };
          return step;
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
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1})
  };
  EXPECT_TRUE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalSortedByOriginTime) {
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{250}, TripId{2}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{200}, TimeSinceServiceStart{300}, TripId{3})
  };
  EXPECT_TRUE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalNotSortedByOriginTime) {
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{200}, TimeSinceServiceStart{250}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{2})
  };
  EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalNotSortedByDestinationTime) {
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{200}, TripId{2})
  };
  EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalEqualOriginTimes) {
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{250}, TripId{2})
  };
  EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, CheckSortedAndMinimalEqualDestinationTimes) {
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{200}, TripId{2})
  };
  EXPECT_FALSE(CheckSortedAndMinimal(steps));
}

TEST(StepMergeTest, MergeStepsTest) {
  // Load pre-filtered GTFS data (contains all CT: trips from 20250718)
  std::string gtfs_directory_path = "../data/RG_20250718_CT";
  GtfsDay gtfs_day = GtfsLoadDay(gtfs_directory_path);

  // Get steps from GTFS
  StepsFromGtfs result = GetStepsFromGtfs(gtfs_day, GetStepsOptions{500.0});

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
    if (step.origin.stop == stop_a && step.destination.stop == stop_b) {
      ab.push_back(step);
    }
  }

  // Construct bc vector (steps from B to C)
  std::vector<Step> bc;
  for (const auto& step : result.steps) {
    if (step.origin.stop == stop_b && step.destination.stop == stop_c) {
      bc.push_back(step);
    }
  }

  // Sort by origin_time ascending
  SortSteps(ab);
  SortSteps(bc);

  // These should be minimal because Caltrain doesn't have overtakes.
  EXPECT_TRUE(CheckSortedAndMinimal(ab));
  EXPECT_TRUE(CheckSortedAndMinimal(bc));

  // Call MergeSteps
  std::vector<Step> ac = PairwiseMergedSteps(ab, bc);

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
      Step::PrimitiveScheduled(stop_a, stop_c, TimeSinceServiceStart{ParseGtfsTime("06:22:00").seconds}, TimeSinceServiceStart{ParseGtfsTime("06:36:00").seconds}, ct503),
      Step::PrimitiveScheduled(stop_a, stop_c, TimeSinceServiceStart{ParseGtfsTime("07:22:00").seconds}, TimeSinceServiceStart{ParseGtfsTime("07:36:00").seconds}, ct507),
      Step::PrimitiveScheduled(stop_a, stop_c, TimeSinceServiceStart{ParseGtfsTime("08:22:00").seconds}, TimeSinceServiceStart{ParseGtfsTime("08:36:00").seconds}, ct511),
      Step::PrimitiveScheduled(stop_a, stop_c, TimeSinceServiceStart{ParseGtfsTime("15:22:00").seconds}, TimeSinceServiceStart{ParseGtfsTime("15:36:00").seconds}, ct515),
      Step::PrimitiveScheduled(stop_a, stop_c, TimeSinceServiceStart{ParseGtfsTime("16:22:00").seconds}, TimeSinceServiceStart{ParseGtfsTime("16:36:00").seconds}, ct519),
      Step::PrimitiveScheduled(stop_a, stop_c, TimeSinceServiceStart{ParseGtfsTime("17:22:00").seconds}, TimeSinceServiceStart{ParseGtfsTime("17:36:00").seconds}, ct523),
      Step::PrimitiveScheduled(stop_a, stop_c, TimeSinceServiceStart{ParseGtfsTime("18:22:00").seconds}, TimeSinceServiceStart{ParseGtfsTime("18:36:00").seconds}, ct527),
  };

  EXPECT_EQ(ac, expected_ac);
}

TEST(StepMergeTest, SortByOriginAndDestinationTimeWithSecondarySort) {
  // Test case with same origin times but different destination times
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{2}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{400}, TripId{3}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{250}, TripId{4})
  };

  SortSteps(steps);

  // Verify primary sort by origin_time ascending
  for (size_t i = 1; i < steps.size(); ++i) {
    EXPECT_LE(steps[i - 1].origin.time.seconds, steps[i].origin.time.seconds);
  }

  // Verify secondary sort by destination_time descending when origin times are
  // equal
  EXPECT_EQ(steps[0].origin.time.seconds, 100);
  EXPECT_EQ(
      steps[0].destination.time.seconds,
      400
  );  // highest destination time first
  EXPECT_EQ(steps[1].origin.time.seconds, 100);
  EXPECT_EQ(steps[1].destination.time.seconds, 300);
  EXPECT_EQ(steps[2].origin.time.seconds, 100);
  EXPECT_EQ(
      steps[2].destination.time.seconds,
      200
  );  // lowest destination time last
  EXPECT_EQ(steps[3].origin.time.seconds, 150);
}

TEST(StepMergeTest, MakeMinimalCoverSimpleTest) {
  // Simple test case: step 2 dominates step 1 (departs later, arrives earlier)
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{1}),  // dominated
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{250}, TripId{2})  // dominates step 1
  };

  MakeMinimalCover(steps);

  EXPECT_EQ(steps.size(), 1);
  EXPECT_EQ(steps[0].origin.time.seconds, 150);
  EXPECT_EQ(steps[0].destination.time.seconds, 250);
}

TEST(StepMergeTest, MakeMinimalCoverTest) {
  // Test case where only the last step (earliest arrival) should remain
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{120}, TimeSinceServiceStart{350}, TripId{2}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{250}, TripId{3}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{180}, TimeSinceServiceStart{280}, TripId{4}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{200}, TimeSinceServiceStart{240}, TripId{5})
  };

  MakeMinimalCover(steps);

  // Only the last step should remain (arrives earliest at 240)
  EXPECT_EQ(steps.size(), 1);
  EXPECT_EQ(steps[0].origin.time.seconds, 200);
  EXPECT_EQ(steps[0].destination.time.seconds, 240);
}

TEST(StepMergeTest, MakeMinimalCoverEmptyAndSingle) {
  // Empty vector
  std::vector<Step> empty_steps;
  MakeMinimalCover(empty_steps);
  EXPECT_TRUE(empty_steps.empty());

  // Single step
  std::vector<Step> single_step = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1})
  };
  MakeMinimalCover(single_step);
  EXPECT_EQ(single_step.size(), 1);
}

TEST(StepMergeTest, MakeMinimalCoverParallel) {
  // Same data as MakeMinimalCoverTest but with a parallel vector of labels.
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{300}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{120}, TimeSinceServiceStart{350}, TripId{2}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{250}, TripId{3}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{180}, TimeSinceServiceStart{280}, TripId{4}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{200}, TimeSinceServiceStart{240}, TripId{5}),
  };
  std::vector<int> labels = {10, 20, 30, 40, 50};

  MakeMinimalCover(steps, &labels);

  // Only the last step survives (arrives earliest at 240, dominates all others)
  ASSERT_EQ(steps.size(), 1);
  ASSERT_EQ(labels.size(), 1);
  EXPECT_EQ(steps[0].origin.time.seconds, 200);
  EXPECT_EQ(labels[0], 50);
}

TEST(StepMergeTest, MakeMinimalCoverParallelMultipleSurvivors) {
  // Three non-dominated steps: each departs later and arrives later.
  std::vector<Step> steps = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{200}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{150}, TimeSinceServiceStart{280}, TripId{2}),  // dominated
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{200}, TimeSinceServiceStart{250}, TripId{3}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{300}, TimeSinceServiceStart{350}, TripId{4}),
  };
  std::vector<std::string> labels = {"first", "dominated", "middle", "last"};

  MakeMinimalCover(steps, &labels);

  ASSERT_EQ(steps.size(), 3);
  ASSERT_EQ(labels.size(), 3);
  EXPECT_EQ(labels[0], "first");
  EXPECT_EQ(labels[1], "middle");
  EXPECT_EQ(labels[2], "last");
}

RC_GTEST_PROP(
    StepMergeTest,
    MakeMinimalCoverProperties,
    (std::vector<StepFromTo<1, 2>> phantom_steps)
) {
  std::vector<Step> steps(phantom_steps.begin(), phantom_steps.end());

  DistinctTripIds d;
  d.Assign(steps);

  // Sort as precondition
  SortSteps(steps);
  RC_LOG() << "Sorted steps: " << rc::toString(steps) << "\n";

  // Build a parallel index vector to track which original steps survive.
  std::vector<size_t> indices(steps.size());
  for (size_t i = 0; i < steps.size(); i++) indices[i] = i;

  // Make a copy for the minimal cover
  std::vector<Step> minimal_cover = steps;
  MakeMinimalCover(minimal_cover, &indices);
  RC_LOG() << "Minimal cover: " << rc::toString(minimal_cover) << "\n";

  // Property 0: parallel vector has same size as steps
  RC_ASSERT(indices.size() == minimal_cover.size());

  // Property 0b: each surviving step matches the original at that index
  for (size_t i = 0; i < minimal_cover.size(); i++) {
    RC_ASSERT(minimal_cover[i] == steps[indices[i]]);
  }

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
      if (cover_step.is_flex) {
        // Flex steps dominate flex steps by duration.
        if (orig_step.is_flex && cover_step.FlexDurationSeconds() <=
                                     orig_step.FlexDurationSeconds()) {
          dominated_or_kept = true;
          break;
        }

        // Flex steps dominate non-flex steps by duration.
        if (!orig_step.is_flex && orig_step.destination.time.seconds -
                                          orig_step.origin.time.seconds >=
                                      cover_step.FlexDurationSeconds()) {
          dominated_or_kept = true;
          break;
        }
        continue;
      }

      if (cover_step.origin.time.seconds >= orig_step.origin.time.seconds &&
          cover_step.destination.time.seconds <=
              orig_step.destination.time.seconds) {
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

  DistinctTripIds d;
  d.Assign(steps);

  // Sort using our function
  SortSteps(steps);

  // Verify the new sorting order: flex steps first, then non-flex steps
  size_t first_non_flex = 0;
  while (first_non_flex < steps.size() && steps[first_non_flex].is_flex) {
    first_non_flex++;
  }

  // Verify flex steps are sorted by duration ascending
  for (size_t i = 1; i < first_non_flex; ++i) {
    RC_ASSERT(
        steps[i - 1].FlexDurationSeconds() <= steps[i].FlexDurationSeconds()
    );
  }

  // Verify non-flex steps are sorted by origin_time ascending
  for (size_t i = first_non_flex + 1; i < steps.size(); ++i) {
    RC_ASSERT(steps[i - 1].origin.time.seconds <= steps[i].origin.time.seconds);
  }

  // Verify secondary sort by destination_time descending for equal origin times
  // (non-flex only)
  for (size_t i = first_non_flex + 1; i < steps.size(); ++i) {
    if (steps[i - 1].origin.time.seconds == steps[i].origin.time.seconds) {
      RC_ASSERT(
          steps[i - 1].destination.time.seconds >=
          steps[i].destination.time.seconds
      );
    }
  }
}

// Regression test: PairwiseMergedSteps used to infinite-loop when the ab-flex
// and bc-flex merged steps had identical origin/destination times (a tie in
// SmallerStep) and the non-flex pair was invalid. The 3-way merge would fall
// through to the non-flex branch which couldn't advance, looping forever.
TEST(StepMergeTest, PairwiseMergedStepsTiedFlexSteps) {
  // ab: flex with duration 20, plus a non-flex step (duration 10 < 20)
  std::vector<Step> ab = {
      Step::PrimitiveFlex(StopId{1}, StopId{2}, 20, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{110}, TripId{2}),
  };
  // bc: flex with duration 15, plus a non-flex step (duration 5 < 15)
  // Chosen so that all three merge candidates produce origin=100, dest=125:
  //   ab_flex merged with bc_non_flex: origin = 120-20 = 100, dest = 125
  //   bc_flex merged with ab_non_flex: origin = 100, dest = 110+15 = 125
  //   non_flex pair merged:            origin = 100, dest = 125
  // The non_flex pair is valid but gets consumed first, then the two tied
  // flex steps used to cause an infinite loop.
  std::vector<Step> bc = {
      Step::PrimitiveFlex(StopId{2}, StopId{3}, 15, TripId{3}),
      Step::PrimitiveScheduled(StopId{2}, StopId{3}, TimeSinceServiceStart{120}, TimeSinceServiceStart{125}, TripId{4}),
  };

  ASSERT_TRUE(CheckSortedAndMinimal(ab));
  ASSERT_TRUE(CheckSortedAndMinimal(bc));

  // This used to hang. Now it should terminate and produce a valid result.
  std::vector<Step> result = PairwiseMergedSteps(ab, bc);
  EXPECT_TRUE(CheckSortedAndMinimal(result));
  // Should have: flex+flex (duration 35), plus one non-flex step at (100, 125).
  // The three tied merged steps (all origin=100, dest=125) collapse to one
  // via MakeMinimalCover.
  ASSERT_EQ(result.size(), 2u);
  EXPECT_TRUE(result[0].is_flex);
  EXPECT_EQ(result[0].FlexDurationSeconds(), 35);
  EXPECT_FALSE(result[1].is_flex);
  EXPECT_EQ(result[1].origin.time.seconds, 100);
  EXPECT_EQ(result[1].destination.time.seconds, 125);
}

TEST(StepMergeTest, PairwiseMergedStepsProvenance) {
  // ab: flex (duration 20) + non-flex at (100, 110)
  std::vector<Step> ab = {
      Step::PrimitiveFlex(StopId{1}, StopId{2}, 20, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{110}, TripId{2}),
  };
  // bc: flex (duration 15) + non-flex at (120, 125)
  std::vector<Step> bc = {
      Step::PrimitiveFlex(StopId{2}, StopId{3}, 15, TripId{3}),
      Step::PrimitiveScheduled(StopId{2}, StopId{3}, TimeSinceServiceStart{120}, TimeSinceServiceStart{125}, TripId{4}),
  };

  ASSERT_TRUE(CheckSortedAndMinimal(ab));
  ASSERT_TRUE(CheckSortedAndMinimal(bc));

  std::vector<StepProvenance> provenance;
  std::vector<Step> result = PairwiseMergedSteps(ab, bc, &provenance);

  ASSERT_EQ(result.size(), provenance.size());

  // Verify each result step matches what MergedStep(ab[prov.ab_index],
  // bc[prov.bc_index]) would produce.
  for (size_t i = 0; i < result.size(); i++) {
    Step expected = MergedStep(ab[provenance[i].ab_index],
                               bc[provenance[i].bc_index]);
    EXPECT_EQ(result[i], expected)
        << "Mismatch at result[" << i << "]: provenance says ab["
        << provenance[i].ab_index << "] + bc[" << provenance[i].bc_index << "]";
  }
}

TEST(StepMergeTest, PairwiseMergedStepsProvenanceNonFlex) {
  // Pure non-flex case: three ab steps, two bc steps.
  std::vector<Step> ab = {
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{100}, TimeSinceServiceStart{150}, TripId{1}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{200}, TimeSinceServiceStart{250}, TripId{2}),
      Step::PrimitiveScheduled(StopId{1}, StopId{2}, TimeSinceServiceStart{300}, TimeSinceServiceStart{350}, TripId{3}),
  };
  std::vector<Step> bc = {
      Step::PrimitiveScheduled(StopId{2}, StopId{3}, TimeSinceServiceStart{160}, TimeSinceServiceStart{200}, TripId{4}),
      Step::PrimitiveScheduled(StopId{2}, StopId{3}, TimeSinceServiceStart{260}, TimeSinceServiceStart{300}, TripId{5}),
  };

  ASSERT_TRUE(CheckSortedAndMinimal(ab));
  ASSERT_TRUE(CheckSortedAndMinimal(bc));

  std::vector<StepProvenance> provenance;
  std::vector<Step> result = PairwiseMergedSteps(ab, bc, &provenance);

  ASSERT_EQ(result.size(), provenance.size());
  for (size_t i = 0; i < result.size(); i++) {
    Step expected = MergedStep(ab[provenance[i].ab_index],
                               bc[provenance[i].bc_index]);
    EXPECT_EQ(result[i], expected);
  }

  // We expect two merged steps: ab[0]+bc[0] and ab[1]+bc[1].
  // ab[2] + bc[1] is dominated by ab[1] + bc[1] (departs later, arrives same).
  ASSERT_EQ(result.size(), 2);
  EXPECT_EQ(provenance[0].ab_index, 0);
  EXPECT_EQ(provenance[0].bc_index, 0);
  EXPECT_EQ(provenance[1].ab_index, 1);
  EXPECT_EQ(provenance[1].bc_index, 1);
}

RC_GTEST_PROP(
    StepMergeTest,
    MergeStepsProperty,
    (std::vector<StepFromTo<1, 2>> phantom_steps_12,
     std::vector<StepFromTo<2, 3>> phantom_steps_23)
) {
  std::vector<Step> steps_12(phantom_steps_12.begin(), phantom_steps_12.end());
  std::vector<Step> steps_23(phantom_steps_23.begin(), phantom_steps_23.end());

  DistinctTripIds d;
  d.Assign(steps_12);
  d.Assign(steps_23);

  // Sort both vectors
  SortSteps(steps_12);
  SortSteps(steps_23);

  // Get minimal covers
  MakeMinimalCover(steps_12);
  MakeMinimalCover(steps_23);
  RC_LOG() << "Minimal steps 1->2: " << rc::toString(steps_12) << "\n";
  RC_LOG() << "Minimal steps 2->3: " << rc::toString(steps_23) << "\n";

  // Merge the steps, requesting provenance
  std::vector<StepProvenance> provenance;
  std::vector<Step> merged_steps =
      PairwiseMergedSteps(steps_12, steps_23, &provenance);
  RC_LOG() << "Merged steps: " << rc::toString(merged_steps) << "\n";

  // Property 0: provenance has same size as merged result
  RC_ASSERT(provenance.size() == merged_steps.size());

  // Property 0b: each provenance entry points to valid indices and the
  // result step equals MergedStep(ab[i], bc[j])
  for (size_t i = 0; i < merged_steps.size(); i++) {
    RC_ASSERT(provenance[i].ab_index < steps_12.size());
    RC_ASSERT(provenance[i].bc_index < steps_23.size());
    Step expected = MergedStep(steps_12[provenance[i].ab_index],
                               steps_23[provenance[i].bc_index]);
    RC_ASSERT(merged_steps[i] == expected);
  }

  // Property 1: the result satisfies CheckSortedAndMinimal
  RC_ASSERT(CheckSortedAndMinimal(merged_steps));

  // Property 2: for every valid connection (where transfer is possible),
  // there is a merged step that dominates or equals the connection
  for (const auto& step_12 : steps_12) {
    for (const auto& step_23 : steps_23) {
      bool step_12_flex = step_12.is_flex;
      bool step_23_flex = step_23.is_flex;

      // If both steps are flex, check that the first step of the result is a
      // flex combining them.
      if (step_12_flex && step_23_flex) {
        RC_ASSERT(
            merged_steps[0] == (Step{
                                   StepEndpoint{step_12.origin.stop, true, StepPartitionId::NONE, TimeSinceServiceStart{0}, step_12.origin.trip},
                                   StepEndpoint{step_23.destination.stop, true, StepPartitionId::NONE, TimeSinceServiceStart{
                                       step_12.FlexDurationSeconds() +
                                       step_23.FlexDurationSeconds()
                                   }, step_23.destination.trip},
                                   true
                               })
        );
        continue;
      }

      // If this is an invalid connection, don't need to check that it is
      // dominated.
      if (!step_12_flex && !step_23_flex &&
          step_12.destination.time.seconds > step_23.origin.time.seconds) {
        continue;
      }

      // Compute the origin and destination times for the pair.
      int origin_time_seconds = step_12_flex ? step_23.origin.time.seconds -
                                                   step_12.FlexDurationSeconds()
                                             : step_12.origin.time.seconds;
      int destination_time_seconds =
          step_23_flex
              ? step_12.destination.time.seconds + step_23.FlexDurationSeconds()
              : step_23.destination.time.seconds;

      // There should be a merged step that dominates this connection
      bool found_dominating_step = false;
      for (const auto& merged_step : merged_steps) {
        if (merged_step.origin.time.seconds >= origin_time_seconds &&
            merged_step.destination.time.seconds <= destination_time_seconds) {
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
    if (merged_step.is_flex) {
      RC_ASSERT(
          merged_step.FlexDurationSeconds() ==
          steps_12[0].FlexDurationSeconds() + steps_23[0].FlexDurationSeconds()
      );
    }

    // Find the corresponding steps_12 and steps_23 that this merged step is
    // based on
    const Step* source_step_12 = nullptr;
    const Step* source_step_23 = nullptr;

    for (const auto& step_12 : steps_12) {
      if (step_12.origin.stop == merged_step.origin.stop &&
          step_12.origin.time == merged_step.origin.time &&
          step_12.origin.trip == merged_step.origin.trip) {
        source_step_12 = &step_12;
        break;
      }
    }

    for (const auto& step_23 : steps_23) {
      if (step_23.destination.stop == merged_step.destination.stop &&
          step_23.destination.time == merged_step.destination.time &&
          step_23.destination.trip == merged_step.destination.trip) {
        source_step_23 = &step_23;
        break;
      }
    }

    if (source_step_12 == nullptr && source_step_23 == nullptr) {
      RC_FAIL("both source steps nullptr");
    } else if (source_step_12 == nullptr) {
      RC_ASSERT(
          merged_step.origin.time.seconds + steps_12[0].FlexDurationSeconds() ==
          source_step_23->origin.time.seconds
      );
    } else if (source_step_23 == nullptr) {
      RC_ASSERT(
          source_step_12->destination.time.seconds +
              steps_23[0].FlexDurationSeconds() ==
          merged_step.destination.time.seconds
      );
    } else {
      // Check that connection is valid
      // For non-flex steps, destination_time of step_12 <= origin_time of
      // step_23 For flex steps, they can connect at any time, so we need
      // different logic
      if (!source_step_12->is_flex && !source_step_23->is_flex) {
        RC_ASSERT(
            source_step_12->destination.time.seconds <=
            source_step_23->origin.time.seconds
        );
      }
      // For flex steps, the connection is always valid since they can be taken
      // at any time
    }
  }
}

TEST(StepMergeTest, MergedStepOriginDestinationFlex) {
  // Verify origin.is_flex comes from first step, destination.is_flex from second.
  Step ab = Step::PrimitiveFlex(StopId{1}, StopId{2}, 20, TripId{1});
  Step bc{
      StepEndpoint{StopId{2}, false, StepPartitionId{5}, TimeSinceServiceStart{100}, TripId{2}},
      StepEndpoint{StopId{3}, false, StepPartitionId{7}, TimeSinceServiceStart{120}, TripId{2}},
      false};

  Step merged = MergedStep(ab, bc);
  EXPECT_TRUE(merged.origin.is_flex);
  EXPECT_FALSE(merged.destination.is_flex);
  EXPECT_EQ(merged.origin.stop, StopId{1});
  EXPECT_EQ(merged.destination.stop, StopId{3});
}

TEST(StepMergeTest, MergedStepPartitionFlexFlex) {
  Step ab{
      StepEndpoint{StopId{1}, true, StepPartitionId{10}, TimeSinceServiceStart{0}, TripId{1}},
      StepEndpoint{StopId{2}, true, StepPartitionId{20}, TimeSinceServiceStart{30}, TripId{1}},
      true};
  Step bc{
      StepEndpoint{StopId{2}, true, StepPartitionId{30}, TimeSinceServiceStart{0}, TripId{2}},
      StepEndpoint{StopId{3}, true, StepPartitionId{40}, TimeSinceServiceStart{25}, TripId{2}},
      true};

  Step merged = MergedStep(ab, bc);
  EXPECT_EQ(merged.origin.partition, StepPartitionId{10});
  EXPECT_EQ(merged.destination.partition, StepPartitionId{40});
}

TEST(StepMergeTest, MergedStepPartitionFlexScheduled) {
  Step ab{
      StepEndpoint{StopId{1}, true, StepPartitionId{10}, TimeSinceServiceStart{0}, TripId{1}},
      StepEndpoint{StopId{2}, true, StepPartitionId{20}, TimeSinceServiceStart{30}, TripId{1}},
      true};
  Step bc{
      StepEndpoint{StopId{2}, false, StepPartitionId{30}, TimeSinceServiceStart{100}, TripId{2}},
      StepEndpoint{StopId{3}, false, StepPartitionId{40}, TimeSinceServiceStart{150}, TripId{2}},
      false};

  Step merged = MergedStep(ab, bc);
  EXPECT_EQ(merged.origin.partition, StepPartitionId{30});
  EXPECT_EQ(merged.destination.partition, StepPartitionId{40});
}

TEST(StepMergeTest, MergedStepPartitionScheduledFlex) {
  Step ab{
      StepEndpoint{StopId{1}, false, StepPartitionId{10}, TimeSinceServiceStart{100}, TripId{1}},
      StepEndpoint{StopId{2}, false, StepPartitionId{20}, TimeSinceServiceStart{150}, TripId{1}},
      false};
  Step bc{
      StepEndpoint{StopId{2}, true, StepPartitionId{30}, TimeSinceServiceStart{0}, TripId{2}},
      StepEndpoint{StopId{3}, true, StepPartitionId{40}, TimeSinceServiceStart{25}, TripId{2}},
      true};

  Step merged = MergedStep(ab, bc);
  EXPECT_EQ(merged.origin.partition, StepPartitionId{10});
  EXPECT_EQ(merged.destination.partition, StepPartitionId{20});
}

TEST(StepMergeTest, MergedStepPartitionScheduledScheduled) {
  Step ab{
      StepEndpoint{StopId{1}, false, StepPartitionId{10}, TimeSinceServiceStart{100}, TripId{1}},
      StepEndpoint{StopId{2}, false, StepPartitionId{20}, TimeSinceServiceStart{150}, TripId{1}},
      false};
  Step bc{
      StepEndpoint{StopId{2}, false, StepPartitionId{30}, TimeSinceServiceStart{150}, TripId{2}},
      StepEndpoint{StopId{3}, false, StepPartitionId{40}, TimeSinceServiceStart{200}, TripId{2}},
      false};

  Step merged = MergedStep(ab, bc);
  EXPECT_EQ(merged.origin.partition, StepPartitionId{10});
  EXPECT_EQ(merged.destination.partition, StepPartitionId{40});
}

TEST(StepMergeTest, ConsecutiveMergedStepsEqualsReduce) {
  rc::check([](void) {
    int path_length = *rc::gen::inRange(1, 6);

    std::vector<Step> path;
    // Start at time 0 so that all-flex paths agree with MergedStep's
    // normalization (MergedStep sets origin=0 for flex+flex merges).
    int current_time = 0;

    for (int i = 0; i < path_length; i++) {
      bool is_flex = *rc::gen::inRange(0, 2) == 1;
      int duration = *rc::gen::inRange(1, 600);
      StepPartitionId origin_partition = StepPartitionId{*rc::gen::inRange(0, 10)};
      StepPartitionId dest_partition = StepPartitionId{*rc::gen::inRange(0, 10)};
      TripId origin_trip{*rc::gen::inRange(0, 100)};
      TripId dest_trip{*rc::gen::inRange(0, 100)};

      int origin_time = current_time;
      int dest_time = origin_time + duration;

      path.push_back(Step{
          StepEndpoint{StopId{i}, is_flex, origin_partition, TimeSinceServiceStart{origin_time}, origin_trip},
          StepEndpoint{StopId{i + 1}, is_flex, dest_partition, TimeSinceServiceStart{dest_time}, dest_trip},
          is_flex
      });

      current_time = dest_time;
    }

    Step consecutive_result = ConsecutiveMergedSteps(path);

    Step fold_result = path[0];
    for (size_t i = 1; i < path.size(); i++) {
      fold_result = MergedStep(fold_result, path[i]);
    }

    RC_ASSERT(consecutive_result == fold_result);
  });
}

}  // namespace vats5
