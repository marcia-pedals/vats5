#include "step_merge.h"

#include <algorithm>
#include <limits>

namespace vats5 {

void SortSteps(std::vector<Step>& steps) {
  std::sort(steps.begin(), steps.end(), [](const Step& a, const Step& b) {
    // Flex steps come first
    if (a.is_flex != b.is_flex) {
      return a.is_flex > b.is_flex;  // flex first
    }

    // Among flex steps, sort by duration ascending
    if (a.is_flex && b.is_flex) {
      return a.FlexDurationSeconds() < b.FlexDurationSeconds();
    }

    // Among non-flex steps, sort by origin time then destination time
    if (a.origin_time.seconds != b.origin_time.seconds) {
      return a.origin_time.seconds < b.origin_time.seconds;
    }
    return a.destination_time.seconds > b.destination_time.seconds;
  });
}

void MakeMinimalCover(std::vector<Step>& steps) {
  if (steps.size() <= 1) {
    return;
  }

  const StopId deletion_marker = StopId{std::numeric_limits<int>::min()};

  // Forward sweep through all the flex steps to delete all but the first
  // (shortest) one and to record the duration of the first (shortest) one.
  int flex_step_duration = std::numeric_limits<int>::max();
  for (int i = 0; i < steps.size() && steps[i].is_flex; ++i) {
    if (i == 0) {
      flex_step_duration = steps[i].FlexDurationSeconds();
    } else {
      steps[i].origin_stop = deletion_marker;
    }
  }

  // Backwards sweep through the non-flex steps: a step is dominated if there's
  // a later-departing step that arrives earlier OR if the flex step is
  // equal-or-shorter.
  int earliest_destination_time = std::numeric_limits<int>::max();
  for (int i = static_cast<int>(steps.size()) - 1; i >= 0 && !steps[i].is_flex;
       i--) {
    if (steps[i].destination_time.seconds >= earliest_destination_time ||
        steps[i].destination_time.seconds - steps[i].origin_time.seconds >=
            flex_step_duration) {
      // This step is dominated.
      steps[i].origin_stop = deletion_marker;
    } else {
      // This step is not dominated, update earliest destination time
      earliest_destination_time = steps[i].destination_time.seconds;
    }
  }

  // Remove marked steps in-place using two-pointer technique
  size_t write_pos = 0;
  for (size_t read_pos = 0; read_pos < steps.size(); read_pos++) {
    if (steps[read_pos].origin_stop != deletion_marker) {
      if (write_pos != read_pos) {
        steps[write_pos] = std::move(steps[read_pos]);
      }
      write_pos++;
    }
  }

  steps.resize(write_pos);
}

bool CheckSortedAndMinimal(const std::vector<Step>& steps) {
  if (steps.empty()) {
    return true;
  }
  bool has_flex = steps[0].is_flex;
  size_t first_to_check = has_flex ? 1 : 0;
  int flex_duration = has_flex ? steps[0].FlexDurationSeconds()
                               : std::numeric_limits<int>::max();
  for (size_t i = first_to_check; i < steps.size(); i++) {
    const Step& curr = steps[i];

    // Only the first step is allowed to be flex.
    if (curr.is_flex) {
      return false;
    }

    // Can't have any non-flex step that takes as long as the flex step.
    if (curr.destination_time.seconds - curr.origin_time.seconds >=
        flex_duration) {
      return false;
    }

    if (i > first_to_check) {
      const Step& prev = steps[i - 1];

      // Must be sorted by origin time ascending.
      if (curr.origin_time.seconds <= prev.origin_time.seconds) {
        return false;
      }

      // Must be sorted by destination time ascending.
      if (curr.destination_time.seconds <= prev.destination_time.seconds) {
        return false;
      }
    }
  }
  return true;
}

std::vector<Step> MergeSteps(
    const std::vector<Step>& ab, const std::vector<Step>& bc
) {
  std::vector<Step> result;
  if (ab.empty() || bc.empty()) {
    return result;
  }

  bool ab_flex = ab[0].is_flex;
  bool bc_flex = bc[0].is_flex;

  size_t ab_idx = ab_flex ? 1 : 0;
  size_t bc_idx = bc_flex ? 1 : 0;

  // If both have flex steps, make a combined flex step and put it first in the
  // result.
  if (ab_flex && bc_flex) {
    result.emplace_back(Step{
        ab[0].origin_stop,
        bc[0].destination_stop,
        TimeSinceServiceStart{0},
        TimeSinceServiceStart{
            ab[0].FlexDurationSeconds() + bc[0].FlexDurationSeconds()
        },
        ab[0].origin_trip,
        bc[0].destination_trip,
        true  // is_flex
    });
  }

  int prev_origin_time_seconds = 0;

  while (ab_idx < ab.size()) {
    // Advance bc_idx to the first step that can be reached by ab_idx.
    // While doing so, if ab has a flex step, consider inserting steps that use
    // that flex step to reach the bc steps that we are skipping.
    while (bc_idx < bc.size() && bc[bc_idx].origin_time.seconds <
                                     ab[ab_idx].destination_time.seconds) {
      // If we can flex to bc[bc_idx] leaving the origin after our previous
      // origin time, then it is worthwhile to do this, so add a step.
      if (ab_flex) {
        int flex_origin_time_seconds =
            bc[bc_idx].origin_time.seconds - ab[0].FlexDurationSeconds();
        if (flex_origin_time_seconds > prev_origin_time_seconds) {
          result.emplace_back(Step{
              ab[0].origin_stop,
              bc[bc_idx].destination_stop,
              TimeSinceServiceStart{flex_origin_time_seconds},
              bc[bc_idx].destination_time,
              ab[0].origin_trip,
              bc[bc_idx].destination_trip,
              false  // is_flex
          });
          prev_origin_time_seconds = flex_origin_time_seconds;
        }
      }
      bc_idx += 1;
    }

    // If we reached the end of bc, we are almost done. We just need to flex
    // from any remaining ab steps, if applicable.
    if (bc_idx == bc.size()) {
      if (bc_flex) {
        while (ab_idx < ab.size()) {
          result.emplace_back(Step{
              ab[ab_idx].origin_stop,
              bc[0].destination_stop,
              ab[ab_idx].origin_time,
              TimeSinceServiceStart{
                  ab[ab_idx].origin_time.seconds + bc[0].FlexDurationSeconds()
              },
              ab[ab_idx].origin_trip,
              bc[0].destination_trip,
              false  // is_flex
          });
          prev_origin_time_seconds = ab[ab_idx].origin_time.seconds;
          ab_idx += 1;
        }
      }
      return result;
    }

    // Advance ab_idx to the last ab step that arrives in time to catch
    // bc_idx.
    // While doing so, if bc has a flex step, consider inserting steps that use
    // that flex step to continue from the ab steps we are skipping.
    while (ab_idx < ab.size() - 1 && ab[ab_idx + 1].destination_time.seconds <=
                                         bc[bc_idx].origin_time.seconds) {
      if (bc_flex) {
        int flex_destination_time_seconds =
            ab[ab_idx].destination_time.seconds + bc[0].FlexDurationSeconds();
        if (flex_destination_time_seconds <
            bc[bc_idx].destination_time.seconds) {
          result.emplace_back(Step{
              ab[ab_idx].origin_stop,
              bc[0].destination_stop,
              ab[ab_idx].origin_time,
              TimeSinceServiceStart{flex_destination_time_seconds},
              ab[ab_idx].origin_trip,
              bc[0].destination_trip,
              false  // is_flex
          });
          prev_origin_time_seconds = ab[ab_idx].origin_time.seconds;
        }
      }
      ab_idx += 1;
    }

    result.emplace_back(Step{
        ab[ab_idx].origin_stop,
        bc[bc_idx].destination_stop,
        ab[ab_idx].origin_time,
        bc[bc_idx].destination_time,
        ab[ab_idx].origin_trip,
        bc[bc_idx].destination_trip,
        false  // is_flex
    });
    prev_origin_time_seconds = ab[ab_idx].origin_time.seconds;

    ab_idx += 1;
  }

  return result;
}

}  // namespace vats5