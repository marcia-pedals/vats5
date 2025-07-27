#include "step_merge.h"

#include <algorithm>
#include <limits>

namespace vats5 {

static bool SmallerStep(const Step& a, const Step& b) {
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
}

void SortSteps(std::vector<Step>& steps) {
  std::sort(steps.begin(), steps.end(), SmallerStep);
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

static Step MergedStep(Step ab, Step bc) {
  TimeSinceServiceStart origin_time{0}, destination_time{0};
  if (ab.is_flex && bc.is_flex) {
    origin_time.seconds = 0;
    destination_time.seconds =
        ab.FlexDurationSeconds() + bc.FlexDurationSeconds();
  } else {
    origin_time.seconds =
        ab.is_flex ? bc.origin_time.seconds - ab.FlexDurationSeconds()
                   : ab.origin_time.seconds;
    destination_time.seconds =
        bc.is_flex ? ab.destination_time.seconds + bc.FlexDurationSeconds()
                   : bc.destination_time.seconds;
  }
  return Step{
      ab.origin_stop,
      bc.destination_stop,
      origin_time,
      destination_time,
      ab.origin_trip,
      bc.destination_trip,
      ab.is_flex && bc.is_flex  // is_flex
  };
}

std::vector<Step> MergeSteps(
    const std::vector<Step>& ab, const std::vector<Step>& bc
) {
  std::vector<Step> result;
  if (ab.empty() || bc.empty()) {
    return result;
  }

  // Reserve a (pretty tight I think) upper bound on the amount of space we're
  // gonna allocate.
  size_t alloc_ub = 0;
  if (ab[0].is_flex) {
    alloc_ub += bc.size();
  }
  if (bc[0].is_flex) {
    alloc_ub += ab.size();
  }
  alloc_ub += std::min(ab.size(), bc.size());
  result.reserve(alloc_ub);

  // If both have flex steps, make a combined flex step and put it first in the
  // result.
  if (ab[0].is_flex && bc[0].is_flex) {
    result.emplace_back(MergedStep(ab[0], bc[0]));
  }

  size_t ab_start = ab[0].is_flex ? 1 : 0;
  size_t bc_start = bc[0].is_flex ? 1 : 0;

  // Think of this as a 3-way merge of sorted lists into a sorted list.
  //
  // This lists are:
  // ab_flex - [ab flex step] x bc[non-flex]
  // bc_flex - ab[non-flex] x [bc flex step]
  // non_flex - ab[non-flex] x bc[non-flex]
  //
  // They are sorted by the `SortSteps` order.
  size_t ab_flex_bc_i =
      ab[0].is_flex ? bc_start : bc.size();  // Only use this if ab has flex.
  size_t bc_flex_ab_i =
      bc[0].is_flex ? ab_start : ab.size();  // Only use this if bc has flex.

  // oh no i need to advance this to a valid and minimal connection
  size_t non_flex_ab_i = ab_start;
  size_t non_flex_bc_i = bc_start;

  // first make the connection valid by advancing bc_i
  while (non_flex_ab_i < ab.size() && non_flex_bc_i < bc.size() &&
         ab[non_flex_ab_i].destination_time > bc[non_flex_bc_i].origin_time) {
    non_flex_bc_i += 1;
  }
  // then make the conection minimal by advancing ab_i
  while (non_flex_ab_i + 1 < ab.size() && non_flex_bc_i < bc.size() &&
         ab[non_flex_ab_i + 1].destination_time <= bc[non_flex_bc_i].origin_time
  ) {
    non_flex_ab_i += 1;
  }

  Step BAD_STEP{
      StopId{0},
      StopId{0},
      TimeSinceServiceStart{std::numeric_limits<int>::max()},
      TimeSinceServiceStart{std::numeric_limits<int>::max()},
      TripId{0},
      TripId{0},
      false
  };

  Step ab_flex_step =
      ab_flex_bc_i < bc.size() ? MergedStep(ab[0], bc[ab_flex_bc_i]) : BAD_STEP;
  Step bc_flex_step =
      bc_flex_ab_i < ab.size() ? MergedStep(ab[bc_flex_ab_i], bc[0]) : BAD_STEP;
  Step non_flex_step = non_flex_ab_i < ab.size() && non_flex_bc_i < bc.size()
                           ? MergedStep(ab[non_flex_ab_i], bc[non_flex_bc_i])
                           : BAD_STEP;

  while (ab_flex_bc_i < bc.size() || bc_flex_ab_i < ab.size() ||
         (non_flex_ab_i < ab.size() && non_flex_bc_i < bc.size())) {
    if (SmallerStep(ab_flex_step, bc_flex_step) &&
        SmallerStep(ab_flex_step, non_flex_step)) {
      // ab_flex_step is smallest
      result.emplace_back(ab_flex_step);
      ab_flex_bc_i += 1;
      ab_flex_step = ab_flex_bc_i < bc.size()
                         ? MergedStep(ab[0], bc[ab_flex_bc_i])
                         : BAD_STEP;
    } else if (SmallerStep(bc_flex_step, ab_flex_step) &&
               SmallerStep(bc_flex_step, non_flex_step)) {
      // bc_flex_step is smallest
      result.emplace_back(bc_flex_step);
      bc_flex_ab_i += 1;
      bc_flex_step = bc_flex_ab_i < ab.size()
                         ? MergedStep(ab[bc_flex_ab_i], bc[0])
                         : BAD_STEP;
    } else {
      // non_flex_step is smallest
      result.emplace_back(non_flex_step);
      non_flex_ab_i += 1;
      // first make the connection valid by advancing bc_i
      while (non_flex_ab_i < ab.size() && non_flex_bc_i < bc.size() &&
             ab[non_flex_ab_i].destination_time > bc[non_flex_bc_i].origin_time
      ) {
        non_flex_bc_i += 1;
      }
      // then make the conection minimal by advancing ab_i
      while (non_flex_ab_i + 1 < ab.size() && non_flex_bc_i < bc.size() &&
             ab[non_flex_ab_i + 1].destination_time <=
                 bc[non_flex_bc_i].origin_time) {
        non_flex_ab_i += 1;
      }
      non_flex_step = non_flex_ab_i < ab.size() && non_flex_bc_i < bc.size()
                          ? MergedStep(ab[non_flex_ab_i], bc[non_flex_bc_i])
                          : BAD_STEP;
    }
  }

  // The 3-way merge may have introduced dominated steps. Delete them.
  // TODO: I think it might be possible to suppress these during the 3-way merge
  // so that we don't have to allocate space and then do an extra sweep to
  // delete them.
  MakeMinimalCover(result);
  return result;
}

std::vector<Step> MergeStepsOLD(
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