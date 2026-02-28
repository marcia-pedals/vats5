#include "step_merge.h"

#include <algorithm>
#include <limits>

#include "solver/data.h"

namespace vats5 {

static bool SmallerOrEqualStep(const Step& a, const Step& b) {
  return !SmallerStep(b, a);
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
    if (curr.DurationSeconds() >= flex_duration) {
      return false;
    }

    if (i > first_to_check) {
      const Step& prev = steps[i - 1];

      // Must be sorted by origin time ascending.
      if (curr.origin.time.seconds <= prev.origin.time.seconds) {
        return false;
      }

      // Must be sorted by destination time ascending.
      if (curr.destination.time.seconds <= prev.destination.time.seconds) {
        return false;
      }
    }
  }
  return true;
}

Step MergedStep(Step ab, Step bc) {
  TimeSinceServiceStart origin_time{0}, destination_time{0};
  if (ab.is_flex && bc.is_flex) {
    origin_time.seconds = 0;
    destination_time.seconds =
        ab.FlexDurationSeconds() + bc.FlexDurationSeconds();
  } else {
    origin_time.seconds =
        ab.is_flex ? bc.origin.time.seconds - ab.FlexDurationSeconds()
                   : ab.origin.time.seconds;
    destination_time.seconds =
        bc.is_flex ? ab.destination.time.seconds + bc.FlexDurationSeconds()
                   : bc.destination.time.seconds;
  }
  return Step{
      StepEndpoint{
          ab.origin.stop,
          ab.origin.is_flex,
          // Use the origin partition of the first non-flex step, falling back
          // to ab if both are flex.
          (ab.is_flex && !bc.is_flex) ? bc.origin.partition
                                      : ab.origin.partition,
          origin_time,
          ab.origin.trip
      },
      StepEndpoint{
          bc.destination.stop,
          bc.destination.is_flex,
          // Use the destination partition of the last non-flex step, falling
          // back to bc if both are flex.
          (bc.is_flex && !ab.is_flex) ? ab.destination.partition
                                      : bc.destination.partition,
          destination_time,
          bc.destination.trip
      },
      ab.is_flex && bc.is_flex  // is_flex
  };
}

std::vector<Step> PairwiseMergedSteps(
    const std::vector<Step>& ab,
    const std::vector<Step>& bc,
    std::vector<StepProvenance>* provenance
) {
  std::vector<Step> result;
  if (ab.empty() || bc.empty()) {
    return result;
  }

  {
    StopId expected_b = ab[0].destination.stop;
    for (const Step& step : ab) {
      assert(step.destination.stop == expected_b);
    }
    for (const Step& step : bc) {
      assert(step.origin.stop == expected_b);
    }
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

  if (provenance) {
    provenance->clear();
    provenance->reserve(alloc_ub);
  }

  // If both have flex steps, make a combined flex step and put it first in the
  // result.
  if (ab[0].is_flex && bc[0].is_flex) {
    result.emplace_back(MergedStep(ab[0], bc[0]));
    if (provenance) {
      provenance->push_back({0, 0});
    }
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
  size_t non_flex_ab_i = ab_start;
  size_t non_flex_bc_i = bc_start;

  const auto MakeNonFlexValidMinimal =
      [&non_flex_ab_i, &non_flex_bc_i, &ab, &bc]() {
        // first make the connection valid by advancing bc_i
        while (non_flex_ab_i < ab.size() && non_flex_bc_i < bc.size() &&
               ab[non_flex_ab_i].destination.time >
                   bc[non_flex_bc_i].origin.time) {
          non_flex_bc_i += 1;
        }
        // then make the conection minimal by advancing ab_i
        while (non_flex_ab_i + 1 < ab.size() && non_flex_bc_i < bc.size() &&
               ab[non_flex_ab_i + 1].destination.time <=
                   bc[non_flex_bc_i].origin.time) {
          non_flex_ab_i += 1;
        }
      };
  MakeNonFlexValidMinimal();

  Step BAD_STEP = Step::PrimitiveScheduled(
      StopId{0},
      StopId{0},
      TimeSinceServiceStart{std::numeric_limits<int>::max()},
      TimeSinceServiceStart{std::numeric_limits<int>::max()},
      TripId{0}
  );

  const auto GetABFlexStep = [&ab_flex_bc_i, &ab, &bc, &BAD_STEP]() {
    return ab_flex_bc_i < bc.size() ? MergedStep(ab[0], bc[ab_flex_bc_i])
                                    : BAD_STEP;
  };
  const auto GetBCFlexStep = [&bc_flex_ab_i, &ab, &bc, &BAD_STEP]() {
    return bc_flex_ab_i < ab.size() ? MergedStep(ab[bc_flex_ab_i], bc[0])
                                    : BAD_STEP;
  };
  const auto GetNonFlexStep =
      [&non_flex_ab_i, &non_flex_bc_i, &ab, &bc, &BAD_STEP] {
        return non_flex_ab_i < ab.size() && non_flex_bc_i < bc.size()
                   ? MergedStep(ab[non_flex_ab_i], bc[non_flex_bc_i])
                   : BAD_STEP;
      };

  Step ab_flex_step = GetABFlexStep();
  Step bc_flex_step = GetBCFlexStep();
  Step non_flex_step = GetNonFlexStep();

  const auto StepGood = [](const Step& s) {
    return s.origin.time.seconds != std::numeric_limits<int>::max();
  };

  while (StepGood(ab_flex_step) || StepGood(bc_flex_step) ||
         StepGood(non_flex_step)) {
    if (SmallerOrEqualStep(ab_flex_step, bc_flex_step) &&
        SmallerOrEqualStep(ab_flex_step, non_flex_step)) {
      // ab_flex_step is smallest (or tied)
      result.emplace_back(ab_flex_step);
      if (provenance) {
        provenance->push_back({0, ab_flex_bc_i});
      }
      ab_flex_bc_i += 1;
      ab_flex_step = GetABFlexStep();
    } else if (SmallerOrEqualStep(bc_flex_step, non_flex_step)) {
      // bc_flex_step is smallest (or tied with non_flex)
      result.emplace_back(bc_flex_step);
      if (provenance) {
        provenance->push_back({bc_flex_ab_i, 0});
      }
      bc_flex_ab_i += 1;
      bc_flex_step = GetBCFlexStep();
    } else {
      // non_flex_step is smallest
      result.emplace_back(non_flex_step);
      if (provenance) {
        provenance->push_back({non_flex_ab_i, non_flex_bc_i});
      }
      non_flex_ab_i += 1;
      MakeNonFlexValidMinimal();
      non_flex_step = GetNonFlexStep();
    }
  }

  // The 3-way merge may have introduced dominated steps. Delete them.
  // TODO: I think it might be possible to suppress these during the 3-way merge
  // so that we don't have to allocate space and then do an extra sweep to
  // delete them.
  MakeMinimalCover(result, provenance);
  return result;
}

Step ConsecutiveMergedSteps(const std::vector<Step>& path) {
  if (path.empty()) {
    return Step{};
  }

  const Step& first = path.front();
  const Step& last = path.back();

  // Scan left-to-right for first non-flex step's origin partition and time.
  // If leading steps are flex, origin time = first_non_flex_origin - sum of
  // leading flex durations. If all steps are flex, origin time = 0.
  StepPartitionId origin_partition = path[0].origin.partition;
  int leading_flex_duration = 0;
  int first_non_flex_origin = 0;
  bool is_flex = true;
  for (size_t i = 0; i < path.size(); ++i) {
    if (path[i].is_flex) {
      leading_flex_duration += path[i].FlexDurationSeconds();
    } else {
      first_non_flex_origin = path[i].origin.time.seconds;
      origin_partition = path[i].origin.partition;
      is_flex = false;
      break;
    }
  }
  TimeSinceServiceStart origin_time{
      is_flex ? 0 : first_non_flex_origin - leading_flex_duration
  };

  // Scan right-to-left for last non-flex step's destination partition and time.
  // If trailing steps are flex, destination time = last_non_flex_dest + sum of
  // trailing flex durations. If all steps are flex, destination time = sum of
  // all flex durations.
  StepPartitionId destination_partition = path.back().destination.partition;
  int trailing_flex_duration = 0;
  int last_non_flex_dest = 0;  // 0 if all steps are flex
  for (int i = path.size() - 1; i >= 0; --i) {
    if (path[i].is_flex) {
      trailing_flex_duration += path[i].FlexDurationSeconds();
    } else {
      last_non_flex_dest = path[i].destination.time.seconds;
      destination_partition = path[i].destination.partition;
      break;
    }
  }
  TimeSinceServiceStart destination_time{
      last_non_flex_dest + trailing_flex_duration
  };

  return Step{
      StepEndpoint{
          first.origin.stop,
          first.origin.is_flex,
          origin_partition,
          origin_time,
          first.origin.trip
      },
      StepEndpoint{
          last.destination.stop,
          last.destination.is_flex,
          destination_partition,
          destination_time,
          last.destination.trip
      },
      is_flex
  };
}

std::vector<Step> CollapseStepsByTrip(const std::vector<Step>& steps) {
  std::vector<Step> collapsed;
  size_t si = 0;
  while (si < steps.size()) {
    size_t group_end = si + 1;
    while (group_end < steps.size() &&
           steps[group_end].destination.trip ==
               steps[si].destination.trip) {
      group_end++;
    }
    std::vector<Step> group(steps.begin() + si, steps.begin() + group_end);
    collapsed.push_back(ConsecutiveMergedSteps(group));
    si = group_end;
  }
  NormalizeConsecutiveSteps(collapsed);
  return collapsed;
}

void NormalizeConsecutiveSteps(std::vector<Step>& steps) {
  if (steps.empty()) {
    return;
  }

  // Find first non-flex step and sum leading flex durations.
  size_t first_non_flex = steps.size();
  int leading_flex_duration = 0;
  for (size_t i = 0; i < steps.size(); ++i) {
    if (steps[i].is_flex) {
      leading_flex_duration += steps[i].FlexDurationSeconds();
    } else {
      first_non_flex = i;
      break;
    }
  }

  // Calculate start time: arrive at first scheduled step, or start at 0 if all
  // flex.
  int current_time =
      (first_non_flex < steps.size())
          ? steps[first_non_flex].origin.time.seconds - leading_flex_duration
          : 0;

  // Normalize each flex step to have sequential times.
  for (size_t i = 0; i < steps.size(); ++i) {
    if (steps[i].is_flex) {
      int duration = steps[i].FlexDurationSeconds();
      steps[i].origin.time.seconds = current_time;
      steps[i].destination.time.seconds = current_time + duration;
    }
    current_time = steps[i].destination.time.seconds;
  }

  for (size_t i = 0; i < steps.size() - 1; ++i) {
    assert(steps[i].destination.stop == steps[i + 1].origin.stop);
    assert(steps[i].destination.time <= steps[i + 1].destination.time);
  }
}

std::optional<Step> SelectBestNextStep(
    const Step cur, const std::vector<Step>& candidates
) {
  if (candidates.size() == 0) {
    return std::nullopt;
  }

  std::optional<Step> best;
  int first_sched_step = 0;

  if (candidates[0].is_flex) {
    assert(candidates[0].origin.stop == cur.destination.stop);

    best = candidates[0];
    first_sched_step = 1;

    // Adjust this flex step to start when `cur` ends.
    int duration = best->FlexDurationSeconds();
    best->origin.time = cur.destination.time;
    best->destination.time =
        TimeSinceServiceStart{cur.destination.time.seconds + duration};
  }

  for (int i = first_sched_step; i < candidates.size(); ++i) {
    assert(candidates[i].origin.stop == cur.destination.stop);

    const Step& candidate = candidates[i];
    if (candidate.origin.time < cur.destination.time) {
      continue;
    }
    if (!best.has_value() ||
        candidate.destination.time < best->destination.time) {
      best = candidate;
    }
    // Because the steps are sorted and minimal, we can break after checking the
    // first achievable one.
    break;
  }

  return best;
}

}  // namespace vats5
