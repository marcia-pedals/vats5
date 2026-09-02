#include "step_merge.h"

#include <algorithm>
#include <cassert>
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

namespace {

// A sorted minimal vector of Steps as a cover view for MergeCovers.
struct StepsCoverView {
  const std::vector<Step>& steps;

  bool HasFlex() const { return !steps.empty() && steps[0].is_flex; }
  int FlexSeconds() const { return steps[0].FlexDurationSeconds(); }
  size_t NumScheduled() const { return steps.size() - (HasFlex() ? 1 : 0); }
  int Dep(size_t i) const { return At(i).origin.time.seconds; }
  int Arr(size_t i) const { return At(i).destination.time.seconds; }

  // The vector index of the flex step, or of scheduled step i.
  size_t Index(size_t i) const {
    return i == kMergeViaFlex ? 0 : i + (HasFlex() ? 1 : 0);
  }
  const Step& At(size_t i) const { return steps[Index(i)]; }
};
static_assert(CoverView<StepsCoverView>);

// Materializes MergeCovers' result as Steps, with provenance.
struct StepsSink {
  StepsCoverView ab;
  StepsCoverView bc;
  std::vector<Step>& result;
  std::vector<StepProvenance>* provenance;

  void Flex(int) { Add(kMergeViaFlex, kMergeViaFlex); }
  void Scheduled(int dep, int arr, size_t ab_i, size_t bc_i) {
    Add(ab_i, bc_i);
    assert(result.back().origin.time.seconds == dep);
    assert(result.back().destination.time.seconds == arr);
  }
  void Add(size_t ab_i, size_t bc_i) {
    result.push_back(MergedStep(ab.At(ab_i), bc.At(bc_i)));
    if (provenance) {
      provenance->push_back({ab.Index(ab_i), bc.Index(bc_i)});
    }
  }
};
static_assert(CoverSink<StepsSink>);

}  // namespace

std::vector<Step> PairwiseMergedSteps(
    const std::vector<Step>& ab,
    const std::vector<Step>& bc,
    std::vector<StepProvenance>* provenance
) {
  std::vector<Step> result;
  if (provenance) {
    provenance->clear();
  }
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

  StepsSink sink{
      .ab = StepsCoverView{ab},
      .bc = StepsCoverView{bc},
      .result = result,
      .provenance = provenance
  };
  MergeCovers(sink.ab, sink.bc, sink);

  // The sink received the scheduled steps from the latest departure down.
  size_t first_scheduled = (ab[0].is_flex && bc[0].is_flex) ? 1 : 0;
  std::reverse(result.begin() + first_scheduled, result.end());
  if (provenance) {
    std::reverse(provenance->begin() + first_scheduled, provenance->end());
  }
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
