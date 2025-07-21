#include "step_merge.h"

#include <algorithm>
#include <limits>

namespace vats5 {

void SortByOriginAndDestinationTime(std::vector<Step>& steps) {
  std::sort(steps.begin(), steps.end(), [](const Step& a, const Step& b) {
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

  // Mark dominated steps for deletion using backwards sweep
  std::vector<bool> to_delete(steps.size(), false);
  int earliest_destination_time = std::numeric_limits<int>::max();

  // Backwards sweep: a step is dominated if there's a later-departing step that
  // arrives earlier
  for (int i = static_cast<int>(steps.size()) - 1; i >= 0; i--) {
    if (steps[i].destination_time.seconds >= earliest_destination_time) {
      // This step is dominated (arrives no earlier than a later-departing step)
      to_delete[i] = true;
    } else {
      // This step is not dominated, update earliest destination time
      earliest_destination_time = steps[i].destination_time.seconds;
    }
  }

  // Remove marked steps in-place using two-pointer technique
  size_t write_pos = 0;
  for (size_t read_pos = 0; read_pos < steps.size(); read_pos++) {
    if (!to_delete[read_pos]) {
      if (write_pos != read_pos) {
        steps[write_pos] = std::move(steps[read_pos]);
      }
      write_pos++;
    }
  }

  steps.resize(write_pos);
}

bool CheckSortedAndMinimal(const std::vector<Step>& steps) {
  for (size_t i = 1; i < steps.size(); i++) {
    const Step& prev = steps[i - 1];
    const Step& curr = steps[i];
    if (curr.origin_time.seconds <= prev.origin_time.seconds) {
      return false;
    }
    if (curr.destination_time.seconds <= prev.destination_time.seconds) {
      return false;
    }
  }
  return true;
}

std::vector<Step> MergeSteps(const std::vector<Step>& ab,
                             const std::vector<Step>& bc) {
  std::vector<Step> result;
  size_t ab_idx = 0;
  size_t bc_idx = 0;

  while (ab_idx < ab.size()) {
    // Advance bc_idx to the first step that can be reached by ab_idx.
    while (bc_idx < bc.size() && bc[bc_idx].origin_time.seconds <
                                     ab[ab_idx].destination_time.seconds) {
      bc_idx += 1;
    }

    // If we reached the end of bc, we are done.
    if (bc_idx == bc.size()) {
      return result;
    }

    // Advance ab_idx to the last ab step that arrives in time to catch bc_idx.
    while (ab_idx < ab.size() - 1 && ab[ab_idx + 1].destination_time.seconds <=
                                         bc[bc_idx].origin_time.seconds) {
      ab_idx += 1;
    }

    result.emplace_back(Step{
        ab[ab_idx].origin_stop,
        bc[bc_idx].destination_stop,
        ab[ab_idx].origin_time,
        bc[bc_idx].destination_time,
        ab[ab_idx].origin_trip,
        bc[bc_idx].destination_trip,
    });

    ab_idx += 1;
  }

  return result;
}

}  // namespace vats5