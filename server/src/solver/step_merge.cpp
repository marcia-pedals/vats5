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
    
    // Backwards sweep: a step is dominated if there's a later-departing step that arrives earlier
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

std::vector<Step> MergeSteps(const std::vector<Step>& ab, const std::vector<Step>& bc) {
    std::vector<Step> result;
    
    if (ab.empty() || bc.empty()) {
        return result;
    }
    
    // Two-sliding pointer approach to find dominating pairs
    // ab is sorted by origin_time ascending, bc is sorted by origin_time ascending
    
    size_t j = 0;  // pointer for bc
    
    for (size_t i = 0; i < ab.size(); i++) {
        const Step& step_ab = ab[i];
        
        // Find the first step in bc that we can take after step_ab
        // For simplicity, assume no minimum connection time for now
        // (the step must depart at or after step_ab arrives)
        while (j < bc.size() && bc[j].origin_time.seconds < step_ab.destination_time.seconds) {
            j++;
        }
        
        if (j >= bc.size()) {
            // No more valid connections possible
            break;
        }
        
        // Create the combined step from A to C
        Step combined_step;
        combined_step.origin_stop = step_ab.origin_stop;
        combined_step.destination_stop = bc[j].destination_stop;
        combined_step.origin_time = step_ab.origin_time;
        combined_step.destination_time = bc[j].destination_time;
        combined_step.origin_trip = step_ab.origin_trip;
        combined_step.destination_trip = bc[j].destination_trip;
        
        // Check if this step dominates any previous steps or is dominated by them
        bool is_dominated = false;
        auto it = result.begin();
        while (it != result.end()) {
            // A step dominates another if it departs no earlier and arrives no later
            if (combined_step.origin_time.seconds >= it->origin_time.seconds &&
                combined_step.destination_time.seconds <= it->destination_time.seconds) {
                // combined_step is dominated by *it
                is_dominated = true;
                break;
            } else if (it->origin_time.seconds >= combined_step.origin_time.seconds &&
                       it->destination_time.seconds <= combined_step.destination_time.seconds) {
                // *it is dominated by combined_step, remove it
                it = result.erase(it);
            } else {
                ++it;
            }
        }
        
        if (!is_dominated) {
            result.push_back(combined_step);
        }
    }
    
    return result;
}

}  // namespace vats5