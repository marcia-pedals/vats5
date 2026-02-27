#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "log.h"
#include "solver/data.h"
#include "solver/tarel_graph.h"

namespace vats5 {

void PrintPartitions(
    const ProblemState& state,
    const std::unordered_map<StepPartitionId, std::unordered_set<StopId>>&
        partitions,
    const TextLogger& log
);

void PrintStopPartitions(
    const ProblemState& state, StopId s, const TextLogger& log
);

void MyDetailedPrintout(
    const ProblemState& state,
    const std::vector<TarelEdge>& tour,
    const TextLogger& log
);

}  // namespace vats5
