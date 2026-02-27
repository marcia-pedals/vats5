#include "solver/debug_printing.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace vats5 {

void PrintPartitions(
    const ProblemState<>& state,
    const std::unordered_map<StepPartitionId, std::unordered_set<StopId<>>>&
        partitions
) {
  std::vector<std::pair<std::string, std::vector<std::string>>> entries;
  for (const auto& [partition, stops] : partitions) {
    std::vector<std::string> stop_names;
    for (StopId<> stop : stops) {
      stop_names.push_back(state.StopName(stop));
    }
    std::ranges::sort(stop_names);
    entries.emplace_back(state.PartitionName(partition), std::move(stop_names));
  }
  std::ranges::sort(entries, [](const auto& a, const auto& b) {
    return a.first < b.first;
  });
  for (const auto& [name, stop_names] : entries) {
    std::cout << "    " << name << " [";
    for (size_t i = 0; i < stop_names.size(); ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << stop_names[i];
    }
    std::cout << "]\n";
  }
}

void MyDetailedPrintout(
    const ProblemState<>& state, const std::vector<TarelEdge>& tour
) {
  std::vector<StopId<>> tour_stops;
  for (const TarelEdge& edge : tour) {
    tour_stops.push_back(edge.origin.stop);
  }
  if (tour.size() > 0) {
    tour_stops.push_back(tour.back().destination.stop);
  }

  for (StopId<> s : tour_stops) {
    PrintStopPartitions(state, s);
  }
}

void PrintStopPartitions(const ProblemState<>& state, StopId<> s) {
  std::cout << state.StopName(s) << "\n";

  std::unordered_map<StepPartitionId, std::unordered_set<StopId<>>>
      arrive_partitions;
  std::unordered_map<StepPartitionId, std::unordered_set<StopId<>>>
      depart_partitions;
  std::unordered_map<StepPartitionId, std::unordered_set<StopId<>>>
      arrive_partitions_flex;
  std::unordered_map<StepPartitionId, std::unordered_set<StopId<>>>
      depart_partitions_flex;
  for (const Step& step : state.minimal.AllSteps()) {
    if (step.destination.stop == s) {
      if (step.is_flex) {
        arrive_partitions_flex[step.destination.partition].insert(
            step.origin.stop
        );
      } else {
        arrive_partitions[step.destination.partition].insert(step.origin.stop);
      }
    }
    if (step.origin.stop == s) {
      if (step.is_flex) {
        depart_partitions_flex[step.destination.partition].insert(
            step.destination.stop
        );
      } else {
        depart_partitions[step.destination.partition].insert(
            step.destination.stop
        );
      }
    }
  }

  if (arrive_partitions.size() > 0) {
    std::cout << "  arrive partitions:\n";
    PrintPartitions(state, arrive_partitions);
  }
  if (arrive_partitions_flex.size() > 0) {
    std::cout << "  arrive partitions flex:\n";
    PrintPartitions(state, arrive_partitions_flex);
  }

  if (depart_partitions.size() > 0) {
    std::cout << "  depart partitions:\n";
    PrintPartitions(state, depart_partitions);
  }
  if (depart_partitions_flex.size() > 0) {
    std::cout << "  depart partitions flex:\n";
    PrintPartitions(state, depart_partitions_flex);
  }
}

}  // namespace vats5
