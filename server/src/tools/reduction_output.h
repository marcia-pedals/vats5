#pragma once

#include <nlohmann/json.hpp>

#include "solver/data.h"
#include "solver/steps_shortest_path.h"

namespace vats5 {

struct ReductionOutput {
  StepPathsAdjacencyList minimal;
  DataGtfsMapping mapping;
};

inline void to_json(nlohmann::json& j, const ReductionOutput& output) {
  j = nlohmann::json{{"minimal", output.minimal}, {"mapping", output.mapping}};
}

inline void from_json(const nlohmann::json& j, ReductionOutput& output) {
  j.at("minimal").get_to(output.minimal);
  j.at("mapping").get_to(output.mapping);
}

}  // namespace vats5
