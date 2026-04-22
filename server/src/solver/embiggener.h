#pragma once

#include <unordered_map>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"
namespace vats5 {

struct FlatStep {
  TimeSinceServiceStart origin_time;
  TimeSinceServiceStart destination_time;
  int DurationSeconds() const {
    return destination_time.seconds - origin_time.seconds;
  }
  bool operator==(const FlatStep&) const = default;
};

struct EmbiggenerEdge {
  std::vector<FlatStep> steps = {};
  int weight = 0;
  bool operator==(const EmbiggenerEdge&) const = default;
};

inline std::ostream& operator<<(std::ostream& os, const FlatStep& value) {
  return os << "FlatStep{" << value.origin_time.seconds << " -> "
            << value.destination_time.seconds << "}";
}

inline std::ostream& operator<<(std::ostream& os, const EmbiggenerEdge& value) {
  os << "EmbiggenerEdge{w=" << value.weight << ", steps=[";
  for (size_t i = 0; i < value.steps.size(); ++i) {
    if (i > 0) os << ", ";
    os << value.steps[i];
  }
  return os << "]}";
}

struct EmbiggenerState {
  RequiredStops required;
  std::unordered_map<PlainEdge, EmbiggenerEdge> edges;
};

struct PointBound {
  StopId s;
  TimeSinceServiceStart t_lo;
  TimeSinceServiceStart t_hi;
};

struct PointInstant {
  StopId s;
  TimeSinceServiceStart t;
  bool operator==(const PointInstant&) const = default;
};

struct StepId {
  StopId a;
  StopId b;
  TimeSinceServiceStart tb;
  bool operator==(const StepId&) const = default;
};

}  // namespace vats5

template <>
struct std::hash<vats5::PointInstant> {
  std::size_t operator()(const vats5::PointInstant& v) const {
    std::size_t seed = std::hash<vats5::StopId>{}(v.s);
    seed ^=
        std::hash<int>{}(v.t.seconds) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

template <>
struct std::hash<vats5::StepId> {
  std::size_t operator()(const vats5::StepId& v) const {
    std::size_t seed = std::hash<vats5::StopId>{}(v.a);
    seed ^= std::hash<vats5::StopId>{}(v.b) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    seed ^=
        std::hash<int>{}(v.tb.seconds) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

namespace vats5 {

struct EmbiggenerOptions {
  bool include_nonzero_flex = false;
};

EmbiggenerState MakeEmbiggenerState(
    const ProblemState& problem,
    const StepsAdjacencyList& completed,
    std::vector<PointBound> known_points,
    std::unordered_set<StepId> forbidden_steps,
    EmbiggenerOptions options = {}
);

// A perfectly cromulent function.
int Embiggen(int value);

}  // namespace vats5
