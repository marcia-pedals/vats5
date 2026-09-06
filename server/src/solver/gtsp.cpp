#include "solver/gtsp.h"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>
#include <unordered_map>

namespace vats5 {

Gtsp BuildGtsp(
    const StepsAdjacencyList& list,
    ProblemBoundary boundary,
    std::optional<TimeSinceServiceStart> start_time
) {
  Gtsp gtsp;
  gtsp.vertices.push_back({boundary.start, TimeSinceServiceStart{0}});
  gtsp.vertices.push_back({boundary.end, TimeSinceServiceStart{0}});

  std::unordered_map<int64_t, int> vertex_ids;
  auto vertex_id = [&](StopId stop, TimeSinceServiceStart time) {
    int64_t key = (static_cast<int64_t>(stop.v) << 32) |
                  static_cast<uint32_t>(time.seconds);
    auto [it, inserted] =
        vertex_ids.try_emplace(key, static_cast<int>(gtsp.vertices.size()));
    if (inserted) {
      gtsp.vertices.push_back({stop, time});
    }
    return it->second;
  };

  // First departure at or after `time` in `group`, if any. Steps in a group
  // are sorted by origin time.
  auto next_departure = [&](const StepGroup& group, TimeSinceServiceStart time)
      -> std::optional<TimeSinceServiceStart> {
    std::span<const AdjacencyListStep> steps = list.GetSteps(group);
    auto it = std::lower_bound(
        steps.begin(),
        steps.end(),
        time,
        [](const AdjacencyListStep& step, TimeSinceServiceStart t) {
          return step.origin_time.seconds < t.seconds;
        }
    );
    if (it == steps.end()) {
      return std::nullopt;
    }
    return it->origin_time;
  };

  std::vector<TimeSinceServiceStart> targets;
  for (int stop_v = 0; stop_v < list.NumStops(); ++stop_v) {
    StopId a{stop_v};
    for (const StepGroup& group : list.GetGroups(a)) {
      StopId b = group.destination_stop;
      for (const AdjacencyListStep& step : list.GetSteps(group)) {
        int from = vertex_id(a, step.origin_time);

        targets.clear();
        targets.push_back(step.destination_time);
        for (const StepGroup& onward : list.GetGroups(b)) {
          if (auto t = next_departure(onward, step.destination_time)) {
            targets.push_back(*t);
          }
        }
        std::sort(
            targets.begin(),
            targets.end(),
            [](TimeSinceServiceStart x, TimeSinceServiceStart y) {
              return x.seconds < y.seconds;
            }
        );
        targets.erase(
            std::unique(targets.begin(), targets.end()), targets.end()
        );

        for (TimeSinceServiceStart t : targets) {
          gtsp.edges.push_back(
              {.from = from,
               .to = vertex_id(b, t),
               .weight_seconds = t.seconds - step.origin_time.seconds,
               .travel_seconds =
                   step.destination_time.seconds - step.origin_time.seconds,
               .partition = step.destination_partition}
          );
        }
      }
    }
  }

  if (start_time) {
    // Being at stop b at start_time is like arriving there then: the tour
    // may take the first departure towards each onward stop.
    for (int stop_v = 0; stop_v < list.NumStops(); ++stop_v) {
      StopId b{stop_v};
      if (b == boundary.start || b == boundary.end) {
        continue;
      }
      targets.clear();
      for (const StepGroup& onward : list.GetGroups(b)) {
        if (auto t = next_departure(onward, *start_time)) {
          targets.push_back(*t);
        }
      }
      std::sort(
          targets.begin(),
          targets.end(),
          [](TimeSinceServiceStart x, TimeSinceServiceStart y) {
            return x.seconds < y.seconds;
          }
      );
      targets.erase(std::unique(targets.begin(), targets.end()), targets.end());
      for (TimeSinceServiceStart t : targets) {
        gtsp.edges.push_back(
            {Gtsp::kStart, vertex_id(b, t), t.seconds - start_time->seconds}
        );
      }
    }
  } else {
    for (int v = 2; v < static_cast<int>(gtsp.vertices.size()); ++v) {
      gtsp.edges.push_back({Gtsp::kStart, v, 0});
    }
  }
  for (int v = 2; v < static_cast<int>(gtsp.vertices.size()); ++v) {
    gtsp.edges.push_back({v, Gtsp::kEnd, 0});
  }
  gtsp.edges.push_back({Gtsp::kEnd, Gtsp::kStart, 0});

  return gtsp;
}

}  // namespace vats5
