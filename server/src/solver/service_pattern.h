#pragma once

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "gtfs/gtfs.h"
#include "serialization/optional.h"
#include "solver/data.h"
#include "solver/steps_adjacency_list.h"

namespace vats5 {

// The GTFS services that a solve can use, per service day it spans: those
// running on its service date, then those running on each of the subsequent
// dates it pulls trips from (see GtfsFilterDateWithServiceDays).
//
// Two service dates whose active services are the same present the exact same
// problem, so the second one does not have to be solved again.
struct ActiveServices {
  // Indexed by service day, 0 being the service date itself. Each entry is
  // sorted and deduplicated, so that equality is set equality.
  std::vector<std::vector<GtfsServiceId>> by_service_day;

  bool operator==(const ActiveServices& other) const;

  // How many service days this spans, which is the subsequent day count plus
  // one. Patterns spanning different numbers of days never compare equal.
  int NumServiceDays() const { return static_cast<int>(by_service_day.size()); }
};
// Serialized as an object keyed the same way trip ids are suffixed:
// {"sd-0": [...], "sd-1": [...], ...}.
void to_json(nlohmann::json& j, const ActiveServices& s);
void from_json(const nlohmann::json& j, ActiveServices& s);

// The key ActiveServices is serialized under for the `day`'th service day.
std::string ActiveServicesJsonKey(int day);

// Which already-solved service date a date turned out to duplicate, and which
// set of services was found to be identical.
struct DuplicateOf {
  std::string service_date;
  std::string matched_on;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DuplicateOf, service_date, matched_on)

// What one service date's initialization records about the services it uses.
// Fed back into later dates as the `--solved-service-patterns` list.
struct ServicePattern {
  std::string service_date;

  // Every service running on any of the days the solve spans.
  ActiveServices all_services;

  // Only the services of trips that survive into the pruned steps. Absent when
  // initialization exited early, before the pruning was done.
  std::optional<ActiveServices> pruned_services;

  // Set only on the pattern written by a date that was skipped as a duplicate.
  // Patterns fed back in as previously-solved ones never have it.
  std::optional<DuplicateOf> duplicate_of;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ServicePattern, service_date, all_services, pruned_services, duplicate_of
)

// Exit code initialize_problem_state uses to report that it stopped early
// because this service date duplicates an already-solved one. Kept distinct
// from 1 so that callers can tell it apart from a real failure.
inline constexpr int kDuplicateServicePatternExitCode = 3;

// Where initialize_problem_state writes the service pattern for the problem
// state it was asked to write, e.g. "foo.json" -> "foo-active-services.json".
std::string ActiveServicesPath(const std::string& problem_state_path);

// Every service that has a trip in `gtfs_day`, split by which service day the
// trip belongs to. `num_service_days` is how many days the combined day spans,
// so that a day with no services at all still gets an (empty) entry.
ActiveServices GetAllActiveServices(
    const GtfsDay& gtfs_day, int num_service_days
);

// Every service that has a trip used by a step of `paths`. Steps on flex trips
// (walking) belong to no service and are ignored.
ActiveServices GetPrunedActiveServices(
    const StepPathsAdjacencyList& paths,
    const DataGtfsMapping& mapping,
    const GtfsDay& gtfs_day,
    int num_service_days
);

// Read a JSON list of patterns from already-solved service dates.
std::vector<ServicePattern> LoadServicePatterns(const std::string& path);

// Write one pattern as JSON.
void WriteServicePattern(
    const ServicePattern& pattern, const std::string& path
);

// The first pattern of `solved` with the same `all_services`, if any.
const ServicePattern* FindMatchingAllServices(
    const std::vector<ServicePattern>& solved, const ActiveServices& services
);

// The first pattern of `solved` with the same `pruned_services`, if any.
// Patterns with no pruned services recorded never match.
const ServicePattern* FindMatchingPrunedServices(
    const std::vector<ServicePattern>& solved, const ActiveServices& services
);

}  // namespace vats5
