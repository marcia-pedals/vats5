#include "solver/service_pattern.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "gtfs/gtfs_filter.h"

namespace vats5 {

namespace {

// Sorted, deduplicated service ids, so that two ActiveServices holding the
// same services compare equal however they were collected.
std::vector<GtfsServiceId> Normalize(
    const std::unordered_set<std::string>& service_ids
) {
  std::vector<std::string> sorted(service_ids.begin(), service_ids.end());
  std::ranges::sort(sorted);
  std::vector<GtfsServiceId> result;
  result.reserve(sorted.size());
  for (std::string& service_id : sorted) {
    result.push_back(GtfsServiceId{std::move(service_id)});
  }
  return result;
}

// Which service each trip of `gtfs_day` runs under.
std::unordered_map<GtfsTripId, GtfsServiceId> TripToService(
    const GtfsDay& gtfs_day
) {
  std::unordered_map<GtfsTripId, GtfsServiceId> result;
  for (const GtfsTrip& trip : gtfs_day.trips) {
    result[trip.trip_id] = trip.service_id;
  }
  return result;
}

// Collects service ids into the service day they belong to.
struct ServiceCollector {
  std::vector<std::unordered_set<std::string>> by_service_day;

  explicit ServiceCollector(int num_service_days)
      : by_service_day(num_service_days) {}

  void Add(const GtfsTripId& trip_id, const GtfsServiceId& service_id) {
    int day = ServiceDayFromTripId(trip_id);
    if (day >= static_cast<int>(by_service_day.size())) {
      throw std::runtime_error(
          "Trip '" + trip_id.v + "' is on service day " + std::to_string(day) +
          ", beyond the " + std::to_string(by_service_day.size()) +
          " days this solve spans"
      );
    }
    by_service_day[day].insert(service_id.v);
  }

  ActiveServices Build() const {
    ActiveServices result;
    for (const std::unordered_set<std::string>& day : by_service_day) {
      result.by_service_day.push_back(Normalize(day));
    }
    return result;
  }
};

}  // namespace

bool ActiveServices::operator==(const ActiveServices& other) const {
  return by_service_day == other.by_service_day;
}

std::string ActiveServicesJsonKey(int day) {
  return day == 0 ? "cur_sd" : "next_sd_" + std::to_string(day);
}

void to_json(nlohmann::json& j, const ActiveServices& s) {
  j = nlohmann::json::object();
  for (int day = 0; day < s.NumServiceDays(); ++day) {
    j[ActiveServicesJsonKey(day)] = s.by_service_day[day];
  }
}

void from_json(const nlohmann::json& j, ActiveServices& s) {
  s.by_service_day.clear();
  // Days run consecutively from the service date, so the first missing key is
  // the end: a pattern recorded with fewer days stays shorter, and so never
  // compares equal to one recorded with more.
  for (int day = 0;; ++day) {
    auto it = j.find(ActiveServicesJsonKey(day));
    if (it == j.end()) {
      break;
    }
    s.by_service_day.push_back(it->get<std::vector<GtfsServiceId>>());
  }
  if (s.by_service_day.size() != j.size()) {
    throw std::runtime_error(
        "Active services has " + std::to_string(j.size()) + " keys but only " +
        std::to_string(s.by_service_day.size()) +
        " consecutive service days: " + j.dump()
    );
  }
}

std::string ActiveServicesPath(const std::string& problem_state_path) {
  size_t dot_pos = problem_state_path.rfind('.');
  if (dot_pos != std::string::npos) {
    return problem_state_path.substr(0, dot_pos) + "-active-services.json";
  }
  return problem_state_path + "-active-services.json";
}

ActiveServices GetAllActiveServices(
    const GtfsDay& gtfs_day, int num_service_days
) {
  ServiceCollector collector(num_service_days);
  for (const GtfsTrip& trip : gtfs_day.trips) {
    collector.Add(trip.trip_id, trip.service_id);
  }
  return collector.Build();
}

ActiveServices GetPrunedActiveServices(
    const StepPathsAdjacencyList& paths,
    const DataGtfsMapping& mapping,
    const GtfsDay& gtfs_day,
    int num_service_days
) {
  std::unordered_map<GtfsTripId, GtfsServiceId> trip_to_service =
      TripToService(gtfs_day);

  ServiceCollector collector(num_service_days);
  std::unordered_set<TripId> seen;
  auto add_trip = [&](TripId trip) {
    if (!seen.insert(trip).second) {
      return;
    }
    auto info_it = mapping.trip_id_to_trip_info.find(trip);
    if (info_it == mapping.trip_id_to_trip_info.end()) {
      throw std::runtime_error(
          "Step uses TripId " + std::to_string(trip.v) +
          ", which has no trip info"
      );
    }
    // Flex trips (walking) run every day and belong to no service.
    if (!std::holds_alternative<GtfsTripId>(info_it->second.v)) {
      return;
    }
    const GtfsTripId& gtfs_trip_id = std::get<GtfsTripId>(info_it->second.v);
    auto service_it = trip_to_service.find(gtfs_trip_id);
    if (service_it == trip_to_service.end()) {
      throw std::runtime_error(
          "Step uses trip '" + gtfs_trip_id.v +
          "', which is not in the GTFS day"
      );
    }
    collector.Add(gtfs_trip_id, service_it->second);
  };

  for (const auto& [origin_stop, path_groups] : paths.adjacent) {
    for (const std::vector<Path>& path_group : path_groups) {
      for (const Path& path : path_group) {
        for (const Step& step : path.steps) {
          add_trip(step.origin.trip);
          add_trip(step.destination.trip);
        }
      }
    }
  }
  return collector.Build();
}

std::vector<ServicePattern> LoadServicePatterns(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error(
        "Failed to open service patterns file '" + path + "'"
    );
  }
  nlohmann::json j;
  in >> j;
  return j.get<std::vector<ServicePattern>>();
}

void WriteServicePattern(
    const ServicePattern& pattern, const std::string& path
) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error(
        "Failed to write service pattern to '" + path + "'"
    );
  }
  out << nlohmann::json(pattern).dump();
}

const ServicePattern* FindMatchingAllServices(
    const std::vector<ServicePattern>& solved, const ActiveServices& services
) {
  for (const ServicePattern& pattern : solved) {
    if (pattern.all_services == services) {
      return &pattern;
    }
  }
  return nullptr;
}

const ServicePattern* FindMatchingPrunedServices(
    const std::vector<ServicePattern>& solved, const ActiveServices& services
) {
  for (const ServicePattern& pattern : solved) {
    if (pattern.pruned_services.has_value() &&
        *pattern.pruned_services == services) {
      return &pattern;
    }
  }
  return nullptr;
}

}  // namespace vats5
