#include <CLI/CLI.hpp>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>

#include "tools/reduction_output.h"

using namespace vats5;

// GTFS-space versions of the data structures for comparison
struct GtfsStep {
  GtfsStopId origin_stop;
  GtfsStopId destination_stop;
  TimeSinceServiceStart origin_time;
  TimeSinceServiceStart destination_time;
  bool is_flex;

  int duration_seconds() const {
    return destination_time.seconds - origin_time.seconds;
  }

  bool operator==(const GtfsStep& other) const {
    if (origin_stop != other.origin_stop ||
        destination_stop != other.destination_stop ||
        is_flex != other.is_flex) {
      return false;
    }
    if (is_flex) {
      // For flex steps, only compare duration
      return duration_seconds() == other.duration_seconds();
    } else {
      return origin_time == other.origin_time &&
             destination_time == other.destination_time;
    }
  }

  bool operator<(const GtfsStep& other) const {
    if (origin_stop.v != other.origin_stop.v)
      return origin_stop.v < other.origin_stop.v;
    if (destination_stop.v != other.destination_stop.v)
      return destination_stop.v < other.destination_stop.v;
    if (origin_time.seconds != other.origin_time.seconds)
      return origin_time.seconds < other.origin_time.seconds;
    if (destination_time.seconds != other.destination_time.seconds)
      return destination_time.seconds < other.destination_time.seconds;
    return is_flex < other.is_flex;
  }
};

struct GtfsPath {
  GtfsStep merged_step;
  std::vector<GtfsStep> steps;

  bool operator==(const GtfsPath& other) const {
    if (steps.size() != other.steps.size()) return false;
    for (size_t i = 0; i < steps.size(); ++i) {
      if (!(steps[i] == other.steps[i])) return false;
    }
    return true;
  }

  // Check if two paths are "equivalent" - same origin, destination, and times
  bool IsEquivalentTo(const GtfsPath& other) const {
    return merged_step.origin_stop == other.merged_step.origin_stop &&
           merged_step.destination_stop == other.merged_step.destination_stop &&
           merged_step.origin_time == other.merged_step.origin_time &&
           merged_step.destination_time == other.merged_step.destination_time &&
           merged_step.is_flex == other.merged_step.is_flex;
  }

  bool operator<(const GtfsPath& other) const {
    if (merged_step < other.merged_step) return true;
    if (other.merged_step < merged_step) return false;
    return steps < other.steps;
  }
};

// Map from (origin gtfs stop, dest gtfs stop) -> paths
using GtfsPathsMap =
    std::map<std::pair<std::string, std::string>, std::vector<GtfsPath>>;

GtfsStep ToGtfsStep(const Step& step, const DataGtfsMapping& mapping) {
  GtfsStep gs;
  auto origin_it = mapping.stop_id_to_gtfs_stop_id.find(step.origin.stop);
  if (origin_it != mapping.stop_id_to_gtfs_stop_id.end()) {
    gs.origin_stop = origin_it->second;
  } else {
    gs.origin_stop = GtfsStopId{"?" + std::to_string(step.origin.stop.v)};
  }
  auto dest_it = mapping.stop_id_to_gtfs_stop_id.find(step.destination.stop);
  if (dest_it != mapping.stop_id_to_gtfs_stop_id.end()) {
    gs.destination_stop = dest_it->second;
  } else {
    gs.destination_stop =
        GtfsStopId{"?" + std::to_string(step.destination.stop.v)};
  }
  gs.origin_time = step.origin.time;
  gs.destination_time = step.destination.time;
  gs.is_flex = step.is_flex;
  return gs;
}

GtfsPath ToGtfsPath(const Path& path, const DataGtfsMapping& mapping) {
  GtfsPath gp;
  gp.merged_step = ToGtfsStep(path.merged_step, mapping);
  for (const auto& step : path.steps) {
    gp.steps.push_back(ToGtfsStep(step, mapping));
  }
  return gp;
}

GtfsPathsMap ToGtfsPathsMap(
    const StepPathsAdjacencyList& adj, const DataGtfsMapping& mapping
) {
  GtfsPathsMap result;
  for (const auto& [origin, path_groups] : adj.adjacent) {
    auto origin_gtfs_it = mapping.stop_id_to_gtfs_stop_id.find(origin);
    if (origin_gtfs_it == mapping.stop_id_to_gtfs_stop_id.end()) continue;
    std::string origin_gtfs = origin_gtfs_it->second.v;

    for (const auto& paths : path_groups) {
      if (paths.empty()) continue;
      auto dest_gtfs_it = mapping.stop_id_to_gtfs_stop_id.find(
          paths[0].merged_step.destination.stop
      );
      if (dest_gtfs_it == mapping.stop_id_to_gtfs_stop_id.end()) continue;
      std::string dest_gtfs = dest_gtfs_it->second.v;

      auto key = std::make_pair(origin_gtfs, dest_gtfs);
      for (const auto& path : paths) {
        result[key].push_back(ToGtfsPath(path, mapping));
      }
    }
  }
  return result;
}

// Map from GTFS stop ID to stop name
using GtfsStopNameMap = std::unordered_map<std::string, std::string>;

GtfsStopNameMap BuildGtfsStopNameMap(const DataGtfsMapping& mapping) {
  GtfsStopNameMap result;
  for (const auto& [stop_id, gtfs_stop_id] : mapping.stop_id_to_gtfs_stop_id) {
    auto name_it = mapping.stop_id_to_stop_name.find(stop_id);
    if (name_it != mapping.stop_id_to_stop_name.end()) {
      result[gtfs_stop_id.v] = name_it->second;
    }
  }
  return result;
}

std::string GetStopDisplay(
    const std::string& gtfs_id, const GtfsStopNameMap& names
) {
  auto it = names.find(gtfs_id);
  if (it != names.end()) {
    return it->second + " (" + gtfs_id + ")";
  }
  return gtfs_id;
}

std::string FormatStepDiff(
    const GtfsStep& a, const GtfsStep& b, const GtfsStopNameMap& names
) {
  std::stringstream ss;
  if (a.origin_stop.v != b.origin_stop.v) {
    ss << "origin stop: " << GetStopDisplay(a.origin_stop.v, names) << " vs "
       << GetStopDisplay(b.origin_stop.v, names);
  } else if (a.destination_stop.v != b.destination_stop.v) {
    ss << "destination stop: " << GetStopDisplay(a.destination_stop.v, names)
       << " vs " << GetStopDisplay(b.destination_stop.v, names);
  } else if (a.is_flex != b.is_flex) {
    ss << "is_flex: " << (a.is_flex ? "true" : "false") << " vs "
       << (b.is_flex ? "true" : "false");
  } else if (a.is_flex && b.is_flex) {
    // For flex steps, compare duration
    if (a.duration_seconds() != b.duration_seconds()) {
      ss << "duration: " << a.duration_seconds() << "s vs "
         << b.duration_seconds() << "s";
    }
  } else {
    // Non-flex steps: compare absolute times
    if (a.origin_time != b.origin_time) {
      ss << "origin time: " << a.origin_time.ToString() << " vs "
         << b.origin_time.ToString();
    } else if (a.destination_time != b.destination_time) {
      ss << "destination time: " << a.destination_time.ToString() << " vs "
         << b.destination_time.ToString();
    }
  }
  return ss.str();
}

std::string FindFirstDifference(
    const GtfsPath& a, const GtfsPath& b, const GtfsStopNameMap& names
) {
  std::stringstream ss;
  if (a.steps.size() != b.steps.size()) {
    ss << "  First difference: path length (" << a.steps.size() << " steps vs "
       << b.steps.size() << " steps)\n";
    return ss.str();
  }
  for (size_t i = 0; i < a.steps.size(); ++i) {
    if (!(a.steps[i] == b.steps[i])) {
      ss << "  First difference at step " << i << ": "
         << FormatStepDiff(a.steps[i], b.steps[i], names) << "\n";
      return ss.str();
    }
  }
  return "";
}

std::string FormatGtfsPath(const GtfsPath& path, const GtfsStopNameMap& names) {
  std::stringstream ss;
  ss << "    [" << path.merged_step.origin_time.ToString() << " -> "
     << path.merged_step.destination_time.ToString() << "]";
  if (path.merged_step.is_flex) {
    ss << " (flex)";
  }
  ss << "\n";
  for (const auto& step : path.steps) {
    ss << "      " << GetStopDisplay(step.origin_stop.v, names) << " -> "
       << GetStopDisplay(step.destination_stop.v, names) << " ["
       << step.origin_time.ToString() << " -> "
       << step.destination_time.ToString() << "]";
    if (step.is_flex) {
      ss << " flex";
    }
    ss << "\n";
  }
  return ss.str();
}

struct LoadedReductionOutput {
  GtfsPathsMap paths;
  GtfsStopNameMap stop_names;
};

std::optional<LoadedReductionOutput> LoadReductionOutput(
    const std::string& path
) {
  std::cout << "Loading: " << path << std::endl;
  std::ifstream file(path);
  if (!file) {
    std::cerr << "Error: Could not open file: " << path << std::endl;
    return std::nullopt;
  }
  nlohmann::json json;
  file >> json;
  ReductionOutput output = json.get<ReductionOutput>();

  std::cout << "Converting to GTFS space..." << std::endl;
  LoadedReductionOutput result;
  result.paths = ToGtfsPathsMap(output.minimal, output.mapping);
  result.stop_names = BuildGtfsStopNameMap(output.mapping);
  return result;
}

int main(int argc, char* argv[]) {
  CLI::App app{"Compare two reduction output files"};

  std::string file_a;
  std::string file_b;
  bool match_equivalent_paths = false;

  app.add_option("file_a", file_a, "First JSON file to compare")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_option("file_b", file_b, "Second JSON file to compare")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_flag(
      "--match-equivalent-paths",
      match_equivalent_paths,
      "Treat paths with same start/end times as matching, even if "
      "intermediate stops differ"
  );

  CLI11_PARSE(app, argc, argv);

  auto future_a = std::async(std::launch::async, LoadReductionOutput, file_a);
  auto future_b = std::async(std::launch::async, LoadReductionOutput, file_b);

  auto loaded_a = future_a.get();
  if (!loaded_a) return 1;

  auto loaded_b = future_b.get();
  if (!loaded_b) return 1;

  // Build combined stop name map from both files
  GtfsStopNameMap stop_names = loaded_a->stop_names;
  for (const auto& [k, v] : loaded_b->stop_names) {
    if (stop_names.find(k) == stop_names.end()) {
      stop_names[k] = v;
    }
  }

  // Collect all (origin, destination) pairs from both files
  std::set<std::pair<std::string, std::string>> all_pairs;
  for (const auto& [key, _] : loaded_a->paths) {
    all_pairs.insert(key);
  }
  for (const auto& [key, _] : loaded_b->paths) {
    all_pairs.insert(key);
  }

  int perfect_matches = 0;
  int mismatched_pairs = 0;
  int only_in_a = 0;
  int only_in_b = 0;
  int total_extra_paths_a = 0;
  int total_extra_paths_b = 0;

  std::stringstream mismatch_details;

  for (const auto& [origin_gtfs, dest_gtfs] : all_pairs) {
    auto it_a = loaded_a->paths.find({origin_gtfs, dest_gtfs});
    auto it_b = loaded_b->paths.find({origin_gtfs, dest_gtfs});

    const std::vector<GtfsPath>* paths_a =
        (it_a != loaded_a->paths.end()) ? &it_a->second : nullptr;
    const std::vector<GtfsPath>* paths_b =
        (it_b != loaded_b->paths.end()) ? &it_b->second : nullptr;

    std::string origin_display = GetStopDisplay(origin_gtfs, stop_names);
    std::string dest_display = GetStopDisplay(dest_gtfs, stop_names);

    if (!paths_a && paths_b) {
      only_in_b++;
      total_extra_paths_b += paths_b->size();
      mismatch_details << "\n"
                       << origin_display << " -> " << dest_display << "\n"
                       << "  Only in file B: " << paths_b->size() << " paths\n";
      for (const auto& path : *paths_b) {
        mismatch_details << FormatGtfsPath(path, stop_names);
      }
    } else if (paths_a && !paths_b) {
      only_in_a++;
      total_extra_paths_a += paths_a->size();
      mismatch_details << "\n"
                       << origin_display << " -> " << dest_display << "\n"
                       << "  Only in file A: " << paths_a->size() << " paths\n";
      for (const auto& path : *paths_a) {
        mismatch_details << FormatGtfsPath(path, stop_names);
      }
    } else if (paths_a && paths_b) {
      // Compare paths
      std::vector<const GtfsPath*> a_only;
      std::vector<const GtfsPath*> b_only;
      int matching_count = 0;

      std::vector<bool> b_matched(paths_b->size(), false);

      for (const auto& pa : *paths_a) {
        bool found = false;
        for (size_t i = 0; i < paths_b->size(); ++i) {
          if (!b_matched[i]) {
            bool matches = match_equivalent_paths
                               ? pa.IsEquivalentTo((*paths_b)[i])
                               : pa == (*paths_b)[i];
            if (matches) {
              b_matched[i] = true;
              found = true;
              matching_count++;
              break;
            }
          }
        }
        if (!found) {
          a_only.push_back(&pa);
        }
      }

      for (size_t i = 0; i < paths_b->size(); ++i) {
        if (!b_matched[i]) {
          b_only.push_back(&((*paths_b)[i]));
        }
      }

      if (a_only.empty() && b_only.empty()) {
        perfect_matches++;
      } else {
        mismatched_pairs++;
        total_extra_paths_a += a_only.size();
        total_extra_paths_b += b_only.size();
        mismatch_details << "\n"
                         << origin_display << " -> " << dest_display << "\n"
                         << "  " << matching_count << " matching paths\n";

        if (a_only.size() == 1 && b_only.size() == 1) {
          // Special case: highlight the first difference
          mismatch_details << "  File A extra path:\n";
          mismatch_details << FormatGtfsPath(*a_only[0], stop_names);
          mismatch_details << "  File B extra path:\n";
          mismatch_details << FormatGtfsPath(*b_only[0], stop_names);
          mismatch_details << FindFirstDifference(
              *a_only[0], *b_only[0], stop_names
          );
        } else {
          if (!a_only.empty()) {
            mismatch_details << "  File A has " << a_only.size()
                             << " extra paths:\n";
            for (const auto* path : a_only) {
              mismatch_details << FormatGtfsPath(*path, stop_names);
            }
          }

          if (!b_only.empty()) {
            mismatch_details << "  File B has " << b_only.size()
                             << " extra paths:\n";
            for (const auto* path : b_only) {
              mismatch_details << FormatGtfsPath(*path, stop_names);
            }
          }
        }
      }
    }
  }

  if (mismatched_pairs > 0 || only_in_a > 0 || only_in_b > 0) {
    std::cout << "\n=== Details ===" << std::endl;
    std::cout << mismatch_details.str() << std::endl;
  }

  std::cout << "\n=== Summary ===" << std::endl;
  std::cout << perfect_matches
            << " (origin -> destination) pairs have perfectly matching paths."
            << std::endl;
  std::cout << mismatched_pairs
            << " (origin -> destination) pairs have mismatched paths."
            << std::endl;
  std::cout << only_in_a << " (origin -> destination) pairs only in file A."
            << std::endl;
  std::cout << only_in_b << " (origin -> destination) pairs only in file B."
            << std::endl;
  std::cout << total_extra_paths_a << " total extra paths in file A."
            << std::endl;
  std::cout << total_extra_paths_b << " total extra paths in file B."
            << std::endl;

  return 0;
}
