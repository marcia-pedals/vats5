#include <fstream>
#include <iostream>
#include <set>
#include <sstream>

#include "tools/reduction_output.h"

using namespace vats5;

std::string FormatPath(const Path& path, const DataGtfsMapping& mapping) {
  std::stringstream ss;
  ss << "    [" << path.merged_step.origin_time.ToString() << " -> "
     << path.merged_step.destination_time.ToString() << "]";
  if (path.merged_step.is_flex) {
    ss << " (flex)";
  }
  ss << "\n";
  for (const auto& step : path.steps) {
    std::string origin_name = "?";
    std::string dest_name = "?";
    auto origin_it = mapping.stop_id_to_stop_name.find(step.origin_stop);
    if (origin_it != mapping.stop_id_to_stop_name.end()) {
      origin_name = origin_it->second;
    }
    auto dest_it = mapping.stop_id_to_stop_name.find(step.destination_stop);
    if (dest_it != mapping.stop_id_to_stop_name.end()) {
      dest_name = dest_it->second;
    }
    ss << "      " << origin_name << " (" << step.origin_stop.v << ") -> "
       << dest_name << " (" << step.destination_stop.v << ") ["
       << step.origin_time.ToString() << " -> "
       << step.destination_time.ToString() << "]";
    if (step.is_flex) {
      ss << " flex";
    }
    ss << "\n";
  }
  return ss.str();
}

bool PathsEqual(const Path& a, const Path& b) {
  if (a.steps.size() != b.steps.size()) return false;
  for (size_t i = 0; i < a.steps.size(); ++i) {
    if (!(a.steps[i] == b.steps[i])) return false;
  }
  return true;
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <file_a.json> <file_b.json>"
              << std::endl;
    return 1;
  }

  std::string file_a_path = argv[1];
  std::string file_b_path = argv[2];

  std::cout << "Loading file A: " << file_a_path << std::endl;
  std::ifstream file_a(file_a_path);
  if (!file_a) {
    std::cerr << "Error: Could not open file A: " << file_a_path << std::endl;
    return 1;
  }
  nlohmann::json json_a;
  file_a >> json_a;
  ReductionOutput output_a = json_a.get<ReductionOutput>();

  std::cout << "Loading file B: " << file_b_path << std::endl;
  std::ifstream file_b(file_b_path);
  if (!file_b) {
    std::cerr << "Error: Could not open file B: " << file_b_path << std::endl;
    return 1;
  }
  nlohmann::json json_b;
  file_b >> json_b;
  ReductionOutput output_b = json_b.get<ReductionOutput>();

  // Use mapping from file A for display (they should be the same)
  const DataGtfsMapping& mapping = output_a.mapping;

  // Collect all (origin, destination) pairs from both files
  std::set<std::pair<int, int>> all_pairs;

  auto get_destination = [](const std::vector<Path>& paths) -> StopId {
    if (paths.empty()) return StopId{-1};
    return paths[0].merged_step.destination_stop;
  };

  for (const auto& [origin, path_groups] : output_a.minimal.adjacent) {
    for (const auto& paths : path_groups) {
      StopId dest = get_destination(paths);
      if (dest.v >= 0) {
        all_pairs.insert({origin.v, dest.v});
      }
    }
  }

  for (const auto& [origin, path_groups] : output_b.minimal.adjacent) {
    for (const auto& paths : path_groups) {
      StopId dest = get_destination(paths);
      if (dest.v >= 0) {
        all_pairs.insert({origin.v, dest.v});
      }
    }
  }

  // Helper to find paths for a given (origin, dest) pair
  auto find_paths = [](const PathsAdjacencyList& adj, StopId origin, StopId dest
                    ) -> const std::vector<Path>* {
    auto it = adj.adjacent.find(origin);
    if (it == adj.adjacent.end()) return nullptr;
    for (const auto& paths : it->second) {
      if (!paths.empty() && paths[0].merged_step.destination_stop == dest) {
        return &paths;
      }
    }
    return nullptr;
  };

  int perfect_matches = 0;
  int mismatched_pairs = 0;
  int only_in_a = 0;
  int only_in_b = 0;

  std::stringstream mismatch_details;

  for (const auto& [origin_v, dest_v] : all_pairs) {
    StopId origin{origin_v};
    StopId dest{dest_v};

    const std::vector<Path>* paths_a =
        find_paths(output_a.minimal, origin, dest);
    const std::vector<Path>* paths_b =
        find_paths(output_b.minimal, origin, dest);

    std::string origin_name = "?";
    std::string dest_name = "?";
    std::string origin_gtfs = "?";
    std::string dest_gtfs = "?";

    auto origin_name_it = mapping.stop_id_to_stop_name.find(origin);
    if (origin_name_it != mapping.stop_id_to_stop_name.end()) {
      origin_name = origin_name_it->second;
    }
    auto dest_name_it = mapping.stop_id_to_stop_name.find(dest);
    if (dest_name_it != mapping.stop_id_to_stop_name.end()) {
      dest_name = dest_name_it->second;
    }
    auto origin_gtfs_it = mapping.stop_id_to_gtfs_stop_id.find(origin);
    if (origin_gtfs_it != mapping.stop_id_to_gtfs_stop_id.end()) {
      origin_gtfs = origin_gtfs_it->second.v;
    }
    auto dest_gtfs_it = mapping.stop_id_to_gtfs_stop_id.find(dest);
    if (dest_gtfs_it != mapping.stop_id_to_gtfs_stop_id.end()) {
      dest_gtfs = dest_gtfs_it->second.v;
    }

    if (!paths_a && paths_b) {
      only_in_b++;
      mismatch_details << "\n(" << origin_name << ", " << dest_name << ") "
                       << "(gtfs: " << origin_gtfs << ", " << dest_gtfs << ") "
                       << "(stop id: " << origin_v << ", " << dest_v << ")\n"
                       << "  Only in file B: " << paths_b->size() << " paths\n";
      for (const auto& path : *paths_b) {
        mismatch_details << FormatPath(path, mapping);
      }
    } else if (paths_a && !paths_b) {
      only_in_a++;
      mismatch_details << "\n(" << origin_name << ", " << dest_name << ") "
                       << "(gtfs: " << origin_gtfs << ", " << dest_gtfs << ") "
                       << "(stop id: " << origin_v << ", " << dest_v << ")\n"
                       << "  Only in file A: " << paths_a->size() << " paths\n";
      for (const auto& path : *paths_a) {
        mismatch_details << FormatPath(path, mapping);
      }
    } else if (paths_a && paths_b) {
      // Compare paths
      std::vector<const Path*> a_only;
      std::vector<const Path*> b_only;
      std::vector<const Path*> matching;

      std::vector<bool> b_matched(paths_b->size(), false);

      for (const auto& pa : *paths_a) {
        bool found = false;
        for (size_t i = 0; i < paths_b->size(); ++i) {
          if (!b_matched[i] && PathsEqual(pa, (*paths_b)[i])) {
            b_matched[i] = true;
            found = true;
            matching.push_back(&pa);
            break;
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
        mismatch_details << "\n(" << origin_name << ", " << dest_name << ") "
                         << "(gtfs: " << origin_gtfs << ", " << dest_gtfs
                         << ") "
                         << "(stop id: " << origin_v << ", " << dest_v << ")\n"
                         << "  " << matching.size() << " matching paths\n";

        if (!a_only.empty()) {
          mismatch_details << "  File A has " << a_only.size()
                           << " extra paths:\n";
          for (const auto* path : a_only) {
            mismatch_details << FormatPath(*path, mapping);
          }
        }

        if (!b_only.empty()) {
          mismatch_details << "  File B has " << b_only.size()
                           << " extra paths:\n";
          for (const auto* path : b_only) {
            mismatch_details << FormatPath(*path, mapping);
          }
        }
      }
    }
  }

  std::cout << "\n=== Summary ===" << std::endl;
  std::cout << perfect_matches
            << " (origin, destination) pairs have perfectly matching paths."
            << std::endl;
  std::cout << mismatched_pairs
            << " (origin, destination) pairs have mismatched paths."
            << std::endl;
  std::cout << only_in_a << " (origin, destination) pairs only in file A."
            << std::endl;
  std::cout << only_in_b << " (origin, destination) pairs only in file B."
            << std::endl;

  if (mismatched_pairs > 0 || only_in_a > 0 || only_in_b > 0) {
    std::cout << "\n=== Details ===" << std::endl;
    std::cout << mismatch_details.str() << std::endl;
  }

  return 0;
}
