#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "gtfs/gtfs.h"
#include "util/date.h"

using namespace vats5;

template <typename T>
std::string formatGtfsSizes(const T& gtfs) {
  std::ostringstream oss;
  oss << gtfs.stops.size() << " stops, " << gtfs.trips.size() << " trips, "
      << gtfs.stop_times.size() << " stop times, " << gtfs.routes.size()
      << " routes, " << gtfs.directions.size() << " directions";
  return oss.str();
}

// Removes trips not referenced by stop_times, then removes routes and
// directions not referenced by remaining trips
void RemoveUnreferencedTripsRoutesAndDirections(GtfsDay& gtfs_day) {
  // Collect trip_ids referenced by stop_times
  std::unordered_set<GtfsTripId> referenced_trip_ids;
  for (const auto& st : gtfs_day.stop_times) {
    referenced_trip_ids.insert(st.trip_id);
  }

  // Remove unreferenced trips and collect used route_direction_ids
  std::unordered_set<GtfsRouteDirectionId> used_route_direction_ids;
  std::unordered_set<GtfsRouteId> used_route_ids;
  std::erase_if(gtfs_day.trips, [&](const GtfsTrip& trip) {
    if (referenced_trip_ids.count(trip.trip_id)) {
      used_route_direction_ids.insert(trip.route_direction_id);
      used_route_ids.insert(trip.route_direction_id.route_id);
      return false;
    }
    return true;
  });

  // Remove unreferenced routes
  std::erase_if(gtfs_day.routes, [&](const GtfsRoute& route) {
    return !used_route_ids.count(route.route_id);
  });

  // Remove unreferenced directions
  std::erase_if(gtfs_day.directions, [&](const GtfsDirection& dir) {
    return !used_route_direction_ids.count(dir.route_direction_id);
  });
}

void printUsage(const char* program_name) {
  std::cout
      << "Usage: " << program_name
      << " [--prefix <filter>] <input_dir> <date> <output_dir>\n"
      << "\n"
      << "GTFS Filter Tool - Filters GTFS data by date and optionally by trip "
         "ID prefixes\n"
      << "\n"
      << "Arguments:\n"
      << "  input_dir    Directory containing GTFS files (stops.txt, "
         "trips.txt, etc.)\n"
      << "  date         Date in YYYYMMDD format (e.g., 20250708)\n"
      << "  output_dir   Directory where filtered GTFS files will be saved\n"
      << "\n"
      << "Options:\n"
      << "  --prefix <filter>  Filter by trip ID prefix (comma-separated "
         "list)\n"
      << "                     Example: \"CT:,SR:\" (includes all Caltrain and "
         "Santa Rosa trips)\n"
      << "                     If not specified, all trips are included.\n"
      << "  --help, -h         Show this help message\n"
      << "\n"
      << "Examples:\n"
      << "  # Include all trips for the date\n"
      << "  " << program_name << " ../data/RG 20250708 ./filtered_gtfs\n"
      << "\n"
      << "  # Filter all Caltrain trips (CT: prefix)\n"
      << "  " << program_name
      << " --prefix \"CT:\" ../data/RG 20250708 ./filtered_gtfs\n"
      << "\n"
      << "  # Filter all Caltrain and Santa Rosa trips\n"
      << "  " << program_name
      << " --prefix \"CT:,SR:\" ../data/RG 20250708 ./filtered_gtfs\n"
      << std::endl;
}

std::vector<std::string> split(const std::string& str, char delimiter) {
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;

  while (std::getline(ss, token, delimiter)) {
    // Trim whitespace
    size_t start = token.find_first_not_of(" \t\r\n");
    size_t end = token.find_last_not_of(" \t\r\n");

    if (start != std::string::npos && end != std::string::npos) {
      tokens.push_back(token.substr(start, end - start + 1));
    } else if (start != std::string::npos) {
      tokens.push_back(token.substr(start));
    }
  }

  return tokens;
}

bool isValidDate(const std::string& date) {
  if (date.length() != 8) return false;

  for (char c : date) {
    if (c < '0' || c > '9') return false;
  }

  // Basic validation - could be more thorough
  int year = std::stoi(date.substr(0, 4));
  int month = std::stoi(date.substr(4, 2));
  int day = std::stoi(date.substr(6, 2));

  return year >= 1900 && year <= 2100 && month >= 1 && month <= 12 &&
         day >= 1 && day <= 31;
}

int main(int argc, char* argv[]) {
  // Handle help first
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      printUsage(argv[0]);
      return 0;
    }
  }

  // Parse options and positional arguments
  std::string trip_filter;
  std::vector<std::string> positional_args;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--prefix") {
      if (i + 1 >= argc) {
        std::cerr << "Error: --prefix requires an argument." << std::endl;
        std::cerr << "Use --help for usage information." << std::endl;
        return 1;
      }
      trip_filter = argv[++i];
    } else if (arg.substr(0, 2) == "--") {
      std::cerr << "Error: Unknown option '" << arg << "'" << std::endl;
      std::cerr << "Use --help for usage information." << std::endl;
      return 1;
    } else {
      positional_args.push_back(arg);
    }
  }

  // Check that we have exactly 3 positional arguments
  if (positional_args.size() != 3) {
    std::cerr << "Error: Expected 3 arguments, got " << positional_args.size()
              << std::endl;
    std::cerr << "Use --help for usage information." << std::endl;
    return 1;
  }

  std::string input_dir = positional_args[0];
  std::string date = positional_args[1];
  std::string output_dir = positional_args[2];

  // Validate arguments
  if (!std::filesystem::exists(input_dir)) {
    std::cerr << "Error: Input directory '" << input_dir << "' does not exist."
              << std::endl;
    return 1;
  }

  if (!std::filesystem::is_directory(input_dir)) {
    std::cerr << "Error: Input path '" << input_dir << "' is not a directory."
              << std::endl;
    return 1;
  }

  if (!isValidDate(date)) {
    std::cerr << "Error: Invalid date format '" << date
              << "'. Expected YYYYMMDD format." << std::endl;
    return 1;
  }

  std::cout << "Loading GTFS data from: " << input_dir << std::endl;
  std::cout << "Filtering for date: " << date << std::endl;
  if (trip_filter.empty()) {
    std::cout << "Including all trips (no prefix filter)" << std::endl;
  } else {
    std::vector<std::string> prefix_list = split(trip_filter, ',');
    std::cout << "Using prefix filter(s): ";
    for (size_t i = 0; i < prefix_list.size(); ++i) {
      std::cout << "\"" << prefix_list[i] << "\"";
      if (i < prefix_list.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << "Output directory: " << output_dir << std::endl;
  std::cout << std::endl;

  try {
    // Load GTFS data
    std::cout << "Loading GTFS data..." << std::endl;
    Gtfs gtfs = GtfsLoad(input_dir);

    std::cout << "Loaded: " << formatGtfsSizes(gtfs) << std::endl;

    // Filter by prefix.
    if (!trip_filter.empty()) {
      std::unordered_set<GtfsTripId> trip_ids_set;
      std::vector<std::string> prefix_list = split(trip_filter, ',');
      for (const auto& trip : gtfs.trips) {
        for (const auto& prefix : prefix_list) {
          if (trip.trip_id.v.substr(0, prefix.length()) == prefix) {
            trip_ids_set.insert(trip.trip_id);
            break;
          }
        }
      }
      gtfs = GtfsFilterByTrips(gtfs, trip_ids_set);
      std::cout << "After filtering trips: " << formatGtfsSizes(gtfs)
                << std::endl;
    }

    // Filter to the target date.
    GtfsDay gtfs_day = GtfsFilterByDate(gtfs, date);
    std::cout << "Target date (" << date << "): " << formatGtfsSizes(gtfs_day)
              << std::endl;

    // Take all >=24:00 stop times from the day before and add them to
    // `gtfs_day` with time -24:00.
    constexpr int kSecondsPerDay = 24 * 3600;
    const std::string prev_date = OffsetDate(date, -1);
    GtfsDay prev_gtfs_day = GtfsFilterByDate(gtfs, prev_date);
    prev_gtfs_day.AppendToTripIds(":prev-sd");
    std::erase_if(prev_gtfs_day.stop_times, [](const GtfsStopTime& st) {
      return st.arrival_time.seconds < kSecondsPerDay;
    });
    RemoveUnreferencedTripsRoutesAndDirections(prev_gtfs_day);
    for (auto& stop_time : prev_gtfs_day.stop_times) {
      stop_time.arrival_time.seconds -= kSecondsPerDay;
      stop_time.departure_time.seconds -= kSecondsPerDay;
    }
    std::cout << "Previous date (" << prev_date
              << "): " << formatGtfsSizes(prev_gtfs_day) << std::endl;

    // Take all <24:00 stop times from the next day and add them to `gtfs_day`
    // with time +24:00.
    const std::string next_date = OffsetDate(date, 1);
    GtfsDay next_gtfs_day = GtfsFilterByDate(gtfs, next_date);
    next_gtfs_day.AppendToTripIds(":next-sd");
    std::erase_if(next_gtfs_day.stop_times, [](const GtfsStopTime& st) {
      return st.arrival_time.seconds >= kSecondsPerDay;
    });
    RemoveUnreferencedTripsRoutesAndDirections(next_gtfs_day);
    for (auto& stop_time : next_gtfs_day.stop_times) {
      stop_time.arrival_time.seconds += kSecondsPerDay;
      stop_time.departure_time.seconds += kSecondsPerDay;
    }
    std::cout << "Next date (" << next_date
              << "): " << formatGtfsSizes(next_gtfs_day) << std::endl;

    GtfsDay result = GtfsDayCombine({prev_gtfs_day, gtfs_day, next_gtfs_day});
    std::cout << "Combined result: " << formatGtfsSizes(result) << std::endl;

    // Save to output directory
    std::cout << "Saving filtered data to: " << output_dir << "..."
              << std::endl;
    GtfsSave(result, output_dir);

    std::cout << "Successfully saved filtered GTFS data!" << std::endl;
    std::cout << "Output files created in: " << output_dir << std::endl;

    // List created files
    std::cout << "\nCreated files:" << std::endl;
    for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
      if (entry.is_regular_file()) {
        auto file_size = std::filesystem::file_size(entry.path());
        std::cout << "  " << entry.path().filename().string() << " ("
                  << file_size << " bytes)" << std::endl;
      }
    }

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}