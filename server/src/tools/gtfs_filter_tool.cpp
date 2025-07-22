#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "gtfs/gtfs.h"

using namespace vats5;

void printUsage(const char* program_name) {
  std::cout
      << "Usage: " << program_name
      << " [OPTIONS] <input_dir> <date> <trip_filter> <output_dir>\n"
      << "\n"
      << "GTFS Filter Tool - Filters GTFS data by date and trip IDs or "
         "prefixes\n"
      << "\n"
      << "Arguments:\n"
      << "  input_dir    Directory containing GTFS files (stops.txt, "
         "trips.txt, etc.)\n"
      << "  date         Date in YYYYMMDD format (e.g., 20250708)\n"
      << "  trip_filter  Trip filter specification (see options below)\n"
      << "  output_dir   Directory where filtered GTFS files will be saved\n"
      << "\n"
      << "Options:\n"
      << "  --prefix     Filter by trip ID prefix (include all trips starting "
         "with prefix)\n"
      << "  --trips      Filter by specific trip IDs (default mode)\n"
      << "  --help, -h   Show this help message\n"
      << "\n"
      << "Trip Filter Modes:\n"
      << "  Default (--trips): Comma-separated list of specific trip IDs\n"
      << "                     Example: \"CT:507,SR:198\"\n"
      << "\n"
      << "  Prefix (--prefix): Comma-separated list of prefixes to match trip "
         "IDs\n"
      << "                     Example: \"CT:,SR:\" (includes all Caltrain and "
         "Santa Rosa trips)\n"
      << "\n"
      << "Examples:\n"
      << "  # Filter specific trips\n"
      << "  " << program_name
      << " ../data/RG 20250708 \"CT:507,SR:198\" ./filtered_gtfs\n"
      << "\n"
      << "  # Filter all Caltrain trips (CT: prefix)\n"
      << "  " << program_name
      << " --prefix ../data/RG 20250708 \"CT:\" ./filtered_gtfs\n"
      << "\n"
      << "  # Filter all Santa Rosa trips (SR: prefix)\n"
      << "  " << program_name
      << " --prefix ../data/RG 20250708 \"SR:\" ./filtered_gtfs\n"
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
  // Parse command line arguments
  bool use_prefix = false;
  std::string input_dir, date, trip_filter, output_dir;

  // Handle help first
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      printUsage(argv[0]);
      return 0;
    }
  }

  // Parse options and arguments
  std::vector<std::string> non_option_args;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--prefix") {
      use_prefix = true;
    } else if (arg == "--trips") {
      use_prefix = false;
    } else if (arg.substr(0, 2) == "--") {
      std::cerr << "Error: Unknown option '" << arg << "'" << std::endl;
      std::cerr << "Use --help for usage information." << std::endl;
      return 1;
    } else {
      non_option_args.push_back(arg);
    }
  }

  // Check that we have exactly 4 non-option arguments
  if (non_option_args.size() != 4) {
    std::cerr << "Error: Expected 4 arguments, got " << non_option_args.size()
              << std::endl;
    std::cerr << "Use --help for usage information." << std::endl;
    return 1;
  }

  input_dir = non_option_args[0];
  date = non_option_args[1];
  trip_filter = non_option_args[2];
  output_dir = non_option_args[3];

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

  if (trip_filter.empty()) {
    std::cerr << "Error: Trip filter cannot be empty." << std::endl;
    return 1;
  }

  std::cout << "Loading GTFS data from: " << input_dir << std::endl;
  std::cout << "Filtering for date: " << date << std::endl;
  if (use_prefix) {
    std::vector<std::string> prefix_list = split(trip_filter, ',');
    std::cout << "Using prefix filter(s): ";
    for (size_t i = 0; i < prefix_list.size(); ++i) {
      std::cout << "\"" << prefix_list[i] << "\"";
      if (i < prefix_list.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
  } else {
    std::vector<std::string> trip_id_list = split(trip_filter, ',');
    std::cout << "Including " << trip_id_list.size() << " specific trip(s): ";
    for (size_t i = 0; i < trip_id_list.size(); ++i) {
      std::cout << trip_id_list[i];
      if (i < trip_id_list.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << "Output directory: " << output_dir << std::endl;
  std::cout << std::endl;

  try {
    // Load GTFS data
    std::cout << "Loading GTFS data..." << std::endl;
    Gtfs gtfs = GtfsLoad(input_dir);

    std::cout << "Loaded: " << gtfs.stops.size() << " stops, "
              << gtfs.trips.size() << " trips, " << gtfs.stop_times.size()
              << " stop times, " << gtfs.routes.size() << " routes, "
              << gtfs.directions.size() << " directions" << std::endl;

    // Filter by date
    std::cout << "Filtering by date: " << date << "..." << std::endl;
    GtfsDay gtfs_day = GtfsFilterByDate(gtfs, date);

    std::cout << "After date filter: " << gtfs_day.stops.size() << " stops, "
              << gtfs_day.trips.size() << " trips, "
              << gtfs_day.stop_times.size() << " stop times, "
              << gtfs_day.routes.size() << " routes, "
              << gtfs_day.directions.size() << " directions" << std::endl;

    if (gtfs_day.trips.empty()) {
      std::cerr << "Warning: No trips found for date " << date
                << ". Check if the date is within the service period."
                << std::endl;
    }

    // Filter by trip IDs or prefix
    std::unordered_set<GtfsTripId> trip_ids_set;

    if (use_prefix) {
      std::vector<std::string> prefix_list = split(trip_filter, ',');
      std::cout << "Finding trips with prefix(es)..." << std::endl;
      int matching_count = 0;
      for (const auto& trip : gtfs_day.trips) {
        for (const auto& prefix : prefix_list) {
          if (trip.trip_id.v.substr(0, prefix.length()) == prefix) {
            trip_ids_set.insert(trip.trip_id);
            matching_count++;
            break;  // Don't count the same trip multiple times
          }
        }
      }
      std::cout << "Found " << matching_count << " trips matching prefix(es)."
                << std::endl;

      if (matching_count == 0) {
        std::cerr << "Warning: No trips found with prefix(es) \"" << trip_filter
                  << "\" for date " << date << "." << std::endl;
      }
    } else {
      std::cout << "Filtering by specific trip IDs..." << std::endl;
      std::vector<std::string> trip_id_list = split(trip_filter, ',');
      if (trip_id_list.empty()) {
        std::cerr << "Error: No valid trip IDs found in '" << trip_filter
                  << "'." << std::endl;
        return 1;
      }

      for (const auto& trip_id_str : trip_id_list) {
        trip_ids_set.insert(GtfsTripId{trip_id_str});
      }
    }

    GtfsDay filtered_gtfs = GtfsFilterByTrips(gtfs_day, trip_ids_set);

    std::cout << "After trip filter: " << filtered_gtfs.stops.size()
              << " stops, " << filtered_gtfs.trips.size() << " trips, "
              << filtered_gtfs.stop_times.size() << " stop times, "
              << filtered_gtfs.routes.size() << " routes, "
              << filtered_gtfs.directions.size() << " directions" << std::endl;

    if (filtered_gtfs.trips.empty()) {
      if (use_prefix) {
        std::cerr << "Warning: No trips found matching prefix(es) \""
                  << trip_filter << "\" for date " << date << "." << std::endl;
      } else {
        std::cerr << "Warning: No trips found matching the specified trip IDs "
                     "for date "
                  << date << "." << std::endl;
        std::cerr
            << "Make sure the trip IDs exist and are active on the given date."
            << std::endl;
      }
    }

    // Save to output directory
    std::cout << "Saving filtered data to: " << output_dir << "..."
              << std::endl;
    GtfsSave(filtered_gtfs, output_dir);

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