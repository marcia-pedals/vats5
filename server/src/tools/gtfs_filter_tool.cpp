#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "gtfs/gtfs.h"
#include "gtfs/gtfs_filter.h"

using namespace vats5;

template <typename T>
std::string formatGtfsSizes(const T& gtfs) {
  std::ostringstream oss;
  oss << gtfs.stops.size() << " stops, " << gtfs.trips.size() << " trips, "
      << gtfs.stop_times.size() << " stop times, " << gtfs.routes.size()
      << " routes, " << gtfs.directions.size() << " directions";
  return oss.str();
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
  CLI::App app{"Filter GTFS data by date and optionally by trip ID prefixes"};

  std::string input_dir;
  std::string date;
  std::string output_dir;
  std::string trip_filter;

  app.add_option(
         "input_dir",
         input_dir,
         "Directory containing GTFS files (stops.txt, trips.txt, etc.)"
  )
      ->required()
      ->check(CLI::ExistingDirectory);
  app.add_option("date", date, "Date in YYYYMMDD format (e.g., 20250708)")
      ->required();
  app.add_option(
         "output_dir",
         output_dir,
         "Directory where filtered GTFS files will be saved"
  )
      ->required();
  app.add_option(
      "--prefix",
      trip_filter,
      "Filter by trip ID prefix (comma-separated list, e.g., "
      "\"CT:,SR:\")"
  );

  CLI11_PARSE(app, argc, argv);

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
      std::vector<std::string> prefix_list = split(trip_filter, ',');
      gtfs = GtfsFilterByPrefixes(gtfs, prefix_list);
      std::cout << "After filtering trips: " << formatGtfsSizes(gtfs)
                << std::endl;
    }

    // Filter by date and combine with adjacent service days.
    GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, date);
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
