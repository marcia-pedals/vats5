#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <toml++/toml.hpp>
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

bool isValidDate(const std::string& date) {
  if (date.length() != 8) return false;

  for (char c : date) {
    if (c < '0' || c > '9') return false;
  }

  int year = std::stoi(date.substr(0, 4));
  int month = std::stoi(date.substr(4, 2));
  int day = std::stoi(date.substr(6, 2));

  return year >= 1900 && year <= 2100 && month >= 1 && month <= 12 &&
         day >= 1 && day <= 31;
}

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: gtfs_filter_tool <config.toml>" << std::endl;
    return 1;
  }

  std::string config_path = argv[1];

  toml::table config;
  try {
    config = toml::parse_file(config_path);
  } catch (const toml::parse_error& e) {
    std::cerr << "Error parsing config file: " << e << std::endl;
    return 1;
  }

  auto input_dir = config["input_dir"].value<std::string>();
  auto date = config["date"].value<std::string>();
  auto output_dir = config["output_dir"].value<std::string>();

  if (!input_dir || !date || !output_dir) {
    std::cerr << "Config file must contain input_dir, date, and output_dir"
              << std::endl;
    return 1;
  }

  if (!isValidDate(*date)) {
    std::cerr << "Error: Invalid date format '" << *date
              << "'. Expected YYYYMMDD format." << std::endl;
    return 1;
  }

  if (!std::filesystem::is_directory(*input_dir)) {
    std::cerr << "Error: Input directory '" << *input_dir
              << "' does not exist." << std::endl;
    return 1;
  }

  std::vector<std::string> prefixes;
  if (auto arr = config["prefixes"].as_array()) {
    for (const auto& elem : *arr) {
      if (auto val = elem.value<std::string>()) {
        prefixes.push_back(*val);
      }
    }
  }

  std::cout << "Loading GTFS data from: " << *input_dir << std::endl;
  std::cout << "Filtering for date: " << *date << std::endl;
  if (prefixes.empty()) {
    std::cout << "Including all trips (no prefix filter)" << std::endl;
  } else {
    std::cout << "Using prefix filter(s): ";
    for (size_t i = 0; i < prefixes.size(); ++i) {
      std::cout << "\"" << prefixes[i] << "\"";
      if (i < prefixes.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << "Output directory: " << *output_dir << std::endl;
  std::cout << std::endl;

  try {
    std::cout << "Loading GTFS data..." << std::endl;
    Gtfs gtfs = GtfsLoad(*input_dir);

    std::cout << "Loaded: " << formatGtfsSizes(gtfs) << std::endl;

    if (!prefixes.empty()) {
      gtfs = GtfsFilterByPrefixes(gtfs, prefixes);
      std::cout << "After filtering trips: " << formatGtfsSizes(gtfs)
                << std::endl;
    }

    GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, *date);
    std::cout << "Combined result: " << formatGtfsSizes(result) << std::endl;

    std::cout << "Saving filtered data to: " << *output_dir << "..."
              << std::endl;
    GtfsSave(result, *output_dir);

    std::cout << "Successfully saved filtered GTFS data!" << std::endl;
    std::cout << "Output files created in: " << *output_dir << std::endl;

    std::cout << "\nCreated files:" << std::endl;
    for (const auto& entry :
         std::filesystem::directory_iterator(*output_dir)) {
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
