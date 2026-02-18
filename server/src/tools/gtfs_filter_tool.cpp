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

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: gtfs_filter_tool <config.toml>" << std::endl;
    return 1;
  }

  try {
    GtfsFilterConfig config = GtfsFilterConfigLoad(argv[1]);

    std::cout << "Loading GTFS data from: " << config.input_dir << std::endl;
    std::cout << "Filtering for date: " << config.date << std::endl;
    if (config.prefixes.empty()) {
      std::cout << "Including all trips (no prefix filter)" << std::endl;
    } else {
      std::cout << "Using prefix filter(s): ";
      for (size_t i = 0; i < config.prefixes.size(); ++i) {
        std::cout << "\"" << config.prefixes[i] << "\"";
        if (i < config.prefixes.size() - 1) std::cout << ", ";
      }
      std::cout << std::endl;
    }
    std::cout << "Output directory: " << config.output_dir << std::endl;
    std::cout << std::endl;

    GtfsDay result = GtfsFilterFromConfig(config);
    std::cout << "Result: " << formatGtfsSizes(result) << std::endl;

    std::cout << "Saving filtered data to: " << config.output_dir << "..."
              << std::endl;
    GtfsSave(result, config.output_dir);

    std::cout << "Successfully saved filtered GTFS data!" << std::endl;
    std::cout << "Output files created in: " << config.output_dir << std::endl;

    std::cout << "\nCreated files:" << std::endl;
    for (const auto& entry :
         std::filesystem::directory_iterator(config.output_dir)) {
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
