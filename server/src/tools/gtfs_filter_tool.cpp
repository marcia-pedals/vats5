#include <filesystem>
#include <iostream>
#include <string>

#include "gtfs/gtfs.h"
#include "gtfs/gtfs_filter.h"
#include "log.h"

using namespace vats5;

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: gtfs_filter_tool <config.toml> <output_dir>"
              << std::endl;
    return 1;
  }

  std::string output_dir = argv[2];

  try {
    GtfsFilterConfig config = GtfsFilterConfigLoad(argv[1]);
    GtfsDay result = GtfsFilterFromConfig(config, OstreamLogger(std::cerr));

    std::cout << "Saving filtered data to: " << output_dir << "..."
              << std::endl;
    GtfsSave(result, output_dir);

    std::cout << "Successfully saved filtered GTFS data!" << std::endl;

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
