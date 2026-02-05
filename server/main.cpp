#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

#include "crow.h"
#include "gtfs/gtfs.h"

int main() {
  crow::SimpleApp app;

  vats5::Gtfs gtfs;
  try {
    gtfs = vats5::GtfsLoad("../data/raw_RG_202506");
    std::cout << "Loaded " << gtfs.stops.size() << " stops from GTFS data"
              << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error loading GTFS data: " << e.what() << std::endl;
    return 1;
  }

  CROW_ROUTE(app, "/")([]() { return "Hello world oh yeah!"; });

  CROW_ROUTE(app, "/stops")([&gtfs]() {
    nlohmann::json j = gtfs.stops;
    crow::response res(200, j.dump());
    res.add_header("Content-Type", "application/json");
    return res;
  });

  CROW_ROUTE(app, "/visualization")([]() {
    std::ifstream file("../data/visualization.json");
    if (!file.is_open()) {
      return crow::response(500, "Failed to open visualization.json");
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    crow::response res(200, buffer.str());
    res.add_header("Content-Type", "application/json");
    return res;
  });

  // List all visualizationN.json files
  CROW_ROUTE(app, "/visualizations")([]() {
    std::vector<std::string> files;
    std::regex pattern("visualization(\\d+)\\.json");

    for (const auto& entry : std::filesystem::directory_iterator("../data")) {
      if (entry.is_regular_file()) {
        std::string filename = entry.path().filename().string();
        if (std::regex_match(filename, pattern)) {
          files.push_back(filename);
        }
      }
    }

    // Sort by the numeric part
    std::sort(
        files.begin(), files.end(), [&pattern](const auto& a, const auto& b) {
          std::smatch ma, mb;
          std::regex_match(a, ma, pattern);
          std::regex_match(b, mb, pattern);
          return std::stoi(ma[1].str()) < std::stoi(mb[1].str());
        }
    );

    nlohmann::json j = files;
    crow::response res(200, j.dump());
    res.add_header("Content-Type", "application/json");
    return res;
  });

  // Serve a specific visualization file
  CROW_ROUTE(app, "/visualizations/<string>")([](const std::string& filename) {
    // Validate filename to prevent path traversal
    std::regex pattern("visualization\\d+\\.json");
    if (!std::regex_match(filename, pattern)) {
      return crow::response(400, "Invalid filename");
    }

    std::ifstream file("../data/" + filename);
    if (!file.is_open()) {
      return crow::response(404, "File not found");
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    crow::response res(200, buffer.str());
    res.add_header("Content-Type", "application/json");
    return res;
  });

  app.port(18080).run();
}
