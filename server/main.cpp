#include <fstream>
#include <iostream>
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

  app.port(18080).run();
}
