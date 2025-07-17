#include <iostream>

#include "crow.h"
#include "src/gtfs/gtfs.h"

int main() {
  crow::SimpleApp app;

  std::vector<vats5::Stop> stops;
  try {
    stops = vats5::GtfsLoadStops("../data/RG/stops.txt");
    std::cout << "Loaded " << stops.size() << " stops from GTFS data"
              << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error loading GTFS data: " << e.what() << std::endl;
    return 1;
  }

  CROW_ROUTE(app, "/")([]() { return "Hello world oh yeah!"; });

  CROW_ROUTE(app, "/stops")([&stops]() {
    crow::json::wvalue json_stops = crow::json::wvalue::list();

    for (size_t i = 0; i < stops.size(); ++i) {
      const auto& stop = stops[i];
      crow::json::wvalue stop_json;
      stop_json["stop_id"] = stop.stop_id.v;
      stop_json["stop_name"] = stop.stop_name;
      stop_json["stop_lat"] = stop.stop_lat;
      stop_json["stop_lon"] = stop.stop_lon;
      if (stop.parent_station.has_value()) {
        stop_json["parent_station"] = stop.parent_station.value().v;
      } else {
        stop_json["parent_station"] = nullptr;
      }
      json_stops[i] = std::move(stop_json);
    }

    crow::response res(200, json_stops);
    res.add_header("Content-Type", "application/json");
    return res;
  });

  app.port(18080).run();
}
