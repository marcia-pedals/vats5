#include "gtfs.h"

#include <csv.hpp>
#include <iostream>
#include <stdexcept>

namespace vats5 {

std::vector<Stop> GtfsLoadStops(const std::string &stops_file_path) {
  std::vector<Stop> stops;

  try {
    csv::CSVReader reader(stops_file_path);

    for (csv::CSVRow &row : reader) {
      Stop &stop = stops.emplace_back();
      stop.stop_id = StopId{row["stop_id"].get<std::string>()};
      stop.stop_name = row["stop_name"].get<std::string>();
      stop.stop_lat = row["stop_lat"].get<double>();
      stop.stop_lon = row["stop_lon"].get<double>();
      std::string parent_station_str = row["parent_station"].get<std::string>();
      if (parent_station_str.empty()) {
        stop.parent_station = std::nullopt;
      } else {
        stop.parent_station = StopId{std::move(parent_station_str)};
      }
    }
  } catch (const std::exception &e) {
    throw std::runtime_error(
        "Could not open or parse file: " + stops_file_path + " - " + e.what());
  }

  return stops;
}

}  // namespace vats5