#include "visualization/visualization.h"

#include <sqlite3.h>

#include <stdexcept>
#include <unordered_map>

#include "gtfs/gtfs.h"
#include "solver/tarel_graph.h"

namespace vats5::viz {

Visualization MakeVisualization(
    const ProblemState& state, const GtfsDay& gtfs_day
) {
  // Build a map from GtfsStopId to GtfsStop for direct lookup
  std::unordered_map<GtfsStopId, GtfsStop> gtfs_stop_by_id;
  for (const auto& stop : gtfs_day.stops) {
    gtfs_stop_by_id[stop.stop_id] = stop;
  }

  Visualization viz;
  for (const StopId& stop_id : state.required_stops) {
    // Skip START and END boundary stops (they are synthetic)
    if (stop_id == state.boundary.start || stop_id == state.boundary.end) {
      continue;
    }

    // Get the stop info from the state
    auto stop_info_it = state.stop_infos.find(stop_id);
    if (stop_info_it == state.stop_infos.end()) {
      throw std::runtime_error(
          "StopId " + std::to_string(stop_id.v) + " not found in stop_infos"
      );
    }
    const ProblemStateStopInfo& stop_info = stop_info_it->second;

    // Get the GtfsStop by ID for coordinates
    auto gtfs_stop_it = gtfs_stop_by_id.find(stop_info.gtfs_stop_id);
    if (gtfs_stop_it == gtfs_stop_by_id.end()) {
      throw std::runtime_error(
          "GtfsStopId '" + stop_info.gtfs_stop_id.v + "' not found in GTFS data"
      );
    }
    const GtfsStop& gtfs_stop = gtfs_stop_it->second;

    // Create viz::Stop
    viz.stops.push_back(
        Stop{
            .stop_id = gtfs_stop.stop_id.v,
            .stop_name = gtfs_stop.stop_name,
            .lat = gtfs_stop.stop_lat,
            .lon = gtfs_stop.stop_lon,
        }
    );
  }

  // Helper to get the GTFS stop_id string for a StopId
  auto gtfs_id_for = [&](StopId sid) -> std::string {
    auto it = state.stop_infos.find(sid);
    if (it == state.stop_infos.end()) return "";
    return it->second.gtfs_stop_id.v;
  };

  // Serialize paths from completed adjacency list
  for (const auto& [origin_stop, path_groups] : state.completed.adjacent) {
    // Skip boundary stops
    if (origin_stop == state.boundary.start ||
        origin_stop == state.boundary.end) {
      continue;
    }

    std::string origin_gtfs_id = gtfs_id_for(origin_stop);
    if (origin_gtfs_id.empty()) continue;

    for (const auto& path_group : path_groups) {
      if (path_group.empty()) continue;

      StopId dest_stop = path_group[0].merged_step.destination.stop;
      // Skip boundary stops
      if (dest_stop == state.boundary.start ||
          dest_stop == state.boundary.end) {
        continue;
      }

      std::string dest_gtfs_id = gtfs_id_for(dest_stop);
      if (dest_gtfs_id.empty()) continue;

      VizPathGroup vpg;
      vpg.origin_stop_id = origin_gtfs_id;
      vpg.destination_stop_id = dest_gtfs_id;

      for (const Path& path : path_group) {
        vpg.paths.push_back(
            VizPath{
                .depart_time = path.merged_step.origin.time.seconds,
                .arrive_time = path.merged_step.destination.time.seconds,
                .duration_seconds = path.DurationSeconds(),
                .is_flex = path.merged_step.is_flex,
            }
        );
      }

      viz.path_groups.push_back(std::move(vpg));
    }
  }

  return viz;
}

void WriteVisualizationSqlite(
    const Visualization& viz, const std::string& path
) {
  sqlite3* db = nullptr;
  int rc = sqlite3_open(path.c_str(), &db);
  if (rc != SQLITE_OK) {
    std::string err = sqlite3_errmsg(db);
    sqlite3_close(db);
    throw std::runtime_error("Failed to open SQLite database: " + err);
  }

  auto exec = [&](const char* sql) {
    char* err_msg = nullptr;
    rc = sqlite3_exec(db, sql, nullptr, nullptr, &err_msg);
    if (rc != SQLITE_OK) {
      std::string err = err_msg;
      sqlite3_free(err_msg);
      sqlite3_close(db);
      throw std::runtime_error("SQLite exec error: " + err);
    }
  };

  exec("PRAGMA journal_mode=WAL");
  exec("BEGIN TRANSACTION");

  exec(R"(
    CREATE TABLE stops (
      stop_id TEXT PRIMARY KEY,
      stop_name TEXT NOT NULL,
      lat REAL NOT NULL,
      lon REAL NOT NULL
    )
  )");

  exec(R"(
    CREATE TABLE paths (
      origin_stop_id TEXT NOT NULL,
      destination_stop_id TEXT NOT NULL,
      depart_time INTEGER NOT NULL,
      arrive_time INTEGER NOT NULL,
      duration_seconds INTEGER NOT NULL,
      is_flex INTEGER NOT NULL,
      FOREIGN KEY (origin_stop_id) REFERENCES stops(stop_id),
      FOREIGN KEY (destination_stop_id) REFERENCES stops(stop_id)
    )
  )");

  exec(
      "CREATE INDEX idx_paths_pair ON paths(origin_stop_id, "
      "destination_stop_id)"
  );

  // Insert stops
  sqlite3_stmt* stop_stmt = nullptr;
  rc = sqlite3_prepare_v2(
      db,
      "INSERT INTO stops (stop_id, stop_name, lat, lon) VALUES (?, ?, ?, ?)",
      -1,
      &stop_stmt,
      nullptr
  );
  if (rc != SQLITE_OK) {
    std::string err = sqlite3_errmsg(db);
    sqlite3_close(db);
    throw std::runtime_error("Failed to prepare stop insert: " + err);
  }

  for (const auto& stop : viz.stops) {
    sqlite3_bind_text(stop_stmt, 1, stop.stop_id.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stop_stmt, 2, stop.stop_name.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_double(stop_stmt, 3, stop.lat);
    sqlite3_bind_double(stop_stmt, 4, stop.lon);
    sqlite3_step(stop_stmt);
    sqlite3_reset(stop_stmt);
  }
  sqlite3_finalize(stop_stmt);

  // Insert paths
  sqlite3_stmt* path_stmt = nullptr;
  rc = sqlite3_prepare_v2(
      db,
      "INSERT INTO paths (origin_stop_id, destination_stop_id, depart_time, "
      "arrive_time, duration_seconds, is_flex) VALUES (?, ?, ?, ?, ?, ?)",
      -1,
      &path_stmt,
      nullptr
  );
  if (rc != SQLITE_OK) {
    std::string err = sqlite3_errmsg(db);
    sqlite3_close(db);
    throw std::runtime_error("Failed to prepare path insert: " + err);
  }

  for (const auto& pg : viz.path_groups) {
    for (const auto& p : pg.paths) {
      sqlite3_bind_text(
          path_stmt, 1, pg.origin_stop_id.c_str(), -1, SQLITE_STATIC
      );
      sqlite3_bind_text(
          path_stmt, 2, pg.destination_stop_id.c_str(), -1, SQLITE_STATIC
      );
      sqlite3_bind_int(path_stmt, 3, p.depart_time);
      sqlite3_bind_int(path_stmt, 4, p.arrive_time);
      sqlite3_bind_int(path_stmt, 5, p.duration_seconds);
      sqlite3_bind_int(path_stmt, 6, p.is_flex ? 1 : 0);
      sqlite3_step(path_stmt);
      sqlite3_reset(path_stmt);
    }
  }
  sqlite3_finalize(path_stmt);

  exec("COMMIT");
  sqlite3_close(db);
}

}  // namespace vats5::viz
