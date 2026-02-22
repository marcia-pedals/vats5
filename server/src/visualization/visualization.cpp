#include "visualization/visualization.h"

#include <sqlite3.h>

#include <stdexcept>
#include <string>
#include <unordered_map>

#include "gtfs/gtfs.h"
#include "solver/tarel_graph.h"
#include "visualization/viz_schema_sql.h"

namespace vats5::viz {

// RAII wrapper for sqlite3 database handle.
class SqliteDb {
 public:
  explicit SqliteDb(const std::string& path) {
    int rc = sqlite3_open(path.c_str(), &db_);
    if (rc != SQLITE_OK) {
      std::string err = sqlite3_errmsg(db_);
      sqlite3_close(db_);
      db_ = nullptr;
      throw std::runtime_error("Failed to open SQLite database: " + err);
    }
  }

  ~SqliteDb() {
    if (db_) sqlite3_close(db_);
  }

  SqliteDb(const SqliteDb&) = delete;
  SqliteDb& operator=(const SqliteDb&) = delete;

  void exec(const char* sql) {
    char* err_msg = nullptr;
    int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &err_msg);
    if (rc != SQLITE_OK) {
      std::string err = err_msg;
      sqlite3_free(err_msg);
      throw std::runtime_error("SQLite exec error: " + err);
    }
  }

  sqlite3_stmt* prepare(const char* sql) {
    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
      throw std::runtime_error(
          std::string("Failed to prepare statement: ") + sqlite3_errmsg(db_)
      );
    }
    return stmt;
  }

  sqlite3* handle() { return db_; }

 private:
  sqlite3* db_ = nullptr;
};

void WriteVisualizationSqlite(
    const ProblemState& state, const GtfsDay& gtfs_day, const std::string& path
) {
  // Build a map from GtfsStopId to GtfsStop for direct lookup
  std::unordered_map<GtfsStopId, GtfsStop> gtfs_stop_by_id;
  for (const auto& stop : gtfs_day.stops) {
    gtfs_stop_by_id[stop.stop_id] = stop;
  }

  // Helper to get the GTFS stop_id string for a StopId
  auto gtfs_id_for = [&](StopId sid) -> std::string {
    auto it = state.stop_infos.find(sid);
    if (it == state.stop_infos.end()) return "";
    return it->second.gtfs_stop_id.v;
  };

  SqliteDb db(path);
  db.exec("PRAGMA journal_mode=WAL");
  db.exec("BEGIN TRANSACTION");
  db.exec(kSchemaSql);

  // Insert stops
  sqlite3_stmt* stop_stmt = db.prepare(
      "INSERT INTO stops (stop_id, stop_name, lat, lon) VALUES (?, ?, ?, ?)"
  );

  for (const StopId& stop_id : state.required_stops) {
    if (stop_id == state.boundary.start || stop_id == state.boundary.end) {
      continue;
    }

    auto stop_info_it = state.stop_infos.find(stop_id);
    if (stop_info_it == state.stop_infos.end()) {
      throw std::runtime_error(
          "StopId " + std::to_string(stop_id.v) + " not found in stop_infos"
      );
    }
    const ProblemStateStopInfo& stop_info = stop_info_it->second;

    auto gtfs_stop_it = gtfs_stop_by_id.find(stop_info.gtfs_stop_id);
    if (gtfs_stop_it == gtfs_stop_by_id.end()) {
      throw std::runtime_error(
          "GtfsStopId '" + stop_info.gtfs_stop_id.v + "' not found in GTFS data"
      );
    }
    const GtfsStop& gtfs_stop = gtfs_stop_it->second;

    sqlite3_bind_text(
        stop_stmt, 1, gtfs_stop.stop_id.v.c_str(), -1, SQLITE_TRANSIENT
    );
    sqlite3_bind_text(
        stop_stmt, 2, gtfs_stop.stop_name.c_str(), -1, SQLITE_TRANSIENT
    );
    sqlite3_bind_double(stop_stmt, 3, gtfs_stop.stop_lat);
    sqlite3_bind_double(stop_stmt, 4, gtfs_stop.stop_lon);
    sqlite3_step(stop_stmt);
    sqlite3_reset(stop_stmt);
  }
  sqlite3_finalize(stop_stmt);

  // Insert paths
  sqlite3_stmt* path_stmt = db.prepare(
      "INSERT INTO paths (origin_stop_id, destination_stop_id, depart_time, "
      "arrive_time, duration_seconds, is_flex) VALUES (?, ?, ?, ?, ?, ?)"
  );

  for (const auto& [origin_stop, path_groups] : state.completed.adjacent) {
    if (origin_stop == state.boundary.start ||
        origin_stop == state.boundary.end) {
      continue;
    }

    std::string origin_gtfs_id = gtfs_id_for(origin_stop);
    if (origin_gtfs_id.empty()) continue;

    for (const auto& path_group : path_groups) {
      if (path_group.empty()) continue;

      StopId dest_stop = path_group[0].merged_step.destination.stop;
      if (dest_stop == state.boundary.start ||
          dest_stop == state.boundary.end) {
        continue;
      }

      std::string dest_gtfs_id = gtfs_id_for(dest_stop);
      if (dest_gtfs_id.empty()) continue;

      for (const Path& p : path_group) {
        sqlite3_bind_text(
            path_stmt, 1, origin_gtfs_id.c_str(), -1, SQLITE_TRANSIENT
        );
        sqlite3_bind_text(
            path_stmt, 2, dest_gtfs_id.c_str(), -1, SQLITE_TRANSIENT
        );
        sqlite3_bind_int(path_stmt, 3, p.merged_step.origin.time.seconds);
        sqlite3_bind_int(path_stmt, 4, p.merged_step.destination.time.seconds);
        sqlite3_bind_int(path_stmt, 5, p.DurationSeconds());
        sqlite3_bind_int(path_stmt, 6, p.merged_step.is_flex ? 1 : 0);
        sqlite3_step(path_stmt);
        sqlite3_reset(path_stmt);
      }
    }
  }
  sqlite3_finalize(path_stmt);

  db.exec("COMMIT");
}

}  // namespace vats5::viz
