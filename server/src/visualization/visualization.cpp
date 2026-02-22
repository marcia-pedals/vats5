#include "visualization/visualization.h"

#include <sqlite3.h>

#include <cstdio>
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

  sqlite3* handle() { return db_; }

 private:
  sqlite3* db_ = nullptr;
};

// RAII wrapper for sqlite3 prepared statements.
class SqliteStmt {
 public:
  SqliteStmt(SqliteDb& db, const char* sql) {
    int rc = sqlite3_prepare_v2(db.handle(), sql, -1, &stmt_, nullptr);
    if (rc != SQLITE_OK) {
      throw std::runtime_error(
          std::string("Failed to prepare statement: ") +
          sqlite3_errmsg(db.handle())
      );
    }
  }

  ~SqliteStmt() {
    if (stmt_) sqlite3_finalize(stmt_);
  }

  SqliteStmt(const SqliteStmt&) = delete;
  SqliteStmt& operator=(const SqliteStmt&) = delete;

  void bind_text(int col, const char* val) {
    sqlite3_bind_text(stmt_, col, val, -1, SQLITE_TRANSIENT);
  }
  void bind_double(int col, double val) {
    sqlite3_bind_double(stmt_, col, val);
  }
  void bind_int(int col, int val) { sqlite3_bind_int(stmt_, col, val); }

  void step_and_reset() {
    sqlite3_step(stmt_);
    sqlite3_reset(stmt_);
  }

 private:
  sqlite3_stmt* stmt_ = nullptr;
};

void WriteVisualizationSqlite(
    const ProblemState& state, const GtfsDay& gtfs_day, const std::string& path
) {
  // Delete existing db file if present
  std::remove(path.c_str());

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
  SqliteStmt stop_stmt(
      db, "INSERT INTO stops (stop_id, stop_name, lat, lon) VALUES (?, ?, ?, ?)"
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

    stop_stmt.bind_text(1, gtfs_stop.stop_id.v.c_str());
    stop_stmt.bind_text(2, gtfs_stop.stop_name.c_str());
    stop_stmt.bind_double(3, gtfs_stop.stop_lat);
    stop_stmt.bind_double(4, gtfs_stop.stop_lon);
    stop_stmt.step_and_reset();
  }

  // Insert paths
  SqliteStmt path_stmt(
      db,
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
        path_stmt.bind_text(1, origin_gtfs_id.c_str());
        path_stmt.bind_text(2, dest_gtfs_id.c_str());
        path_stmt.bind_int(3, p.merged_step.origin.time.seconds);
        path_stmt.bind_int(4, p.merged_step.destination.time.seconds);
        path_stmt.bind_int(5, p.DurationSeconds());
        path_stmt.bind_int(6, p.merged_step.is_flex ? 1 : 0);
        path_stmt.step_and_reset();
      }
    }
  }

  db.exec("COMMIT");
}

}  // namespace vats5::viz
