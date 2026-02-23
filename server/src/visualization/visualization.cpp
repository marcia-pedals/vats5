#include "visualization/visualization.h"

#include <sqlite3.h>

#include <cstdio>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "gtfs/gtfs.h"
#include "solver/data.h"
#include "solver/tarel_graph.h"  // InitializeProblemStateResult
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
    const InitializeProblemStateResult& result,
    const GtfsDay& gtfs_day,
    const DataGtfsMapping& mapping,
    const std::string& path
) {
  const ProblemState& state = result.problem_state;
  // Delete existing db file and associated WAL/shm files if present
  std::remove(path.c_str());
  std::remove((path + "-wal").c_str());
  std::remove((path + "-shm").c_str());

  // Build a map from GtfsStopId to GtfsStop for direct lookup
  std::unordered_map<GtfsStopId, GtfsStop> gtfs_stop_by_id;
  for (const auto& stop : gtfs_day.stops) {
    gtfs_stop_by_id[stop.stop_id] = stop;
  }

  SqliteDb db(path);
  db.exec("PRAGMA journal_mode=WAL");
  db.exec("BEGIN TRANSACTION");
  db.exec(kSchemaSql);

  // Build sets of GTFS stop IDs that are in problem_state / required.
  std::unordered_set<GtfsStopId> required_gtfs_ids;
  std::unordered_set<GtfsStopId> in_problem_state_gtfs_ids;
  for (const auto& [stop_id, stop_info] : state.stop_infos) {
    if (stop_id == state.boundary.start || stop_id == state.boundary.end) {
      continue;
    }
    in_problem_state_gtfs_ids.insert(stop_info.gtfs_stop_id);
    if (state.required_stops.count(stop_id) > 0) {
      required_gtfs_ids.insert(stop_info.gtfs_stop_id);
    }
  }

  // Collect all stops referenced by any step in minimal_paths_sparse.
  std::unordered_set<StopId> all_sparse_stops;
  for (const Path& p : result.minimal_paths_sparse.AllPaths()) {
    for (const Step& s : p.steps) {
      all_sparse_stops.insert(s.origin.stop);
      all_sparse_stops.insert(s.destination.stop);
    }
  }

  // Insert stops from minimal_paths_sparse with stop_type classification.
  SqliteStmt stop_stmt(
      db,
      "INSERT OR IGNORE INTO stops (stop_id, stop_name, lat, lon, "
      "stop_type) VALUES (?, ?, ?, ?, ?)"
  );

  for (StopId stop_id : all_sparse_stops) {
    auto gtfs_id_it = mapping.stop_id_to_gtfs_stop_id.find(stop_id);
    if (gtfs_id_it == mapping.stop_id_to_gtfs_stop_id.end()) {
      throw std::runtime_error(
          "StopId " + std::to_string(stop_id.v) +
          " not found in stop_id_to_gtfs_stop_id mapping"
      );
    }
    const GtfsStopId& gtfs_id = gtfs_id_it->second;

    auto gtfs_stop_it = gtfs_stop_by_id.find(gtfs_id);
    if (gtfs_stop_it == gtfs_stop_by_id.end()) {
      throw std::runtime_error(
          "GtfsStopId " + gtfs_id.v + " not found in gtfs_stop_by_id"
      );
    }
    const GtfsStop& gtfs_stop = gtfs_stop_it->second;

    const char* stop_type = "original";
    if (required_gtfs_ids.count(gtfs_id) > 0) {
      stop_type = "required";
    } else if (in_problem_state_gtfs_ids.count(gtfs_id) > 0) {
      stop_type = "in_problem_state";
    }

    stop_stmt.bind_text(1, gtfs_id.v.c_str());
    stop_stmt.bind_text(2, gtfs_stop.stop_name.c_str());
    stop_stmt.bind_double(3, gtfs_stop.stop_lat);
    stop_stmt.bind_double(4, gtfs_stop.stop_lon);
    stop_stmt.bind_text(5, stop_type);
    stop_stmt.step_and_reset();
  }

  // Insert paths and path steps
  SqliteStmt path_stmt(
      db,
      "INSERT INTO paths (path_id, origin_stop_id, destination_stop_id, "
      "depart_time, arrive_time, is_flex) VALUES (?, ?, ?, ?, ?, ?)"
  );
  SqliteStmt step_stmt(
      db,
      "INSERT INTO paths_steps (path_id, origin_stop_id, "
      "destination_stop_id, depart_time, arrive_time, "
      "is_flex, route_name) VALUES (?, ?, ?, ?, ?, ?, ?)"
  );

  int next_path_id = 1;

  for (const auto& [origin_stop, path_groups] : state.completed.adjacent) {
    if (origin_stop == state.boundary.start ||
        origin_stop == state.boundary.end) {
      continue;
    }

    for (const auto& path_group : path_groups) {
      if (path_group.empty()) continue;

      StopId dest_stop = path_group[0].merged_step.destination.stop;
      if (dest_stop == state.boundary.start ||
          dest_stop == state.boundary.end) {
        continue;
      }

      for (const Path& p : path_group) {
        int path_id = next_path_id++;

        const std::string& origin_gtfs =
            state.stop_infos.at(origin_stop).gtfs_stop_id.v;
        const std::string& dest_gtfs =
            state.stop_infos.at(dest_stop).gtfs_stop_id.v;

        path_stmt.bind_int(1, path_id);
        path_stmt.bind_text(2, origin_gtfs.c_str());
        path_stmt.bind_text(3, dest_gtfs.c_str());
        path_stmt.bind_int(4, p.merged_step.origin.time.seconds);
        path_stmt.bind_int(5, p.merged_step.destination.time.seconds);
        path_stmt.bind_int(6, p.merged_step.is_flex ? 1 : 0);
        path_stmt.step_and_reset();

        // Deduplicate: only emit the last step per consecutive
        // destination trip (collapses multi-stop rides into one row).
        for (size_t si = 0; si < p.steps.size(); ++si) {
          const Step& s = p.steps[si];
          // Skip if the next step has the same trip (and it's not NOOP).
          if (si + 1 < p.steps.size() && s.destination.trip != TripId::NOOP &&
              s.destination.trip == p.steps[si + 1].destination.trip) {
            continue;
          }

          std::string route_name;
          if (s.destination.trip != TripId::NOOP) {
            auto it = mapping.trip_id_to_route_desc.find(s.destination.trip);
            if (it != mapping.trip_id_to_route_desc.end()) {
              route_name = it->second;
            }
          }

          const std::string& step_origin_gtfs =
              state.stop_infos.at(s.origin.stop).gtfs_stop_id.v;
          const std::string& step_dest_gtfs =
              state.stop_infos.at(s.destination.stop).gtfs_stop_id.v;

          step_stmt.bind_int(1, path_id);
          step_stmt.bind_text(2, step_origin_gtfs.c_str());
          step_stmt.bind_text(3, step_dest_gtfs.c_str());
          step_stmt.bind_int(4, s.origin.time.seconds);
          step_stmt.bind_int(5, s.destination.time.seconds);
          step_stmt.bind_int(6, s.is_flex ? 1 : 0);
          step_stmt.bind_text(7, route_name.c_str());
          step_stmt.step_and_reset();
        }
      }
    }
  }

  db.exec("COMMIT");
}

}  // namespace vats5::viz
