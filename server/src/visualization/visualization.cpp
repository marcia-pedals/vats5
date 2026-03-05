#include "visualization/visualization.h"

#include <cstdio>
#include <string>
#include <unordered_map>
#include <variant>

#include "gtfs/gtfs.h"
#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/tarel_graph.h"  // InitializeProblemStateResult
#include "visualization/sqlite_wrapper.h"
#include "visualization/viz_schema_sql.h"

namespace vats5::viz {

std::string VizSqlitePath(const std::string& input_path) {
  size_t dot_pos = input_path.rfind('.');
  if (dot_pos != std::string::npos) {
    return input_path.substr(0, dot_pos) + "-viz.sqlite";
  }
  return input_path + "-viz.sqlite";
}

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

  // Build a map from GtfsRouteId to GtfsRoute for color lookup.
  std::unordered_map<GtfsRouteId, const GtfsRoute*> gtfs_route_by_id;
  for (const auto& route : gtfs_day.routes) {
    gtfs_route_by_id[route.route_id] = &route;
  }

  // Build a map from GtfsTripId to GtfsTrip for route lookup.
  std::unordered_map<GtfsTripId, const GtfsTrip*> gtfs_trip_by_id;
  for (const auto& trip : gtfs_day.trips) {
    gtfs_trip_by_id[trip.trip_id] = &trip;
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
    if (state.required.Contains(stop_id)) {
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

  // Build a map from required GTFS stop IDs to their representative's GTFS
  // stop ID, so we can write group_representative into the stops table.
  std::unordered_map<GtfsStopId, GtfsStopId> gtfs_representative;
  for (const auto& [stop_id, rep_id] : state.required.representative) {
    const GtfsStopId& stop_gtfs = state.stop_infos.at(stop_id).gtfs_stop_id;
    const GtfsStopId& rep_gtfs = state.stop_infos.at(rep_id).gtfs_stop_id;
    gtfs_representative[stop_gtfs] = rep_gtfs;
  }

  // Insert stops from minimal_paths_sparse with stop_type classification.
  SqliteStmt stop_stmt(
      db,
      "INSERT OR IGNORE INTO stops (stop_id, stop_name, lat, lon, "
      "stop_type, group_representative) VALUES (?, ?, ?, ?, ?, ?)"
  );

  double max_lat = -90.0;
  double sum_lon = 0.0;
  int stop_count = 0;

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

    if (gtfs_stop.stop_lat > max_lat) max_lat = gtfs_stop.stop_lat;
    sum_lon += gtfs_stop.stop_lon;
    stop_count++;

    stop_stmt.bind_text(1, gtfs_id.v.c_str());
    stop_stmt.bind_text(2, gtfs_stop.stop_name.c_str());
    stop_stmt.bind_double(3, gtfs_stop.stop_lat);
    stop_stmt.bind_double(4, gtfs_stop.stop_lon);
    stop_stmt.bind_text(5, stop_type);
    auto rep_it = gtfs_representative.find(gtfs_id);
    if (rep_it != gtfs_representative.end()) {
      stop_stmt.bind_text(6, rep_it->second.v.c_str());
    } else {
      stop_stmt.bind_null(6);
    }
    stop_stmt.step_and_reset();
  }

  // Insert synthetic START and END boundary stops above all real stops.
  {
    double boundary_lat = max_lat + 0.02;
    double avg_lon = stop_count > 0 ? sum_lon / stop_count : 0.0;

    stop_stmt.bind_text(1, "__START__");
    stop_stmt.bind_text(2, "START");
    stop_stmt.bind_double(3, boundary_lat);
    stop_stmt.bind_double(4, avg_lon - 0.01);
    stop_stmt.bind_text(5, "required");
    stop_stmt.bind_null(6);
    stop_stmt.step_and_reset();

    stop_stmt.bind_text(1, "__END__");
    stop_stmt.bind_text(2, "END");
    stop_stmt.bind_double(3, boundary_lat);
    stop_stmt.bind_double(4, avg_lon + 0.01);
    stop_stmt.bind_text(5, "required");
    stop_stmt.bind_null(6);
    stop_stmt.step_and_reset();
  }

  // Insert routes keyed by GtfsRouteDirectionId so that opposite directions
  // (e.g. "Green Line North" vs "Green Line South") get separate entries.
  SqliteStmt route_stmt(
      db,
      "INSERT OR IGNORE INTO routes (route_direction_id, route_name, "
      "route_color, route_text_color) VALUES (?, ?, ?, ?)"
  );
  std::unordered_set<GtfsRouteDirectionId> inserted_routes;

  // Serialize a GtfsRouteDirectionId to a string for storage in SQLite.
  auto route_direction_id_str =
      [](const GtfsRouteDirectionId& rd) -> std::string {
    return rd.route_id.v + ":" + std::to_string(rd.direction_id);
  };

  // Ensure a GTFS route+direction is in the routes table. Returns the
  // serialized route_direction_id string, or empty for flex/walk trips.
  auto ensure_route = [&](TripId trip_id) -> std::string {
    auto info_it = mapping.trip_id_to_trip_info.find(trip_id);
    if (info_it == mapping.trip_id_to_trip_info.end()) return "";
    if (!std::holds_alternative<GtfsTripId>(info_it->second.v)) return "";

    const GtfsTripId& gtfs_trip_id = std::get<GtfsTripId>(info_it->second.v);
    auto trip_it = gtfs_trip_by_id.find(gtfs_trip_id);
    if (trip_it == gtfs_trip_by_id.end()) return "";

    const GtfsRouteDirectionId& route_dir = trip_it->second->route_direction_id;
    if (inserted_routes.count(route_dir) == 0) {
      inserted_routes.insert(route_dir);
      const std::string& route_name = mapping.trip_id_to_route_desc.at(trip_id);
      std::string color;
      std::string text_color;
      auto route_it2 = gtfs_route_by_id.find(route_dir.route_id);
      if (route_it2 != gtfs_route_by_id.end()) {
        color = route_it2->second->route_color;
        text_color = route_it2->second->route_text_color;
      }
      std::string rd_str = route_direction_id_str(route_dir);
      route_stmt.bind_text(1, rd_str.c_str());
      route_stmt.bind_text(2, route_name.c_str());
      route_stmt.bind_text(3, color.c_str());
      route_stmt.bind_text(4, text_color.c_str());
      route_stmt.step_and_reset();
    }
    return route_direction_id_str(route_dir);
  };

  // Insert trips table.
  SqliteStmt trip_stmt(
      db,
      "INSERT OR IGNORE INTO trips (trip_id, gtfs_trip_id, "
      "route_direction_id) VALUES (?, ?, ?)"
  );
  for (const auto& [trip_id, trip_info] : mapping.trip_id_to_trip_info) {
    if (!std::holds_alternative<GtfsTripId>(trip_info.v)) continue;
    const GtfsTripId& gtfs_trip_id = std::get<GtfsTripId>(trip_info.v);
    auto trip_it = gtfs_trip_by_id.find(gtfs_trip_id);
    if (trip_it == gtfs_trip_by_id.end()) continue;
    std::string rd_str =
        route_direction_id_str(trip_it->second->route_direction_id);
    // Ensure the route exists first.
    ensure_route(trip_id);
    trip_stmt.bind_int(1, trip_id.v);
    trip_stmt.bind_text(2, gtfs_trip_id.v.c_str());
    trip_stmt.bind_text(3, rd_str.c_str());
    trip_stmt.step_and_reset();
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
      "is_flex, route_direction_id) VALUES (?, ?, ?, ?, ?, ?, ?)"
  );

  // Map StopId to viz stop_id string, using hardcoded IDs for boundary stops.
  auto viz_stop_id = [&](StopId id) -> std::string {
    if (id == state.boundary.start) return "__START__";
    if (id == state.boundary.end) return "__END__";
    return state.stop_infos.at(id).gtfs_stop_id.v;
  };

  auto is_boundary = [&](StopId id) -> bool {
    return id == state.boundary.start || id == state.boundary.end;
  };

  int next_path_id = 1;

  for (const auto& [origin_stop, path_groups] : state.completed.adjacent) {
    for (const auto& path_group : path_groups) {
      if (path_group.empty()) continue;

      StopId dest_stop = path_group[0].merged_step.destination.stop;

      for (const Path& p : path_group) {
        int path_id = next_path_id++;

        std::string origin_gtfs = viz_stop_id(origin_stop);
        std::string dest_gtfs = viz_stop_id(dest_stop);

        path_stmt.bind_int(1, path_id);
        path_stmt.bind_text(2, origin_gtfs.c_str());
        path_stmt.bind_text(3, dest_gtfs.c_str());
        path_stmt.bind_int(4, p.merged_step.origin.time.seconds);
        path_stmt.bind_int(5, p.merged_step.destination.time.seconds);
        path_stmt.bind_int(6, p.merged_step.is_flex ? 1 : 0);
        path_stmt.step_and_reset();

        // Boundary paths are trivial zero-duration edges that can't be
        // expanded through minimal_paths_sparse. Write steps directly.
        if (is_boundary(origin_stop) || is_boundary(dest_stop)) {
          for (const Step& s : p.steps) {
            std::string s_orig = viz_stop_id(s.origin.stop);
            std::string s_dest = viz_stop_id(s.destination.stop);
            step_stmt.bind_int(1, path_id);
            step_stmt.bind_text(2, s_orig.c_str());
            step_stmt.bind_text(3, s_dest.c_str());
            step_stmt.bind_int(4, s.origin.time.seconds);
            step_stmt.bind_int(5, s.destination.time.seconds);
            step_stmt.bind_int(6, s.is_flex ? 1 : 0);
            step_stmt.bind_null(7);
            step_stmt.step_and_reset();
          }
          continue;
        }

        // Expand each step of the completed path using
        // minimal_paths_sparse to get the original GTFS-level detail.
        // Collect all expanded steps first so we can group by trip.
        std::vector<Step> all_expanded_steps;
        for (const Step& s : p.steps) {
          // Map compacted StopIds to original StopIds.
          const GtfsStopId& s_origin_gtfs =
              state.stop_infos.at(s.origin.stop).gtfs_stop_id;
          const GtfsStopId& s_dest_gtfs =
              state.stop_infos.at(s.destination.stop).gtfs_stop_id;
          StopId orig_origin =
              mapping.gtfs_stop_id_to_stop_id.at(s_origin_gtfs);
          StopId orig_dest = mapping.gtfs_stop_id_to_stop_id.at(s_dest_gtfs);

          // Find matching path in minimal_paths_sparse.
          auto sparse_paths =
              result.minimal_paths_sparse.PathsBetween(orig_origin, orig_dest);
          const Path* expanded = nullptr;
          for (const Path& sp : sparse_paths) {
            if (s.is_flex) {
              if (sp.merged_step.is_flex) {
                expanded = &sp;
                break;
              }
            } else {
              if (!sp.merged_step.is_flex &&
                  sp.merged_step.origin.time == s.origin.time &&
                  sp.merged_step.destination.time == s.destination.time) {
                expanded = &sp;
                break;
              }
            }
          }

          if (!expanded) {
            throw std::runtime_error(
                "No matching path in minimal_paths_sparse for step " +
                s_origin_gtfs.v + " -> " + s_dest_gtfs.v +
                " (is_flex=" + (s.is_flex ? "true" : "false") +
                ", origin_time=" + std::to_string(s.origin.time.seconds) +
                ", dest_time=" + std::to_string(s.destination.time.seconds) +
                ")"
            );
          }

          for (const Step& es : expanded->steps) {
            all_expanded_steps.push_back(es);
          }
        }

        // Group consecutive steps by trip and merge each group into a
        // single step using ConsecutiveMergedSteps.
        struct MergedStepInfo {
          Step step;
          std::string route_direction_id;  // empty for flex/walk
        };
        std::vector<MergedStepInfo> merged_steps;

        size_t si = 0;
        while (si < all_expanded_steps.size()) {
          size_t group_end = si + 1;
          while (group_end < all_expanded_steps.size() &&
                 all_expanded_steps[group_end].destination.trip ==
                     all_expanded_steps[si].destination.trip) {
            group_end++;
          }

          std::vector<Step> group_steps(
              all_expanded_steps.begin() + si,
              all_expanded_steps.begin() + group_end
          );
          Step merged = ConsecutiveMergedSteps(group_steps);

          TripId step_trip_id = all_expanded_steps[si].destination.trip;
          std::string rid = ensure_route(step_trip_id);

          merged_steps.push_back({merged, std::move(rid)});
          si = group_end;
        }

        // Normalize flex step times if necessary.
        {
          std::vector<Step> steps;
          steps.reserve(merged_steps.size());
          for (const auto& ms : merged_steps) {
            steps.push_back(ms.step);
          }
          NormalizeConsecutiveSteps(steps);
          for (size_t i = 0; i < merged_steps.size(); ++i) {
            merged_steps[i].step = steps[i];
          }
        }

        for (const auto& ms : merged_steps) {
          const std::string& step_origin_gtfs =
              mapping.stop_id_to_gtfs_stop_id.at(ms.step.origin.stop).v;
          const std::string& step_dest_gtfs =
              mapping.stop_id_to_gtfs_stop_id.at(ms.step.destination.stop).v;

          step_stmt.bind_int(1, path_id);
          step_stmt.bind_text(2, step_origin_gtfs.c_str());
          step_stmt.bind_text(3, step_dest_gtfs.c_str());
          step_stmt.bind_int(4, ms.step.origin.time.seconds);
          step_stmt.bind_int(5, ms.step.destination.time.seconds);
          step_stmt.bind_int(6, ms.step.is_flex ? 1 : 0);
          if (ms.route_direction_id.empty()) {
            step_stmt.bind_null(7);
          } else {
            step_stmt.bind_text(7, ms.route_direction_id.c_str());
          }
          step_stmt.step_and_reset();
        }
      }
    }
  }

  db.exec("COMMIT");
}

void WriteProblemStateVisualizationSqlite(
    const ProblemState& state,
    const std::string& original_viz_path,
    const std::string& output_path
) {
  std::remove(output_path.c_str());
  std::remove((output_path + "-wal").c_str());
  std::remove((output_path + "-shm").c_str());

  auto is_boundary = [&](StopId id) -> bool {
    return id == state.boundary.start || id == state.boundary.end;
  };

  // Collect all GTFS stop IDs referenced by paths in the problem state.
  // Boundary stops are inserted synthetically, so skip them here.
  std::unordered_set<std::string> referenced_gtfs_ids;
  for (const auto& [origin_stop, path_groups] : state.completed.adjacent) {
    for (const auto& path_group : path_groups) {
      for (const Path& p : path_group) {
        StopId dest = p.merged_step.destination.stop;
        if (!is_boundary(origin_stop)) {
          referenced_gtfs_ids.insert(
              state.stop_infos.at(origin_stop).gtfs_stop_id.v
          );
        }
        if (!is_boundary(dest)) {
          referenced_gtfs_ids.insert(state.stop_infos.at(dest).gtfs_stop_id.v);
        }
        for (const Step& s : p.steps) {
          if (!is_boundary(s.origin.stop)) {
            referenced_gtfs_ids.insert(
                state.stop_infos.at(s.origin.stop).gtfs_stop_id.v
            );
          }
          if (!is_boundary(s.destination.stop)) {
            referenced_gtfs_ids.insert(
                state.stop_infos.at(s.destination.stop).gtfs_stop_id.v
            );
          }
        }
      }
    }
  }

  // Determine required GTFS IDs and group representatives.
  std::unordered_set<std::string> required_gtfs;
  std::unordered_map<std::string, std::string> gtfs_representative;
  for (const auto& [stop_id, rep_id] : state.required.representative) {
    const std::string& gtfs = state.stop_infos.at(stop_id).gtfs_stop_id.v;
    const std::string& rep_gtfs = state.stop_infos.at(rep_id).gtfs_stop_id.v;
    required_gtfs.insert(gtfs);
    gtfs_representative[gtfs] = rep_gtfs;
  }

  SqliteDb db(output_path);
  db.exec("PRAGMA journal_mode=WAL");
  db.exec("BEGIN TRANSACTION");
  db.exec(kSchemaSql);

  // Build trip_id -> route_direction_id mapping from the original viz SQLite.
  std::unordered_map<int, std::string> trip_to_route;

  // Copy referenced stops from original viz SQLite.
  {
    SqliteDb orig_db(original_viz_path);
    sqlite3_stmt* read_stmt = nullptr;
    sqlite3_prepare_v2(
        orig_db.handle(),
        "SELECT stop_id, stop_name, lat, lon FROM stops",
        -1,
        &read_stmt,
        nullptr
    );
    SqliteStmt stop_stmt(
        db,
        "INSERT OR IGNORE INTO stops (stop_id, stop_name, lat, lon, "
        "stop_type, group_representative) VALUES (?, ?, ?, ?, ?, ?)"
    );
    double max_lat = -90.0;
    double sum_lon = 0.0;
    int stop_count = 0;
    while (sqlite3_step(read_stmt) == SQLITE_ROW) {
      const char* sid =
          reinterpret_cast<const char*>(sqlite3_column_text(read_stmt, 0));
      if (!referenced_gtfs_ids.contains(sid)) continue;

      const char* sname =
          reinterpret_cast<const char*>(sqlite3_column_text(read_stmt, 1));
      double lat = sqlite3_column_double(read_stmt, 2);
      double lon = sqlite3_column_double(read_stmt, 3);

      if (lat > max_lat) max_lat = lat;
      sum_lon += lon;
      stop_count++;

      const char* stop_type =
          required_gtfs.contains(sid) ? "required" : "in_problem_state";

      stop_stmt.bind_text(1, sid);
      stop_stmt.bind_text(2, sname);
      stop_stmt.bind_double(3, lat);
      stop_stmt.bind_double(4, lon);
      stop_stmt.bind_text(5, stop_type);
      auto rep_it = gtfs_representative.find(sid);
      if (rep_it != gtfs_representative.end()) {
        stop_stmt.bind_text(6, rep_it->second.c_str());
      } else {
        stop_stmt.bind_null(6);
      }
      stop_stmt.step_and_reset();
    }
    sqlite3_finalize(read_stmt);

    // Insert synthetic START and END boundary stops above all real stops.
    // Use hardcoded IDs so this works even with old serialized data where
    // both boundary stops had GtfsStopId{""}.
    double boundary_lat = max_lat + 0.02;
    double avg_lon = stop_count > 0 ? sum_lon / stop_count : 0.0;

    stop_stmt.bind_text(1, "__START__");
    stop_stmt.bind_text(2, "START");
    stop_stmt.bind_double(3, boundary_lat);
    stop_stmt.bind_double(4, avg_lon - 0.01);
    stop_stmt.bind_text(5, "required");
    stop_stmt.bind_null(6);
    stop_stmt.step_and_reset();

    stop_stmt.bind_text(1, "__END__");
    stop_stmt.bind_text(2, "END");
    stop_stmt.bind_double(3, boundary_lat);
    stop_stmt.bind_double(4, avg_lon + 0.01);
    stop_stmt.bind_text(5, "required");
    stop_stmt.bind_null(6);
    stop_stmt.step_and_reset();
  }

  // Copy all routes from original viz SQLite.
  {
    SqliteDb orig_db(original_viz_path);
    sqlite3_stmt* read_stmt = nullptr;
    sqlite3_prepare_v2(
        orig_db.handle(),
        "SELECT route_direction_id, route_name, route_color, "
        "route_text_color FROM routes",
        -1,
        &read_stmt,
        nullptr
    );
    SqliteStmt route_stmt(
        db,
        "INSERT OR IGNORE INTO routes (route_direction_id, route_name, "
        "route_color, route_text_color) VALUES (?, ?, ?, ?)"
    );
    while (sqlite3_step(read_stmt) == SQLITE_ROW) {
      route_stmt.bind_text(
          1, reinterpret_cast<const char*>(sqlite3_column_text(read_stmt, 0))
      );
      route_stmt.bind_text(
          2, reinterpret_cast<const char*>(sqlite3_column_text(read_stmt, 1))
      );
      route_stmt.bind_text(
          3, reinterpret_cast<const char*>(sqlite3_column_text(read_stmt, 2))
      );
      route_stmt.bind_text(
          4, reinterpret_cast<const char*>(sqlite3_column_text(read_stmt, 3))
      );
      route_stmt.step_and_reset();
    }
    sqlite3_finalize(read_stmt);
  }

  // Copy all trips from original viz SQLite and build trip_to_route mapping.
  {
    SqliteDb orig_db(original_viz_path);
    sqlite3_stmt* read_stmt = nullptr;
    sqlite3_prepare_v2(
        orig_db.handle(),
        "SELECT trip_id, gtfs_trip_id, route_direction_id FROM trips",
        -1,
        &read_stmt,
        nullptr
    );
    SqliteStmt trip_stmt(
        db,
        "INSERT OR IGNORE INTO trips (trip_id, gtfs_trip_id, "
        "route_direction_id) VALUES (?, ?, ?)"
    );
    while (sqlite3_step(read_stmt) == SQLITE_ROW) {
      int tid = sqlite3_column_int(read_stmt, 0);
      const char* gtfs_tid =
          reinterpret_cast<const char*>(sqlite3_column_text(read_stmt, 1));
      const char* rd_id =
          reinterpret_cast<const char*>(sqlite3_column_text(read_stmt, 2));
      trip_to_route[tid] = rd_id;
      trip_stmt.bind_int(1, tid);
      trip_stmt.bind_text(2, gtfs_tid);
      trip_stmt.bind_text(3, rd_id);
      trip_stmt.step_and_reset();
    }
    sqlite3_finalize(read_stmt);
  }

  // Map StopId to viz stop_id string, using hardcoded IDs for boundary
  // stops so this works with old serialized data.
  auto viz_stop_id = [&](StopId id) -> std::string {
    if (id == state.boundary.start) return "__START__";
    if (id == state.boundary.end) return "__END__";
    return state.stop_infos.at(id).gtfs_stop_id.v;
  };

  // Write paths and path_steps from the problem state.
  SqliteStmt path_stmt(
      db,
      "INSERT INTO paths (path_id, origin_stop_id, destination_stop_id, "
      "depart_time, arrive_time, is_flex) VALUES (?, ?, ?, ?, ?, ?)"
  );
  SqliteStmt step_stmt(
      db,
      "INSERT INTO paths_steps (path_id, origin_stop_id, "
      "destination_stop_id, depart_time, arrive_time, "
      "is_flex, route_direction_id) VALUES (?, ?, ?, ?, ?, ?, ?)"
  );

  int next_path_id = 1;
  for (const auto& [origin_stop, path_groups] : state.completed.adjacent) {
    for (const auto& path_group : path_groups) {
      if (path_group.empty()) continue;
      StopId dest_stop = path_group[0].merged_step.destination.stop;

      for (const Path& p : path_group) {
        int path_id = next_path_id++;

        std::string origin_gtfs = viz_stop_id(origin_stop);
        std::string dest_gtfs = viz_stop_id(dest_stop);

        path_stmt.bind_int(1, path_id);
        path_stmt.bind_text(2, origin_gtfs.c_str());
        path_stmt.bind_text(3, dest_gtfs.c_str());
        path_stmt.bind_int(4, p.merged_step.origin.time.seconds);
        path_stmt.bind_int(5, p.merged_step.destination.time.seconds);
        path_stmt.bind_int(6, p.merged_step.is_flex ? 1 : 0);
        path_stmt.step_and_reset();

        // Group consecutive steps by trip and merge.
        std::vector<Step> merged_steps;
        size_t si = 0;
        while (si < p.steps.size()) {
          size_t group_end = si + 1;
          while (group_end < p.steps.size() &&
                 p.steps[group_end].destination.trip ==
                     p.steps[si].destination.trip) {
            group_end++;
          }
          std::vector<Step> group_steps(
              p.steps.begin() + si, p.steps.begin() + group_end
          );
          merged_steps.push_back(ConsecutiveMergedSteps(group_steps));
          si = group_end;
        }

        NormalizeConsecutiveSteps(merged_steps);

        for (const Step& s : merged_steps) {
          std::string s_origin_gtfs = viz_stop_id(s.origin.stop);
          std::string s_dest_gtfs = viz_stop_id(s.destination.stop);

          step_stmt.bind_int(1, path_id);
          step_stmt.bind_text(2, s_origin_gtfs.c_str());
          step_stmt.bind_text(3, s_dest_gtfs.c_str());
          step_stmt.bind_int(4, s.origin.time.seconds);
          step_stmt.bind_int(5, s.destination.time.seconds);
          step_stmt.bind_int(6, s.is_flex ? 1 : 0);

          auto trip_route_it = trip_to_route.find(s.destination.trip.v);
          if (trip_route_it != trip_to_route.end()) {
            step_stmt.bind_text(7, trip_route_it->second.c_str());
          } else {
            step_stmt.bind_null(7);
          }
          step_stmt.step_and_reset();
        }
      }
    }
  }

  // Compute tarel lower bound and write tour edges.
  auto tarel_result = ComputeTarelLowerBound(state);
  if (tarel_result.has_value()) {
    SqliteStmt tarel_stmt(
        db,
        "INSERT INTO tarel_tour_edges (origin_stop_id, origin_partition_id, "
        "destination_stop_id, destination_partition_id, weight) "
        "VALUES (?, ?, ?, ?, ?)"
    );
    for (const TarelEdge& edge : tarel_result->tour_edges) {
      std::string origin_gtfs = viz_stop_id(edge.origin.stop);
      std::string dest_gtfs = viz_stop_id(edge.destination.stop);

      tarel_stmt.bind_text(1, origin_gtfs.c_str());
      tarel_stmt.bind_int(2, edge.origin.partition.v);
      tarel_stmt.bind_text(3, dest_gtfs.c_str());
      tarel_stmt.bind_int(4, edge.destination.partition.v);
      tarel_stmt.bind_int(5, edge.weight);
      tarel_stmt.step_and_reset();
    }
  }

  db.exec("COMMIT");
}

}  // namespace vats5::viz
