CREATE TABLE stops (
  stop_id TEXT PRIMARY KEY,
  stop_name TEXT NOT NULL,
  lat REAL NOT NULL,
  lon REAL NOT NULL,
  stop_type TEXT NOT NULL
);

CREATE TABLE routes (
  route_id TEXT PRIMARY KEY,
  route_name TEXT NOT NULL,
  route_color TEXT NOT NULL DEFAULT '',
  route_text_color TEXT NOT NULL DEFAULT ''
);

CREATE TABLE trips (
  trip_id INTEGER PRIMARY KEY,
  gtfs_trip_id TEXT NOT NULL,
  route_id TEXT NOT NULL,
  FOREIGN KEY (route_id) REFERENCES routes(route_id)
);

CREATE TABLE paths (
  path_id INTEGER NOT NULL PRIMARY KEY,
  origin_stop_id TEXT NOT NULL,
  destination_stop_id TEXT NOT NULL,
  depart_time INTEGER NOT NULL,
  arrive_time INTEGER NOT NULL,
  is_flex INTEGER NOT NULL,
  FOREIGN KEY (origin_stop_id) REFERENCES stops(stop_id),
  FOREIGN KEY (destination_stop_id) REFERENCES stops(stop_id)
);

CREATE INDEX idx_paths_pair ON paths(origin_stop_id, destination_stop_id);

CREATE TABLE paths_steps (
  path_id INTEGER NOT NULL,

  origin_stop_id TEXT NOT NULL,
  destination_stop_id TEXT NOT NULL,
  depart_time INTEGER NOT NULL,
  arrive_time INTEGER NOT NULL,
  is_flex INTEGER NOT NULL,
  -- NULL for flex/walking trips that have no GTFS route.
  route_id TEXT,

  FOREIGN KEY (path_id) REFERENCES paths(path_id),
  FOREIGN KEY (origin_stop_id) REFERENCES stops(stop_id),
  FOREIGN KEY (destination_stop_id) REFERENCES stops(stop_id),
  FOREIGN KEY (route_id) REFERENCES routes(route_id)
);

CREATE TABLE partial_solutions (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  run_timestamp TEXT NOT NULL,
  iteration INTEGER NOT NULL,
  data TEXT NOT NULL
);
