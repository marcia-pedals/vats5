CREATE TABLE stops (
  stop_id INTEGER PRIMARY KEY,
  gtfs_stop_id TEXT NOT NULL,
  stop_name TEXT NOT NULL,
  lat REAL NOT NULL,
  lon REAL NOT NULL,
  required INTEGER NOT NULL
);

CREATE TABLE paths (
  path_id INTEGER NOT NULL PRIMARY KEY,
  origin_stop_id INTEGER NOT NULL,
  destination_stop_id INTEGER NOT NULL,
  depart_time INTEGER NOT NULL,
  arrive_time INTEGER NOT NULL,
  is_flex INTEGER NOT NULL,
  FOREIGN KEY (origin_stop_id) REFERENCES stops(stop_id),
  FOREIGN KEY (destination_stop_id) REFERENCES stops(stop_id)
);

CREATE INDEX idx_paths_pair ON paths(origin_stop_id, destination_stop_id);

CREATE TABLE paths_steps (
  path_id INTEGER NOT NULL,

  origin_stop_id INTEGER NOT NULL,
  destination_stop_id INTEGER NOT NULL,
  depart_time INTEGER NOT NULL,
  arrive_time INTEGER NOT NULL,
  is_flex INTEGER NOT NULL,

  FOREIGN KEY (path_id) REFERENCES paths(path_id),
  FOREIGN KEY (origin_stop_id) REFERENCES stops(stop_id),
  FOREIGN KEY (destination_stop_id) REFERENCES stops(stop_id)
);
