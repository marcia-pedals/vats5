CREATE TABLE stops (
  stop_id TEXT PRIMARY KEY,
  stop_name TEXT NOT NULL,
  lat REAL NOT NULL,
  lon REAL NOT NULL,
  required INTEGER NOT NULL
);

CREATE TABLE paths (
  origin_stop_id TEXT NOT NULL,
  destination_stop_id TEXT NOT NULL,
  depart_time INTEGER NOT NULL,
  arrive_time INTEGER NOT NULL,
  duration_seconds INTEGER NOT NULL,
  is_flex INTEGER NOT NULL,
  FOREIGN KEY (origin_stop_id) REFERENCES stops(stop_id),
  FOREIGN KEY (destination_stop_id) REFERENCES stops(stop_id)
);

CREATE INDEX idx_paths_pair ON paths(origin_stop_id, destination_stop_id);
