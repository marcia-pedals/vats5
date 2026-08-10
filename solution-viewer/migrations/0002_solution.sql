CREATE DOMAIN slug AS VARCHAR(32) CHECK (VALUE ~ '^[a-z0-9]+([_-][a-z0-9]+)*$');

-- For ids built by concatenating other ids, which do not fit in 32 characters.
CREATE DOMAIN long_slug AS VARCHAR(64) CHECK (VALUE ~ '^[a-z0-9]+([_-][a-z0-9]+)*$');

CREATE TABLE gtfs_source (
  gtfs_source_id slug PRIMARY KEY,
  title VARCHAR(128) NOT NULL UNIQUE
);

CREATE TABLE gtfs_instance (
  gtfs_instance_id slug PRIMARY KEY,
  gtfs_source_id slug NOT NULL REFERENCES gtfs_source (gtfs_source_id),
  fetched_at TIMESTAMPTZ NOT NULL,
  UNIQUE (gtfs_source_id, fetched_at),
  UNIQUE (gtfs_instance_id, gtfs_source_id)
);

CREATE TABLE target_stops (
  target_stops_id slug PRIMARY KEY,
  gtfs_source_id slug NOT NULL REFERENCES gtfs_source (gtfs_source_id),
  title VARCHAR(128) NOT NULL,
  data JSONB NOT NULL,
  UNIQUE (gtfs_source_id, title),
  UNIQUE (target_stops_id, gtfs_source_id)
);

CREATE TABLE problem_spec (
  problem_spec_id slug PRIMARY KEY,
  gtfs_source_id slug NOT NULL REFERENCES gtfs_source (gtfs_source_id),
  target_stops_id slug NOT NULL,
  title VARCHAR(128) NOT NULL,
  data JSONB NOT NULL,
  UNIQUE (gtfs_source_id, title),
  UNIQUE (problem_spec_id, gtfs_source_id),
  FOREIGN KEY (target_stops_id, gtfs_source_id)
    REFERENCES target_stops (target_stops_id, gtfs_source_id)
);

CREATE TABLE problem_instance (
  problem_instance_id long_slug PRIMARY KEY,
  gtfs_source_id slug NOT NULL REFERENCES gtfs_source (gtfs_source_id),
  gtfs_instance_id slug NOT NULL,
  problem_spec_id slug NOT NULL,
  service_date VARCHAR(8) NOT NULL CHECK (service_date ~ '^\d{8}$'),
  data JSONB NOT NULL,
  created_at TIMESTAMPTZ NOT NULL DEFAULT now(),
  FOREIGN KEY (gtfs_instance_id, gtfs_source_id)
    REFERENCES gtfs_instance (gtfs_instance_id, gtfs_source_id),
  FOREIGN KEY (problem_spec_id, gtfs_source_id)
    REFERENCES problem_spec (problem_spec_id, gtfs_source_id)
);

CREATE INDEX problem_spec_target_stops_idx ON problem_spec (target_stops_id, gtfs_source_id);
CREATE INDEX problem_spec_gtfs_source_idx ON problem_spec (gtfs_source_id);
CREATE INDEX problem_instance_gtfs_instance_idx
  ON problem_instance (gtfs_instance_id, gtfs_source_id);
CREATE INDEX problem_instance_problem_spec_idx
  ON problem_instance (problem_spec_id, gtfs_source_id);
CREATE INDEX problem_instance_gtfs_source_idx ON problem_instance (gtfs_source_id);
