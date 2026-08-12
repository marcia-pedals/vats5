# Problem Configs

These fully specify the problem instances, except for the actual GTFS data and the service dates. These include pointers to GTFS data and configure how the service dates are derived from the fetch date.

These are the source of truth -- projections of these are synced into the `solution-viewer` DB. Because stored solutions are keyed off of these configuration ids, edits to these configurations have backwards-compatibility implications.

A `problem_spec` carries `problem_spec_id`, `gtfs_source_id`, `target_stops_id` and `title` as columns of their own; every other key is passed through to `initialize_problem_state`:

- `max_walking_distance` -- how far a walking connection may be, in meters.
- `walking_speed` -- how fast it is walked, in m/s.
- `subsequent_service_days` -- how many days after the service date the solve may run into, defaulting to 1. Each one is another day of trips shifted forward by 24:00, and it also sets how late a tour may set off: 4 hours into the last of those days. Raise it for a network a tour cannot get around in a day.

Every stop in a `target_stops` set has to be served on every date being solved. A stop with no service on some date makes that date fail, so seasonal and weekday-only stops are excluded when the set is generated.

There are some older configs in `server/configs` and derived data in `server/data`. It might be good to migrate to this new place and delete those eventually.
