# Problem Configs

These fully specify the problem instances, except for the actual GTFS data and the service dates. These include pointers to GTFS data and configure how the service dates are derived from the fetch date.

These are the source of truth -- projections of these are synced into the `solution-viewer` DB. Because stored solutions are keyed off of these configuration ids, edits to these configurations have backwards-compatibility implications.

A `problem_spec` carries `problem_spec_id`, `gtfs_source_id`, `target_stops_id` and `title` as columns of their own; every other key is passed through to `initialize_problem_state`:

- `max_walking_distance` -- how far a walking connection may be, in meters.
- `walking_speed` -- how fast it is walked, in m/s.
- `subsequent_service_days` -- how many days after the service date the solve may run into, defaulting to 1. Each one is another day of trips shifted forward by 24:00, and it also sets how late a tour may set off: 4 hours into the last of those days. Raise it for a network a tour cannot get around in a day.

A `target_stops` set can include stops that are not served on every date. Filtering the GTFS to a date keeps every stop in `stops.txt`, so such a stop still resolves on a date it has no service of its own -- it stays required, and the tour has to reach it on some other operator. Generating a set from a Friday is a good way to catch both the weekday-only and the weekend-only stops in one go, since the window spans both.

There are some older configs in `server/configs` and derived data in `server/data`. It might be good to migrate to this new place and delete those eventually.
