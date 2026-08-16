# Problem Configs

These fully specify the problem instances, except for the actual GTFS data and the service dates. These include pointers to GTFS data and configure how the service dates are derived from the fetch date.

These are the source of truth -- projections of these are synced into the `solution-viewer` DB. Because stored solutions are keyed off of these configuration ids, edits to these configurations have backwards-compatibility implications.

Every config names the GTFS source it belongs to. The sources themselves, and how each one's feed is fetched, live in `pipeline/gtfs_sources.py`; `pipeline/run.py` solves one source per run.

There are some older configs in `server/configs` and derived data in `server/data`. It might be good to migrate to this new place and delete those eventually.
