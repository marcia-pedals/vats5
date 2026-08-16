# Problem Configs

These fully specify the problem instances, except for the actual GTFS data and the service dates. These include pointers to GTFS data and configure how the service dates are derived from the fetch date.

These are the source of truth -- projections of these are synced into the `solution-viewer` DB. `pipeline/sync_configs.py` makes the DB match: it inserts new configs, overwrites changed ones, and deletes configs that are gone from here along with everything derived from them (their stored solutions, and their GTFS instances when a whole source goes away).

Because stored solutions are keyed off of these configuration ids, edits to these configurations have backwards-compatibility implications. Editing a config in place leaves the solutions already stored under its id describing the old problem; giving the new problem a new id instead retires the old one cleanly.

Every problem spec carries a `description`: prose saying what the problem actually asks for, which the solution viewer shows next to each solution. The `title` is the short label the viewer's grid columns are headed with, so what does not fit there belongs here. A description is copied into every solution solved under the spec, so editing one only affects solutions computed after the edit.

Every config names the GTFS source it belongs to. The sources themselves, and how each one's feed is fetched, live in `pipeline/gtfs_sources.py`; `pipeline/run.py` solves one source per run.

There are some older configs in `server/configs` and derived data in `server/data`. It might be good to migrate to this new place and delete those eventually.
