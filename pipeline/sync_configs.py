#!/usr/bin/env python3
"""Sync problem_configs/ into the solution-viewer database.

The TOML files under problem_configs/ are the source of truth; this writes a
projection of them into gtfs_source, target_stops and problem_spec.

The database is made to match the configs exactly. Rows that do not exist yet
are inserted, rows whose synced columns differ are overwritten, and rows whose
config is gone are deleted along with everything that depends on them -- a
deleted problem_spec takes its problem_instances (the stored solutions) with it,
and a deleted gtfs_source takes its gtfs_instances too.

Usage:
    python3 pipeline/sync_configs.py

DATABASE_URL is taken from the environment, falling back to
solution-viewer/.env.local (written by bin/pg init).
"""

from __future__ import annotations

import os
import sys
import tomllib
from pathlib import Path
from typing import Any

import psycopg
from psycopg import sql
from psycopg.rows import dict_row

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIGS_DIR = REPO_ROOT / "problem_configs"
ENV_FILE = REPO_ROOT / "solution-viewer" / ".env.local"

GTFS_SOURCE_ID = "bayarea"
GTFS_SOURCE_TITLE = "Bay Area 511.org Regional"

# Columns that live in their own table column; every other key in the TOML file
# is carried through as the row's JSONB `data`.
TARGET_STOPS_COLUMNS = ("target_stops_id", "gtfs_source_id", "title")
PROBLEM_SPEC_COLUMNS = ("problem_spec_id", "gtfs_source_id", "target_stops_id", "title")

# Rows whose config is gone, as (table, primary key, condition on the ids the
# configs still define). Each condition matches the rows that are stale
# themselves plus the rows whose parent is stale.

# The tables the configs project into, leaves first.
STALE_CONFIGS = (
    (
        "problem_spec",
        "problem_spec_id",
        "gtfs_source_id <> ALL(%(sources)s) OR problem_spec_id <> ALL(%(specs)s)",
    ),
    (
        "target_stops",
        "target_stops_id",
        "gtfs_source_id <> ALL(%(sources)s) OR target_stops_id <> ALL(%(stops)s)",
    ),
    ("gtfs_source", "gtfs_source_id", "gtfs_source_id <> ALL(%(sources)s)"),
)

# What the pipeline derived from those configs: the stored solutions and the
# GTFS instances they were solved against. Deleted first so that dropping a
# config never leaves a dangling foreign key.
STALE_DERIVED = (
    (
        "problem_instance",
        "problem_instance_id",
        "gtfs_source_id <> ALL(%(sources)s) OR problem_spec_id <> ALL(%(specs)s)",
    ),
    ("gtfs_instance", "gtfs_instance_id", "gtfs_source_id <> ALL(%(sources)s)"),
)

# Titles are unique per source, and a retiring config usually hands its title to
# the config replacing it. The replacement is inserted before the retiring row
# can be deleted -- it may still be referenced until the specs are repointed --
# so the retiring row's title is parked under this prefix in between.
RETIRED_TITLE_PREFIX = "retired:"


class SyncError(Exception):
    """Raised when a config cannot be synced; aborts the whole run."""


def database_url() -> str:
    url = os.environ.get("DATABASE_URL")
    if url:
        return url
    if not ENV_FILE.exists():
        raise SyncError(f"DATABASE_URL is not set and {ENV_FILE} does not exist")
    for line in ENV_FILE.read_text().splitlines():
        line = line.strip()
        if line.startswith("DATABASE_URL="):
            return line.split("=", 1)[1]
    raise SyncError(f"DATABASE_URL is not set and {ENV_FILE} does not define it")


def load_config(path: Path, columns: tuple[str, ...]) -> dict[str, Any]:
    """Read a config TOML into a row: named columns plus everything else as data."""
    with path.open("rb") as handle:
        config = tomllib.load(handle)

    missing = [column for column in columns if column not in config]
    if missing:
        raise SyncError(f"{path}: missing required key(s): {', '.join(missing)}")

    if config["gtfs_source_id"] != GTFS_SOURCE_ID:
        raise SyncError(
            f"{path}: gtfs_source_id is {config['gtfs_source_id']!r}, "
            f"but only {GTFS_SOURCE_ID!r} is supported"
        )

    row = {column: config[column] for column in columns}
    row["data"] = {key: value for key, value in config.items() if key not in columns}
    return row


def load_dir(subdir: str, columns: tuple[str, ...]) -> list[dict[str, Any]]:
    directory = CONFIGS_DIR / subdir
    if not directory.is_dir():
        raise SyncError(f"{directory} does not exist")
    paths = sorted(directory.glob("*.toml"))
    if not paths:
        raise SyncError(f"{directory} contains no .toml files")
    return [load_config(path, columns) for path in paths]


def abbreviate(value: Any) -> str:
    """A repr short enough to sit on one line; `data` blobs are long."""
    text = repr(value)
    return text if len(text) <= 80 else f"{text[:77]}..."


def sync_row(
    cursor: psycopg.Cursor, table: str, pk_column: str, row: dict[str, Any]
) -> tuple[str, list[str]]:
    """Insert `row`, or overwrite the existing one.

    Returns "inserted", "updated" or "unchanged", and a line per overwritten
    column for the caller to report.
    """
    pk_value = row[pk_column]

    cursor.execute(
        sql.SQL("SELECT * FROM {} WHERE {} = %s").format(
            sql.Identifier(table), sql.Identifier(pk_column)
        ),
        (pk_value,),
    )
    existing = cursor.fetchone()

    def bind(column: str, value: Any) -> Any:
        return psycopg.types.json.Jsonb(value) if column == "data" else value

    if existing is None:
        cursor.execute(
            sql.SQL("INSERT INTO {} ({}) VALUES ({})").format(
                sql.Identifier(table),
                sql.SQL(", ").join(sql.Identifier(column) for column in row),
                sql.SQL(", ").join(sql.Placeholder() for _ in row),
            ),
            [bind(column, value) for column, value in row.items()],
        )
        return "inserted", []

    changed = {
        column: value for column, value in row.items() if existing[column] != value
    }
    if not changed:
        return "unchanged", []

    report = [
        f"  overwrote {pk_column}={pk_value} {column}: "
        f"{abbreviate(existing[column])} -> {abbreviate(value)}"
        for column, value in sorted(changed.items())
    ]
    cursor.execute(
        sql.SQL("UPDATE {} SET {} WHERE {} = %s").format(
            sql.Identifier(table),
            sql.SQL(", ").join(
                sql.SQL("{} = {}").format(sql.Identifier(column), sql.Placeholder())
                for column in changed
            ),
            sql.Identifier(pk_column),
        ),
        [bind(column, value) for column, value in changed.items()] + [pk_value],
    )
    return "updated", report


def sync_table(
    cursor: psycopg.Cursor, table: str, pk_column: str, rows: list[dict[str, Any]]
) -> list[Any]:
    """Sync every row of one table. Returns the primary keys that were updated."""
    results = {row[pk_column]: sync_row(cursor, table, pk_column, row) for row in rows}
    counts = {
        outcome: sum(1 for result in results.values() if result[0] == outcome)
        for outcome in ("inserted", "updated", "unchanged")
    }
    print(f"{table}: " + ", ".join(f"{count} {name}" for name, count in counts.items()))
    for _, report in results.values():
        for line in report:
            print(line)
    return [pk for pk, (outcome, _) in results.items() if outcome == "updated"]


def delete_stale(cursor: psycopg.Cursor, live: dict[str, list[str]]) -> None:
    """Delete every row whose config is gone, plus everything that depends on it."""
    for table, pk_column, condition in STALE_DERIVED + STALE_CONFIGS:
        cursor.execute(
            sql.SQL("DELETE FROM {} WHERE {} RETURNING {}").format(
                sql.Identifier(table), sql.SQL(condition), sql.Identifier(pk_column)
            ),
            live,
        )
        deleted = sorted(row[pk_column] for row in cursor.fetchall())
        if deleted:
            print(f"{table}: {len(deleted)} deleted")
            for pk_value in deleted:
                print(f"  deleted {pk_column}={pk_value}")


def warn_about_stale_instances(cursor: psycopg.Cursor, spec_ids: list[str]) -> None:
    """Point out solutions that were computed under a config we just overwrote."""
    if not spec_ids:
        return
    cursor.execute(
        "SELECT count(*) AS count FROM problem_instance "
        "WHERE problem_spec_id = ANY(%s)",
        (spec_ids,),
    )
    count = cursor.fetchone()["count"]
    if count:
        print(
            f"warning: {count} problem_instance row(s) for "
            f"{', '.join(sorted(spec_ids))} were solved under the previous "
            "config and were left in place"
        )


def main() -> None:
    gtfs_source = {"gtfs_source_id": GTFS_SOURCE_ID, "title": GTFS_SOURCE_TITLE}
    target_stops = load_dir("target_stops", TARGET_STOPS_COLUMNS)
    problem_specs = load_dir("problem_spec", PROBLEM_SPEC_COLUMNS)

    live = {
        "sources": [gtfs_source["gtfs_source_id"]],
        "stops": [row["target_stops_id"] for row in target_stops],
        "specs": [row["problem_spec_id"] for row in problem_specs],
    }

    # A spec pointing at target stops that no longer have a config would be a
    # foreign key violation halfway through the sync; say so up front instead.
    dangling = sorted(
        {
            row["target_stops_id"]
            for row in problem_specs
            if row["target_stops_id"] not in set(live["stops"])
        }
    )
    if dangling:
        raise SyncError(
            "problem_spec configs reference target stops with no config: "
            f"{', '.join(dangling)}"
        )

    # One transaction for the whole sync: any failure rolls back everything.
    with psycopg.connect(database_url(), row_factory=dict_row) as connection:
        with connection.cursor() as cursor:
            sync_table(cursor, "gtfs_source", "gtfs_source_id", [gtfs_source])
            updated_stops = sync_table(
                cursor, "target_stops", "target_stops_id", target_stops
            )
            updated_specs = sync_table(
                cursor, "problem_spec", "problem_spec_id", problem_specs
            )
            # Deleting afterwards lets a spec be pointed at different target
            # stops in the same sync that drops the old ones.
            delete_stale(cursor, live)

            # Changing the stops changes the problem just as much as changing
            # the spec does, so every spec using them is affected.
            affected = set(updated_specs) | {
                row["problem_spec_id"]
                for row in problem_specs
                if row["target_stops_id"] in set(updated_stops)
            }
            warn_about_stale_instances(cursor, sorted(affected))


if __name__ == "__main__":
    try:
        main()
    except SyncError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
