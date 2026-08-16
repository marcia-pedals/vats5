#!/usr/bin/env python3
"""Sync problem_configs/ into the solution-viewer database.

The TOML files under problem_configs/ are the source of truth for target_stops
and problem_spec; gtfs_sources.py is for gtfs_source. This writes a projection
of both into those three tables.

Rows that do not exist yet are inserted. A row whose primary key already exists
is left alone if every synced column matches, and aborts the entire sync
otherwise -- ids are referenced by stored solutions, so a changed config under
an existing id is a mistake that has to be resolved by hand.

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

from gtfs_sources import GTFS_SOURCES

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIGS_DIR = REPO_ROOT / "problem_configs"
ENV_FILE = REPO_ROOT / "solution-viewer" / ".env.local"

# Columns that live in their own table column; every other key in the TOML file
# is carried through as the row's JSONB `data`.
TARGET_STOPS_COLUMNS = ("target_stops_id", "gtfs_source_id", "title")
PROBLEM_SPEC_COLUMNS = ("problem_spec_id", "gtfs_source_id", "target_stops_id", "title")


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

    if config["gtfs_source_id"] not in GTFS_SOURCES:
        raise SyncError(
            f"{path}: gtfs_source_id is {config['gtfs_source_id']!r}, which is "
            f"not one of the sources in gtfs_sources.py "
            f"({', '.join(sorted(GTFS_SOURCES))})"
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


def sync_row(
    cursor: psycopg.Cursor, table: str, pk_column: str, row: dict[str, Any]
) -> bool:
    """Insert `row`, or verify it matches the existing one. True if inserted."""
    pk_value = row[pk_column]

    cursor.execute(
        sql.SQL("SELECT * FROM {} WHERE {} = %s").format(
            sql.Identifier(table), sql.Identifier(pk_column)
        ),
        (pk_value,),
    )
    existing = cursor.fetchone()

    if existing is None:
        cursor.execute(
            sql.SQL("INSERT INTO {} ({}) VALUES ({})").format(
                sql.Identifier(table),
                sql.SQL(", ").join(sql.Identifier(column) for column in row),
                sql.SQL(", ").join(sql.Placeholder() for _ in row),
            ),
            [
                psycopg.types.json.Jsonb(value) if column == "data" else value
                for column, value in row.items()
            ],
        )
        return True

    differences = {
        column: (existing[column], value)
        for column, value in row.items()
        if existing[column] != value
    }
    if differences:
        detail = "\n".join(
            f"    {column}: database has {old!r}, config has {new!r}"
            for column, (old, new) in sorted(differences.items())
        )
        raise SyncError(
            f"{table} {pk_column}={pk_value!r} already exists with different data:\n"
            f"{detail}\n"
            "  Stored solutions are keyed off these ids -- resolve this by hand, "
            "either by reverting the config or by giving it a new id."
        )
    return False


def sync_table(
    cursor: psycopg.Cursor, table: str, pk_column: str, rows: list[dict[str, Any]]
) -> None:
    inserted = sum(sync_row(cursor, table, pk_column, row) for row in rows)
    print(f"{table}: {inserted} inserted, {len(rows) - inserted} unchanged")


def main() -> None:
    gtfs_sources = [
        {"gtfs_source_id": source.gtfs_source_id, "title": source.title}
        for source in GTFS_SOURCES.values()
    ]
    target_stops = load_dir("target_stops", TARGET_STOPS_COLUMNS)
    problem_specs = load_dir("problem_spec", PROBLEM_SPEC_COLUMNS)

    # One transaction for the whole sync: any mismatch rolls back everything.
    with psycopg.connect(database_url(), row_factory=dict_row) as connection:
        with connection.cursor() as cursor:
            sync_table(cursor, "gtfs_source", "gtfs_source_id", gtfs_sources)
            sync_table(cursor, "target_stops", "target_stops_id", target_stops)
            sync_table(cursor, "problem_spec", "problem_spec_id", problem_specs)


if __name__ == "__main__":
    try:
        main()
    except SyncError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
