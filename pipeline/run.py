#!/usr/bin/env python3
"""Download the latest Bay Area GTFS, then create and solve problem instances.

Each run gets its own directory under ~/vats5-pipeline:

    run_{timestamp}/
        gtfs/                                  the downloaded RG feed
        service_{date}/
            {problem_spec_id}/
                world.toml                     input for initialize_problem_state
                target_stops.toml              input for initialize_problem_state
                problem_state.json             its output (+ .sqlite for viz)
                initialize_problem_state.log
                iterative_expansion.log
                solution.json                  what gets stored in the db

The run directory name doubles as the gtfs_instance_id. Older run directories
are pruned, which does not affect the rows already written for them.

Problem specs are read from the database, so run pipeline/sync_configs.py first.

Usage:
    python3 pipeline/run.py
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from datetime import date, datetime, timedelta, timezone
from pathlib import Path
from typing import Any

import psycopg
from psycopg.rows import dict_row
from psycopg.types.json import Jsonb

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "download_gtfs"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from gtfs_feed import download_feed  # noqa: E402
from sync_configs import database_url  # noqa: E402

OPERATOR_ID = "RG"
GTFS_SOURCE_ID = "bayarea"

PIPELINE_ROOT = Path.home() / "vats5-pipeline"
BUILD_DIR = REPO_ROOT / "server" / "build-debug"

# Number of run directories to keep on disk, newest first.
MAX_RUN_DIRS = 5

# Service dates to solve: this many days, starting the day after the fetch.
SERVICE_DATE_COUNT = 7

# Passed to iterative_expansion, which checks it between iterations and on each
# search event. The subprocess gets a slightly longer hard limit as a backstop
# for a single Concorde solve that blows through the cooperative check.
SOLVE_TIMEOUT_SECONDS = 300
SOLVE_KILL_GRACE_SECONDS = 60


class RunError(Exception):
    """Raised for conditions that should stop the whole run."""


def tool_path(name: str) -> Path:
    path = BUILD_DIR / name
    if not path.is_file():
        raise RunError(f"{path} not found -- build it with ninja in {BUILD_DIR}")
    return path


def prune_run_dirs(keep: int) -> None:
    """Delete all but the `keep` newest run directories."""
    run_dirs = sorted(
        (path for path in PIPELINE_ROOT.glob("run_*") if path.is_dir()),
        reverse=True,
    )
    for stale in run_dirs[keep:]:
        print(f"pruning old run directory {stale}")
        shutil.rmtree(stale)


def toml_string_array(name: str, values: list[str]) -> str:
    lines = [f"{name} = ["]
    lines += [f'  "{value}",' for value in values]
    lines.append("]")
    return "\n".join(lines)


def write_target_stops_toml(path: Path, data: dict[str, Any]) -> None:
    """Write the stop_ids/stop_groups TOML initialize_problem_state expects."""
    if "stop_ids" not in data:
        raise RunError(f"target_stops data has no stop_ids: {sorted(data)}")

    sections = [toml_string_array("stop_ids", data["stop_ids"])]
    if data.get("stop_groups"):
        group_lines = ["stop_groups = ["]
        for group in data["stop_groups"]:
            members = ", ".join(f'"{stop_id}"' for stop_id in group)
            group_lines.append(f"  [{members}],")
        group_lines.append("]")
        sections.append("\n".join(group_lines))

    path.write_text("\n\n".join(sections) + "\n")


def write_world_toml(path: Path, feed_dir: Path, service_date: str) -> None:
    """Write the GtfsFilterConfig for one service date over the whole feed."""
    path.write_text(f'input_dir = "{feed_dir}"\ndate = "{service_date}"\n')


def run_tool(command: list[str], log_path: Path, timeout: float | None) -> int | None:
    """Run `command`, sending stdout+stderr to `log_path`.

    Returns the exit code, or None if the process had to be killed on timeout.
    """
    with log_path.open("w") as log_file:
        log_file.write(f"$ {' '.join(command)}\n\n")
        log_file.flush()
        try:
            completed = subprocess.run(
                command, stdout=log_file, stderr=subprocess.STDOUT, timeout=timeout
            )
        except subprocess.TimeoutExpired:
            log_file.write(f"\n[killed by run.py after {timeout}s]\n")
            return None
    return completed.returncode


def solve_one(
    spec: dict[str, Any], spec_dir: Path, feed_dir: Path, service_date: str
) -> dict[str, Any]:
    """Initialize and solve one spec for one service date; return solution info."""
    spec_dir.mkdir(parents=True, exist_ok=True)

    world_toml = spec_dir / "world.toml"
    target_stops_toml = spec_dir / "target_stops.toml"
    problem_state = spec_dir / "problem_state.json"
    solution_json = spec_dir / "solution.json"

    write_world_toml(world_toml, feed_dir, service_date)
    write_target_stops_toml(target_stops_toml, spec["target_stops_data"])

    spec_data = spec["data"]
    initialize_command = [
        str(tool_path("initialize_problem_state")),
        str(world_toml),
        str(target_stops_toml),
        str(problem_state),
    ]
    if "max_walking_distance" in spec_data:
        initialize_command += [
            "--max-walking-distance",
            str(spec_data["max_walking_distance"]),
        ]
    if "walking_speed" in spec_data:
        initialize_command += ["--walking-speed", str(spec_data["walking_speed"])]

    returncode = run_tool(
        initialize_command, spec_dir / "initialize_problem_state.log", timeout=None
    )
    if returncode != 0:
        print(f"    initialize_problem_state failed (exit {returncode})")
        return {"status": "initialize_failed", "returncode": returncode}

    returncode = run_tool(
        [
            str(tool_path("iterative_expansion")),
            str(problem_state),
            "--solution_json",
            str(solution_json),
            "--timeout",
            str(SOLVE_TIMEOUT_SECONDS),
        ],
        spec_dir / "iterative_expansion.log",
        timeout=SOLVE_TIMEOUT_SECONDS + SOLVE_KILL_GRACE_SECONDS,
    )

    # iterative_expansion writes solution.json for every outcome it reaches on
    # its own, including a timeout; a missing file means it died first.
    if solution_json.exists():
        solution = json.loads(solution_json.read_text())
        print(f"    {solution['status']}: {solution.get('optimal_duration_seconds')}")
        return solution

    status = "killed" if returncode is None else "solve_failed"
    print(f"    {status} (exit {returncode})")
    return {"status": status, "returncode": returncode}


def load_specs(cursor: psycopg.Cursor) -> list[dict[str, Any]]:
    cursor.execute(
        """
        SELECT spec.problem_spec_id,
               spec.data,
               stops.data AS target_stops_data
        FROM problem_spec AS spec
        JOIN target_stops AS stops
          ON stops.target_stops_id = spec.target_stops_id
         AND stops.gtfs_source_id = spec.gtfs_source_id
        WHERE spec.gtfs_source_id = %s
        ORDER BY spec.problem_spec_id
        """,
        (GTFS_SOURCE_ID,),
    )
    specs = cursor.fetchall()
    if not specs:
        raise RunError(
            f"no problem_spec rows for {GTFS_SOURCE_ID!r} -- "
            "run pipeline/sync_configs.py first"
        )
    return specs


def main() -> None:
    # Resolve both tools up front so a missing build fails before downloading.
    tool_path("initialize_problem_state")
    tool_path("iterative_expansion")

    run_id = f"run_{datetime.now():%Y%m%d-%H%M%S}"
    run_dir = PIPELINE_ROOT / run_id
    run_dir.mkdir(parents=True)
    print(f"run directory: {run_dir}")
    prune_run_dirs(MAX_RUN_DIRS)

    feed_dir = run_dir / "gtfs"
    fetched_at = datetime.now(timezone.utc)
    file_count = download_feed(OPERATOR_ID, feed_dir)
    print(f"downloaded {file_count} GTFS files to {feed_dir}")

    first_service_date = date.today() + timedelta(days=1)
    service_dates = [
        f"{first_service_date + timedelta(days=offset):%Y%m%d}"
        for offset in range(SERVICE_DATE_COUNT)
    ]
    print(f"service dates: {service_dates[0]}..{service_dates[-1]}")

    with psycopg.connect(database_url(), row_factory=dict_row) as connection:
        with connection.cursor() as cursor:
            specs = load_specs(cursor)
            cursor.execute(
                "INSERT INTO gtfs_instance (gtfs_instance_id, gtfs_source_id, "
                "fetched_at) VALUES (%s, %s, %s)",
                (run_id, GTFS_SOURCE_ID, fetched_at),
            )
            connection.commit()

            for service_date in service_dates:
                service_dir = run_dir / f"service_{service_date}"
                print(f"{service_date}:")
                for spec in specs:
                    spec_id = spec["problem_spec_id"]
                    print(f"  {spec_id}")
                    solution = solve_one(
                        spec, service_dir / spec_id, feed_dir, service_date
                    )
                    cursor.execute(
                        "INSERT INTO problem_instance (problem_instance_id, "
                        "gtfs_source_id, gtfs_instance_id, problem_spec_id, "
                        "service_date, data) VALUES (%s, %s, %s, %s, %s, %s)",
                        (
                            f"{run_id}-{spec_id}-{service_date}",
                            GTFS_SOURCE_ID,
                            run_id,
                            spec_id,
                            service_date,
                            Jsonb(solution),
                        ),
                    )
                    # Commit per instance so a later failure keeps this work.
                    connection.commit()

    print(f"done: {len(service_dates) * len(specs)} problem instances")


if __name__ == "__main__":
    try:
        main()
    except RunError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
