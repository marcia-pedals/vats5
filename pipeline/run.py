#!/usr/bin/env python3
"""Download the latest Bay Area GTFS, then create and solve problem instances.

Each run gets its own directory under ~/vats5-pipeline:

    run_{timestamp}/
        gtfs/                                  the downloaded RG feed
        service_patterns_{problem_spec_id}.json  services of the dates solved so far
        service_{date}/
            {problem_spec_id}/
                world.toml                     input for initialize_problem_state
                target_stops.toml              input for initialize_problem_state
                problem_state.json             its output (+ .sqlite for viz)
                problem_state-active-services.json  the services this date uses
                initialize_problem_state.log
                iterative_expansion.log
                solution.json                  what gets stored in the db

A service date whose active services are identical to an already-solved one
poses the same problem, so initialize_problem_state stops as soon as it sees
that and the date is recorded as a duplicate instead of being solved again.

The run directory name doubles as the gtfs_instance_id. Older run directories
are pruned, which does not affect the rows already written for them.

Problem specs are read from the database, so run pipeline/sync_configs.py first.

Usage:
    python3 pipeline/run.py [--service-dates N]
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
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
DEFAULT_SERVICE_DATE_COUNT = 7

# Passed to iterative_expansion, which checks it between iterations and on each
# search event. The subprocess gets a slightly longer hard limit as a backstop
# for a single Concorde solve that blows through the cooperative check.
SOLVE_TIMEOUT_SECONDS = 600
SOLVE_KILL_GRACE_SECONDS = 60

# What initialize_problem_state exits with when this service date's services are
# identical to an already-solved one's; see kDuplicateServicePatternExitCode.
DUPLICATE_SERVICE_PATTERN_EXIT_CODE = 3

# The status stored for such a date. The solution-viewer leaves these rows out.
DUPLICATE_STATUS = "duplicate_service_pattern"


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


def trace_node(
    name: str,
    start_seconds: float,
    duration_seconds: float,
    children: list[dict[str, Any]] | None = None,
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """One span of a hierarchical timing trace.

    `start_seconds` is relative to the start of the trace's root span, so a
    whole trace can be laid out without walking it.
    """
    node: dict[str, Any] = {
        "name": name,
        "start_seconds": start_seconds,
        "duration_seconds": duration_seconds,
    }
    if metadata:
        node["metadata"] = metadata
    if children:
        node["children"] = children
    return node


def shift_trace(node: dict[str, Any], offset: float) -> dict[str, Any]:
    """Rebase a trace produced by a subprocess onto this run's timeline."""
    shifted = dict(node)
    shifted["start_seconds"] = node["start_seconds"] + offset
    if "children" in node:
        shifted["children"] = [shift_trace(c, offset) for c in node["children"]]
    return shifted


def active_services_path(problem_state: Path) -> Path:
    """Where initialize_problem_state writes its service pattern.

    Mirrors ActiveServicesPath in server/src/solver/service_pattern.h.
    """
    return problem_state.with_name(f"{problem_state.stem}-active-services.json")


def solve_one(
    spec: dict[str, Any],
    spec_dir: Path,
    feed_dir: Path,
    service_date: str,
    solved_patterns_json: Path,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    """Initialize and solve one spec for one service date.

    Returns the solution info and this date's service pattern, the latter only
    when there is one to add to `solved_patterns_json` for later dates -- a
    duplicate date contributes nothing new, and a failed initialization has no
    pattern at all.

    The solution info carries a `trace` of how long each part took: the two
    tools as timed here, with whatever spans they reported nested underneath.
    """
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
        "--solved-service-patterns",
        str(solved_patterns_json),
    ]
    if "max_walking_distance" in spec_data:
        initialize_command += [
            "--max-walking-distance",
            str(spec_data["max_walking_distance"]),
        ]
    if "walking_speed" in spec_data:
        initialize_command += ["--walking-speed", str(spec_data["walking_speed"])]
    if "subsequent_service_days" in spec_data:
        initialize_command += [
            "--subsequent-service-days",
            str(spec_data["subsequent_service_days"]),
        ]

    started = time.monotonic()
    returncode = run_tool(
        initialize_command, spec_dir / "initialize_problem_state.log", timeout=None
    )
    initialize_node = trace_node(
        "initialize_problem_state", 0.0, time.monotonic() - started
    )

    def finish(
        solution: dict[str, Any],
        children: list[dict[str, Any]],
        outcome: str,
        pattern: dict[str, Any] | None = None,
    ) -> tuple[dict[str, Any], dict[str, Any] | None]:
        """Report how this instance went and how long it took, and trace it."""
        elapsed = time.monotonic() - started
        print(f"    {outcome} in {elapsed:.0f}s")
        solution = solution | {"trace": trace_node("solve", 0.0, elapsed, children)}
        return solution, pattern

    if returncode == DUPLICATE_SERVICE_PATTERN_EXIT_CODE:
        # The pattern it wrote says which already-solved date this one matched.
        duplicate_of = json.loads(active_services_path(problem_state).read_text())[
            "duplicate_of"
        ]
        return finish(
            {
                "status": DUPLICATE_STATUS,
                "duplicate_of_service_date": duplicate_of["service_date"],
                "matched_on": duplicate_of["matched_on"],
            },
            [initialize_node],
            f"same {duplicate_of['matched_on']} as "
            f"{duplicate_of['service_date']}, skipped",
        )

    if returncode != 0:
        return finish(
            {"status": "initialize_failed", "returncode": returncode},
            [initialize_node],
            f"initialize_problem_state failed (exit {returncode})",
        )

    pattern = json.loads(active_services_path(problem_state).read_text())

    expansion_start = time.monotonic() - started
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
    expansion_duration = time.monotonic() - started - expansion_start

    # iterative_expansion writes solution.json for every outcome it reaches on
    # its own, including a timeout; a missing file means it died first.
    if solution_json.exists():
        solution = json.loads(solution_json.read_text())
        outcome = f"{solution['status']}: {solution.get('optimal_duration_seconds')}"
    else:
        solution = {
            "status": "killed" if returncode is None else "solve_failed",
            "returncode": returncode,
        }
        outcome = f"{solution['status']} (exit {returncode})"

    # iterative_expansion times itself from its own start, which is a process
    # spawn after ours; its spans go under the span timed here.
    inner_trace = solution.pop("trace", None)
    expansion_node = trace_node(
        "iterative_expansion",
        expansion_start,
        expansion_duration,
        [
            shift_trace(child, expansion_start)
            for child in (inner_trace or {}).get("children", [])
        ],
    )
    # The pattern goes on the solved list whatever the solve did with it: a
    # later date with the same services would reach the same outcome.
    return finish(solution, [initialize_node, expansion_node], outcome, pattern)


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


def main(service_date_count: int) -> None:
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
        for offset in range(service_date_count)
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

            # Kept per spec: which services matter depends on the spec's target
            # stops and walking options, so patterns are not comparable across
            # specs.
            solved_patterns: dict[str, list[dict[str, Any]]] = {
                spec["problem_spec_id"]: [] for spec in specs
            }
            skipped = 0

            for service_date in service_dates:
                service_dir = run_dir / f"service_{service_date}"
                print(f"{service_date}:")
                for spec in specs:
                    spec_id = spec["problem_spec_id"]
                    print(f"  {spec_id}")
                    patterns_json = run_dir / f"service_patterns_{spec_id}.json"
                    patterns_json.write_text(json.dumps(solved_patterns[spec_id]))
                    solution, pattern = solve_one(
                        spec,
                        service_dir / spec_id,
                        feed_dir,
                        service_date,
                        patterns_json,
                    )
                    if pattern is not None:
                        solved_patterns[spec_id].append(pattern)
                    if solution["status"] == DUPLICATE_STATUS:
                        skipped += 1
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

    total = len(service_dates) * len(specs)
    print(
        f"done: {total} problem instances, {skipped} of them skipped as "
        "duplicates of an earlier service date"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--service-dates",
        type=int,
        default=DEFAULT_SERVICE_DATE_COUNT,
        help="how many service dates to solve, starting the day after the fetch "
        f"(default {DEFAULT_SERVICE_DATE_COUNT})",
    )
    args = parser.parse_args()
    if args.service_dates < 1:
        parser.error("--service-dates must be at least 1")

    try:
        main(args.service_dates)
    except RunError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
