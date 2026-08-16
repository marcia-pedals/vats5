#!/usr/bin/env python3
"""Download one GTFS source's latest feed, then create and solve its instances.

Which sources there are, and how each one's feed is fetched, is in
gtfs_sources.py; this solves the specs of exactly one of them per run.

Each run gets its own directory under ~/vats5-pipeline:

    run_{gtfs_source_id}_{timestamp}/
        gtfs/                                  the downloaded feed
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
that and the date is recorded as a duplicate instead of being solved again. Its
row copies the duplicated date's optimal duration, so that querying for a date's
duration never has to follow the duplicate to another row.

The run directory name doubles as the gtfs_instance_id. Older run directories of
the same source are pruned, which does not affect the rows already written for
them.

Problem specs are read from the database, so run pipeline/sync_configs.py first.
Each one is solved for --service-dates days running from the date it starts at:
the day after the fetch, or the `start_service_date` the spec pins itself to.

Usage:
    python3 pipeline/run.py GTFS_SOURCE_ID [--service-dates N] [--solve-timeout S]
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

from gtfs_feed import FeedDownloadError  # noqa: E402
from gtfs_sources import GTFS_SOURCES, UnknownGtfsSource, get_source  # noqa: E402
from sync_configs import database_url  # noqa: E402

PIPELINE_ROOT = Path.home() / "vats5-pipeline"
BUILD_DIR = REPO_ROOT / "server" / "build-debug"

# Number of run directories to keep on disk per source, newest first.
MAX_RUN_DIRS = 5

# Service dates to solve: this many days, starting the day after the fetch.
DEFAULT_SERVICE_DATE_COUNT = 7

# Passed to iterative_expansion, which checks it between iterations and on each
# search event. The subprocess gets a slightly longer hard limit as a backstop
# for a single Concorde solve that blows through the cooperative check.
DEFAULT_SOLVE_TIMEOUT_SECONDS = 600
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


def run_dir_prefix(gtfs_source_id: str) -> str:
    """What every run directory of one source is named after.

    Sources are pruned independently, so a run for one of them never evicts
    another's work, and the fixed-width timestamp that follows this orders
    directories by age lexicographically.
    """
    return f"run_{gtfs_source_id}_"


def prune_run_dirs(gtfs_source_id: str, keep: int) -> None:
    """Delete all but the `keep` newest run directories of one source."""
    run_dirs = sorted(
        (
            path
            for path in PIPELINE_ROOT.glob(f"{run_dir_prefix(gtfs_source_id)}*")
            if path.is_dir()
        ),
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


def write_world_toml(
    path: Path, feed_dir: Path, service_date: str, trip_id_prefixes: list[str]
) -> None:
    """Write the GtfsFilterConfig for one service date of the feed.

    `trip_id_prefixes` narrows the world to the trips whose ids start with one
    of them, which is how a spec asks to be solved on one mode or operator
    rather than everything the feed carries. Empty keeps the whole feed.
    """
    sections = [f'input_dir = "{feed_dir}"', f'date = "{service_date}"']
    if trip_id_prefixes:
        sections.append(toml_string_array("prefixes", trip_id_prefixes))
    path.write_text("\n".join(sections) + "\n")


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
    solved_solutions: dict[str, dict[str, Any]],
    solve_timeout_seconds: float,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    """Initialize and solve one spec for one service date.

    Returns the solution info and this date's service pattern, the latter only
    when there is one to add to `solved_patterns_json` for later dates -- a
    duplicate date contributes nothing new, and a failed initialization has no
    pattern at all.

    `solved_solutions` holds this spec's already-solved dates by service date,
    matching `solved_patterns_json`, so that a duplicate date can carry the
    result of the date it duplicates.

    The solution info carries a `trace` of how long each part took: the two
    tools as timed here, with whatever spans they reported nested underneath,
    and the spec's `description`, so a stored solution says what problem it was
    a solution to even after the config has moved on.
    """
    spec_dir.mkdir(parents=True, exist_ok=True)

    world_toml = spec_dir / "world.toml"
    target_stops_toml = spec_dir / "target_stops.toml"
    problem_state = spec_dir / "problem_state.json"
    solution_json = spec_dir / "solution.json"

    spec_data = spec["data"]
    write_world_toml(
        world_toml, feed_dir, service_date, spec_data.get("trip_id_prefixes", [])
    )
    write_target_stops_toml(target_stops_toml, spec["target_stops_data"])

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
        solution = solution | {
            "problem_spec_description": spec["description"],
            "trace": trace_node("solve", 0.0, elapsed, children),
        }
        return solution, pattern

    if returncode == DUPLICATE_SERVICE_PATTERN_EXIT_CODE:
        # The pattern it wrote says which already-solved date this one matched.
        duplicate_of = json.loads(active_services_path(problem_state).read_text())[
            "duplicate_of"
        ]
        duplicated_date = duplicate_of["service_date"]
        if duplicated_date not in solved_solutions:
            raise RunError(
                f"{service_date} was skipped as a duplicate of {duplicated_date}, "
                "which is not one of this spec's solved service dates "
                f"({sorted(solved_solutions)})"
            )
        solution: dict[str, Any] = {
            "status": DUPLICATE_STATUS,
            "duplicate_of_service_date": duplicated_date,
            "matched_on": duplicate_of["matched_on"],
        }
        # Copied so that a query for the duration of this date does not have to
        # join with the duplicated date's row. Absent when that date has no
        # duration either, e.g. when its solve timed out.
        duplicated_duration = solved_solutions[duplicated_date].get(
            "optimal_duration_seconds"
        )
        if duplicated_duration is not None:
            solution["optimal_duration_seconds"] = duplicated_duration
        return finish(
            solution,
            [initialize_node],
            f"same {duplicate_of['matched_on']} as {duplicated_date} "
            f"({duplicated_duration}), skipped",
        )

    if returncode != 0:
        return finish(
            {"status": "initialize_failed", "returncode": returncode},
            [initialize_node],
            f"initialize_problem_state failed (exit {returncode})",
        )

    pattern = json.loads(active_services_path(problem_state).read_text())

    expansion_start = time.monotonic() - started
    # A timeout of 0 means iterative_expansion runs to completion, and then
    # there is no deadline for the hard limit to back up either.
    kill_timeout = (
        solve_timeout_seconds + SOLVE_KILL_GRACE_SECONDS
        if solve_timeout_seconds > 0
        else None
    )
    returncode = run_tool(
        [
            str(tool_path("iterative_expansion")),
            str(problem_state),
            "--solution_json",
            str(solution_json),
            "--timeout",
            str(solve_timeout_seconds),
        ],
        spec_dir / "iterative_expansion.log",
        timeout=kill_timeout,
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


def load_specs(cursor: psycopg.Cursor, gtfs_source_id: str) -> list[dict[str, Any]]:
    cursor.execute(
        """
        SELECT spec.problem_spec_id,
               spec.description,
               spec.data,
               stops.data AS target_stops_data
        FROM problem_spec AS spec
        JOIN target_stops AS stops
          ON stops.target_stops_id = spec.target_stops_id
         AND stops.gtfs_source_id = spec.gtfs_source_id
        WHERE spec.gtfs_source_id = %s
        ORDER BY spec.problem_spec_id
        """,
        (gtfs_source_id,),
    )
    specs = cursor.fetchall()
    if not specs:
        raise RunError(
            f"no problem_spec rows for {gtfs_source_id!r} -- "
            "run pipeline/sync_configs.py first"
        )
    return specs


def spec_service_dates(
    spec: dict[str, Any], service_date_count: int
) -> list[str]:
    """The service dates to solve `spec` for, earliest first.

    A spec can pin the date to start from, for a problem that only makes sense
    on particular days -- a network with a months-long closure in it poses a
    different problem than the same network intact, and a spec that means to be
    the latter has to say which dates those are. Without one, a spec starts the
    day after the fetch, which is how a spec meant to track the current
    timetable keeps up with it.
    """
    pinned = spec["data"].get("start_service_date")
    if pinned is None:
        first = date.today() + timedelta(days=1)
    else:
        try:
            first = datetime.strptime(pinned, "%Y%m%d").date()
        except (TypeError, ValueError) as error:
            raise RunError(
                f"{spec['problem_spec_id']}: start_service_date {pinned!r} is "
                "not a YYYYMMDD date"
            ) from error
    return [
        f"{first + timedelta(days=offset):%Y%m%d}"
        for offset in range(service_date_count)
    ]


def main(
    gtfs_source_id: str, service_date_count: int, solve_timeout_seconds: float
) -> None:
    # Resolve the source and both tools up front so an unknown source or a
    # missing build fails before downloading.
    source = get_source(gtfs_source_id)
    tool_path("initialize_problem_state")
    tool_path("iterative_expansion")

    run_id = f"{run_dir_prefix(gtfs_source_id)}{datetime.now():%Y%m%d-%H%M%S}"
    run_dir = PIPELINE_ROOT / run_id
    run_dir.mkdir(parents=True)
    print(f"run directory: {run_dir}")
    prune_run_dirs(gtfs_source_id, MAX_RUN_DIRS)

    feed_dir = run_dir / "gtfs"
    fetched_at = datetime.now(timezone.utc)
    file_count = source.download(feed_dir)
    print(f"downloaded {file_count} GTFS files for {source.title} to {feed_dir}")

    with psycopg.connect(database_url(), row_factory=dict_row) as connection:
        with connection.cursor() as cursor:
            specs = load_specs(cursor, gtfs_source_id)
            cursor.execute(
                "INSERT INTO gtfs_instance (gtfs_instance_id, gtfs_source_id, "
                "fetched_at) VALUES (%s, %s, %s)",
                (run_id, gtfs_source_id, fetched_at),
            )
            connection.commit()

            # Kept per spec: which services matter depends on the spec's target
            # stops and walking options, so patterns are not comparable across
            # specs.
            solved_patterns: dict[str, list[dict[str, Any]]] = {
                spec["problem_spec_id"]: [] for spec in specs
            }
            # The solutions those patterns came from, by service date, so that a
            # duplicate date can copy the result of the date it duplicates.
            solved_solutions: dict[str, dict[str, dict[str, Any]]] = {
                spec["problem_spec_id"]: {} for spec in specs
            }
            skipped = 0
            total = 0

            # Specs are the outer loop because each one has its own dates, and
            # because a date is only a duplicate of the ones already solved for
            # the same spec.
            for spec in specs:
                spec_id = spec["problem_spec_id"]
                service_dates = spec_service_dates(spec, service_date_count)
                print(f"{spec_id}: {service_dates[0]}..{service_dates[-1]}")
                total += len(service_dates)
                for service_date in service_dates:
                    print(f"  {service_date}")
                    patterns_json = run_dir / f"service_patterns_{spec_id}.json"
                    patterns_json.write_text(json.dumps(solved_patterns[spec_id]))
                    solution, pattern = solve_one(
                        spec,
                        run_dir / f"service_{service_date}" / spec_id,
                        feed_dir,
                        service_date,
                        patterns_json,
                        solved_solutions[spec_id],
                        solve_timeout_seconds,
                    )
                    if pattern is not None:
                        solved_patterns[spec_id].append(pattern)
                        solved_solutions[spec_id][service_date] = solution
                    if solution["status"] == DUPLICATE_STATUS:
                        skipped += 1
                    cursor.execute(
                        "INSERT INTO problem_instance (problem_instance_id, "
                        "gtfs_source_id, gtfs_instance_id, problem_spec_id, "
                        "service_date, data) VALUES (%s, %s, %s, %s, %s, %s)",
                        (
                            f"{run_id}-{spec_id}-{service_date}",
                            gtfs_source_id,
                            run_id,
                            spec_id,
                            service_date,
                            Jsonb(solution),
                        ),
                    )
                    # Commit per instance so a later failure keeps this work.
                    connection.commit()

    print(
        f"done: {total} problem instances, {skipped} of them skipped as "
        "duplicates of an earlier service date"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "gtfs_source_id",
        choices=sorted(GTFS_SOURCES),
        help="which GTFS source to fetch and solve, as named in gtfs_sources.py",
    )
    parser.add_argument(
        "--service-dates",
        type=int,
        default=DEFAULT_SERVICE_DATE_COUNT,
        help="how many service dates to solve per spec, from the date it starts "
        f"at (default {DEFAULT_SERVICE_DATE_COUNT})",
    )
    parser.add_argument(
        "--solve-timeout",
        type=float,
        default=DEFAULT_SOLVE_TIMEOUT_SECONDS,
        help="how many seconds to give each solve, or 0 to let it run to "
        f"completion (default {DEFAULT_SOLVE_TIMEOUT_SECONDS})",
    )
    args = parser.parse_args()
    if args.service_dates < 1:
        parser.error("--service-dates must be at least 1")
    if args.solve_timeout < 0:
        parser.error("--solve-timeout must not be negative")

    try:
        main(args.gtfs_source_id, args.service_dates, args.solve_timeout)
    except (RunError, UnknownGtfsSource, FeedDownloadError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
