#!/usr/bin/env python3
"""Store the solutions a run left on disk but never got into the database.

A run that dies between solving an instance and inserting its row leaves the
solution in its run directory, so a solve that took half an hour does not have
to be repeated to get it stored.

Only instances that actually finished solving are backfilled. A date that
initialize_problem_state skipped as a duplicate has no solution of its own, and
one the run never reached has nothing on disk at all; rather than guess at a row
for either, this refuses to run when it finds a spec directory without a
solution.json. Those dates cost nothing to pick up in the next run.

Rows already in the database are left exactly as they are, so this is safe to
re-run and safe to point at a run that was only partly stored.

DATABASE_URL is taken from the environment, falling back to
solution-viewer/.env.local, the same as the rest of the pipeline. Nothing is
written unless --commit is passed.

Usage:
    python3 pipeline/backfill_run.py RUN_DIR [--commit]
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from psycopg.types.json import Jsonb

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "download_gtfs"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from run import db, shift_trace, trace_node  # noqa: E402


class BackfillError(Exception):
    """Raised for conditions that should stop the backfill."""


def rebuild_solution(spec_dir: Path) -> dict[str, Any]:
    """The row `data` run.py would have written for an instance it solved.

    run.py wraps what iterative_expansion reported in a span covering both
    tools, timed from its own clock. That clock is gone by the time anything is
    backfilled, but the files the two tools wrote are timestamped: the inputs
    are written immediately before initialize_problem_state starts, its log is
    closed when it exits, and iterative_expansion's log is closed when it exits
    in turn. The spans those give back agree with what the run printed for the
    instance to within a rounding step.
    """
    solution = json.loads((spec_dir / "solution.json").read_text())

    # world.toml is written first of the two inputs, so it is the earliest
    # moment run.py's own timer for this instance could have started.
    started = (spec_dir / "world.toml").stat().st_mtime
    initialize_ended = (spec_dir / "initialize_problem_state.log").stat().st_mtime
    expansion_ended = (spec_dir / "iterative_expansion.log").stat().st_mtime

    initialize_duration = initialize_ended - started
    expansion_start = initialize_duration
    expansion_duration = expansion_ended - initialize_ended
    if initialize_duration < 0 or expansion_duration < 0:
        raise BackfillError(
            f"{spec_dir}: its files are timestamped out of order, so the "
            "instance cannot be timed from them"
        )

    inner_trace = solution.pop("trace", None)
    children = [
        trace_node("initialize_problem_state", 0.0, initialize_duration),
        trace_node(
            "iterative_expansion",
            expansion_start,
            expansion_duration,
            [
                shift_trace(child, expansion_start)
                for child in (inner_trace or {}).get("children", [])
            ],
        ),
    ]
    return solution | {
        "trace": trace_node(
            "solve", 0.0, initialize_duration + expansion_duration, children
        )
    }


def find_instances(run_dir: Path) -> list[tuple[str, str, Path]]:
    """Every solved instance of a run, as (service date, spec id, directory)."""
    instances = []
    unsolved = []
    # The service_{date} directories, not the service_patterns_{spec}.json
    # files that sit beside them.
    for service_dir in sorted(run_dir.glob("service_" + "[0-9]" * 8)):
        if not service_dir.is_dir():
            raise BackfillError(f"{service_dir} is not a directory")
        service_date = service_dir.name.removeprefix("service_")
        for spec_dir in sorted(p for p in service_dir.iterdir() if p.is_dir()):
            if (spec_dir / "solution.json").exists():
                instances.append((service_date, spec_dir.name, spec_dir))
            else:
                unsolved.append(spec_dir)

    if unsolved:
        listed = "\n  ".join(str(path) for path in unsolved)
        raise BackfillError(
            "these instances have no solution.json, so there is nothing of "
            f"theirs to store:\n  {listed}\n"
            "They were skipped as duplicates or never reached; re-run the "
            "pipeline to pick them up."
        )
    if not instances:
        raise BackfillError(f"{run_dir} has no solved instances in it")
    return instances


def gtfs_source_id_of(run_dir: Path) -> str:
    """The source a run directory is named after: run_{source}_{timestamp}."""
    name = run_dir.name
    if not name.startswith("run_") or name.count("_") < 2:
        raise BackfillError(
            f"{name!r} is not a run directory name of the form "
            "run_{gtfs_source_id}_{timestamp}"
        )
    return name.removeprefix("run_").rsplit("_", 1)[0]


def main(run_dir: Path, commit: bool) -> None:
    if not run_dir.is_dir():
        raise BackfillError(f"{run_dir} is not a directory")

    run_id = run_dir.name
    gtfs_source_id = gtfs_source_id_of(run_dir)
    instances = find_instances(run_dir)
    print(f"{run_id}: {len(instances)} solved instance(s) on disk")

    # One connection for the whole backfill: it only reads and inserts, so it
    # is never idle for long the way a run is while it solves. Without
    # --commit nothing is written, and the empty transaction commits.
    with db() as cursor:
        # A run inserts this row before it solves anything, so it is normally
        # there already; a run that died earlier than this one may not have got
        # that far.
        cursor.execute(
            "SELECT 1 FROM gtfs_instance WHERE gtfs_instance_id = %s "
            "AND gtfs_source_id = %s",
            (run_id, gtfs_source_id),
        )
        if cursor.fetchone() is None:
            # When the row has to be recreated, the feed directory's timestamp
            # is the closest thing left to the fetch the run recorded.
            fetched_at = datetime.fromtimestamp(
                (run_dir / "gtfs").stat().st_mtime, timezone.utc
            )
            print(f"  gtfs_instance {run_id} is missing, inserting it")
            if commit:
                cursor.execute(
                    "INSERT INTO gtfs_instance (gtfs_instance_id, "
                    "gtfs_source_id, fetched_at) VALUES (%s, %s, %s)",
                    (run_id, gtfs_source_id, fetched_at),
                )

        inserted = 0
        for service_date, spec_id, spec_dir in instances:
            instance_id = f"{run_id}-{spec_id}-{service_date}"
            cursor.execute(
                "SELECT 1 FROM problem_instance WHERE problem_instance_id = %s",
                (instance_id,),
            )
            if cursor.fetchone() is not None:
                print(f"  {instance_id}: already stored, left alone")
                continue

            solution = rebuild_solution(spec_dir)
            print(
                f"  {instance_id}: {solution['status']} "
                f"{solution.get('optimal_duration_seconds')} "
                f"in {solution['trace']['duration_seconds']:.0f}s"
            )
            if commit:
                cursor.execute(
                    "INSERT INTO problem_instance (problem_instance_id, "
                    "gtfs_source_id, gtfs_instance_id, problem_spec_id, "
                    "service_date, data) VALUES (%s, %s, %s, %s, %s, %s)",
                    (
                        instance_id,
                        gtfs_source_id,
                        run_id,
                        spec_id,
                        service_date,
                        Jsonb(solution),
                    ),
                )
            inserted += 1

    if commit:
        print(f"\ninserted {inserted} row(s)")
    else:
        print(f"\nwould insert {inserted} row(s); re-run with --commit")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="the run directory to backfill")
    parser.add_argument(
        "--commit",
        action="store_true",
        help="actually write the rows; without this, only prints what it would do",
    )
    args = parser.parse_args()
    try:
        main(args.run_dir.expanduser(), args.commit)
    except BackfillError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
