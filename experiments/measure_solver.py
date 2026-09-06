#!/usr/bin/env python3
"""Measure a solver on every iteration of an iterative-expansion run.

Runs the solver's benchmark binary on DIR/problem_state_iteration_{n}.json for
every n in DIR."""

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import time

EXPERIMENTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(EXPERIMENTS_DIR)

SOLVERS = {
    "hk": "benchmark_hk",
    "bnb": "benchmark_bnb",
    "2opt": "benchmark_2opt",
}

HK_MAX_GROUPS = 24

TWO_OPT_GRACE_SECONDS = 30

TERMINATE_GRACE_SECONDS = 10

ITERATION_FILE_RE = re.compile(r"^problem_state_iteration_(\d+)\.json$")


def find_iterations(dir_path):
    """Returns [(n, path)] for every problem_state_iteration_{n}.json in
    dir_path, sorted by n. path is joined onto dir_path as given."""
    found = []
    for name in os.listdir(dir_path):
        m = ITERATION_FILE_RE.match(name)
        if m:
            found.append((int(m.group(1)), os.path.join(dir_path, name)))
    if not found:
        raise SystemExit(f"no problem_state_iteration_*.json files in {dir_path}")
    found.sort()
    return found


def num_required_groups(problem_path):
    with open(problem_path) as f:
        problem = json.load(f)
    # `required` is a list of [stop, group representative] pairs.
    return len({rep for _stop, rep in problem["required"]})


def write_json_atomically(path, value):
    """Writes so that a concurrent reader sees either the old or the new file,
    never a partially written one."""
    dir_path = os.path.dirname(os.path.abspath(path))
    os.makedirs(dir_path, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=dir_path, prefix=".tmp-", suffix=".json")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(value, f, indent=2)
            f.write("\n")
        os.replace(tmp_path, path)
    except BaseException:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise


def load_problem_metadata(path):
    if not os.path.exists(path):
        return {"datasets": {}, "problems": {}}
    with open(path) as f:
        metadata = json.load(f)
    if set(metadata) != {"datasets", "problems"}:
        raise ValueError(f"{path} is not a problem metadata file (keys: {sorted(metadata)})")
    return metadata


def default_dataset_name(dir_key):
    """The directory's name, or its parent's when it is a pipeline run's
    "iterations" directory, which says nothing on its own."""
    base = os.path.basename(dir_key)
    if base == "iterations":
        return os.path.basename(os.path.dirname(dir_key))
    return base


def register_dataset(metadata, dir_key):
    """Adds the dataset to metadata["datasets"] if it is new. Returns whether
    anything changed."""
    if dir_key in metadata["datasets"]:
        return False
    metadata["datasets"][dir_key] = {"name": default_dataset_name(dir_key)}
    return True


def update_best_bounds(metadata, key, events):
    """Folds a run's events into metadata["problems"][key] = {"lb": ..., "ub":
    ...}. Returns whether anything changed."""
    entry = metadata["problems"].setdefault(key, {})
    changed = False
    lbs = [e["lb"] for e in events if e["type"] == "lb"]
    ubs = [e["ub"] for e in events if e["type"] == "ub"]
    if lbs and ("lb" not in entry or max(lbs) > entry["lb"]):
        entry["lb"] = max(lbs)
        changed = True
    if ubs and ("ub" not in entry or min(ubs) < entry["ub"]):
        entry["ub"] = min(ubs)
        changed = True
    return changed


def read_events(events_path):
    """Parses the binary's JSON Lines events file into (events, converge_t).
    converge_t is None if the binary never reported convergence."""
    events = []
    converge_t = None
    if not os.path.exists(events_path):
        return events, converge_t
    with open(events_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                # The binary was killed mid-write; everything before this
                # line is intact.
                break
            if event["type"] == "converged":
                converge_t = event["t"]
            elif event["type"] == "lb":
                events.append({"type": "lb", "t": event["t"], "lb": event["lb"]})
            elif event["type"] == "ub":
                events.append({"type": "ub", "t": event["t"], "ub": event["ub"]})
            else:
                raise ValueError(f"unknown event type in {events_path}: {event}")
    return events, converge_t


def run_with_limit(cmd, log_file, wait_seconds):
    """Runs cmd, killing it if it outlives wait_seconds. Returns (exit_code,
    timed_out); exit_code is None when the process was killed."""
    proc = subprocess.Popen(cmd, stdout=log_file, stderr=subprocess.STDOUT)
    try:
        try:
            exit_code = proc.wait(timeout=wait_seconds)
            return exit_code, False
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=TERMINATE_GRACE_SECONDS)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            return None, True
    except BaseException:
        proc.kill()
        proc.wait()
        raise


def measure_one(solver, binary, problem_path, time_limit, known_lb, log_path, events_path):
    cmd = [binary, problem_path, "--events-out", events_path]
    wait_seconds = time_limit
    if solver == "2opt":
        # As many restarts as fit in the time limit; the binary stops itself.
        cmd += ["--quiet", "--restarts", str(2**30), "--time-limit", str(time_limit)]
        if known_lb is not None:
            cmd += ["--known-lb", str(known_lb)]
        wait_seconds = time_limit + TWO_OPT_GRACE_SECONDS

    with open(log_path, "w") as log_file:
        log_file.write("$ " + " ".join(cmd) + "\n")
        log_file.flush()
        exit_code, timed_out = run_with_limit(cmd, log_file, wait_seconds)

    events, converge_t = read_events(events_path)
    result = {
        "events": events,
        "timed_out": timed_out,
        "exit_code": exit_code,
    }
    if converge_t is not None:
        result["converge_t"] = converge_t
    if solver == "2opt" and known_lb is not None:
        result["known_lb"] = known_lb
    if not timed_out and exit_code != 0:
        result["error"] = f"{os.path.basename(binary)} exited with code {exit_code}; see {log_path}"
    return result


def format_bound(seconds):
    if seconds is None:
        return "-"
    return f"{seconds // 3600:02d}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}"


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("dir", help="directory containing problem_state_iteration_{n}.json files")
    parser.add_argument("solver", choices=sorted(SOLVERS))
    parser.add_argument("name", help="name of this measurement, recorded in the results file")
    parser.add_argument(
        "--time-limit", type=float, default=300, metavar="SECONDS",
        help="time limit per iteration (default: 300)",
    )
    parser.add_argument(
        "--out", metavar="FILE",
        help="results JSON file (default: experiments/results/{name}.json, which "
             "must not already exist)",
    )
    parser.add_argument(
        "--problem-metadata", metavar="FILE",
        default=os.path.join(EXPERIMENTS_DIR, "problem_metadata.json"),
        help="global file of dataset names and best-ever lb/ub per problem "
             "(default: %(default)s)",
    )
    parser.add_argument(
        "--bin-dir", metavar="DIR",
        default=os.path.join(REPO_DIR, "server", "build-debug"),
        help="directory containing the benchmark binaries (default: %(default)s)",
    )
    parser.add_argument(
        "--log-dir", metavar="DIR",
        help="keep each run's stdout/stderr and events here (default: a temporary "
             "directory that is deleted afterwards)",
    )
    args = parser.parse_args()

    binary = os.path.join(args.bin_dir, SOLVERS[args.solver])
    if not os.access(binary, os.X_OK):
        raise SystemExit(f"benchmark binary not found: {binary}")
    if args.out is None:
        args.out = os.path.join(EXPERIMENTS_DIR, "results", f"{args.name}.json")
        if os.path.exists(args.out):
            raise SystemExit(f"{args.out} already exists; pick another name or pass --out")

    iterations = find_iterations(args.dir)
    print(f"measuring {args.solver} on {len(iterations)} iterations in {args.dir}", file=sys.stderr)

    metadata = load_problem_metadata(args.problem_metadata)
    if register_dataset(metadata, os.path.abspath(args.dir)):
        write_json_atomically(args.problem_metadata, metadata)
    print(f"writing results to {args.out}", file=sys.stderr)

    with tempfile.TemporaryDirectory(prefix="measure_solver-") as tmp_dir:
        log_dir = args.log_dir or tmp_dir
        os.makedirs(log_dir, exist_ok=True)

        results = []
        output = {"name": args.name, "description": "", "dir": args.dir, "results": results}
        write_json_atomically(args.out, output)
        for iteration, problem_path in iterations:
            groups = num_required_groups(problem_path)
            if args.solver == "hk" and groups > HK_MAX_GROUPS:
                continue

            # Re-read every time: other measurements may be running concurrently.
            metadata = load_problem_metadata(args.problem_metadata)
            problem_key = os.path.abspath(problem_path)
            known_lb = metadata["problems"].get(problem_key, {}).get("lb")

            print(f"iteration {iteration}: {groups} groups ...", file=sys.stderr, end="", flush=True)
            started = time.monotonic()
            measured = measure_one(
                args.solver, binary, problem_path, args.time_limit, known_lb,
                log_path=os.path.join(log_dir, f"iteration_{iteration}.log"),
                events_path=os.path.join(log_dir, f"iteration_{iteration}.events.jsonl"),
            )
            wall = time.monotonic() - started

            entry = {
                "path": problem_path,
                "iteration": iteration,
                "num_required_groups": groups,
                "time_limit": args.time_limit,
            }
            entry.update(measured)
            results.append(entry)
            write_json_atomically(args.out, output)

            metadata = load_problem_metadata(args.problem_metadata)
            if update_best_bounds(metadata, problem_key, entry["events"]):
                write_json_atomically(args.problem_metadata, metadata)

            lbs = [e["lb"] for e in entry["events"] if e["type"] == "lb"]
            ubs = [e["ub"] for e in entry["events"] if e["type"] == "ub"]
            status = (
                f"converged at {entry['converge_t']:.1f}s" if "converge_t" in entry
                else "timed out" if entry["timed_out"]
                else f"FAILED: {entry['error']}" if "error" in entry
                else "did not converge"
            )
            print(
                f" lb {format_bound(max(lbs) if lbs else None)}"
                f" ub {format_bound(min(ubs) if ubs else None)}"
                f" {status} ({wall:.1f}s wall)",
                file=sys.stderr,
            )

    print(f"done: {len(results)} iterations measured, results in {args.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
