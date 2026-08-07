#!/usr/bin/env python3
"""Download the GTFS feed for one 511.org operator.

Usage: ./download-gtfs.py SF [--output-dir feeds]

Extracts into <output-dir>/<operator_id>_<yyyymmdd>, where the date is the
date the data was fetched. Operator ids come from ./list-operators.py.
Reads the API key from the `apikey` file next to this script (gitignored).
"""

import argparse
import io
import sys
import zipfile
from datetime import date
from pathlib import Path

import api511

DEFAULT_OUTPUT_DIR = Path(__file__).parent / "feeds"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operator_id", help="operator id, e.g. SF (see list-operators.py)")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"where to write the feed (default: {DEFAULT_OUTPUT_DIR})",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    raw = api511.get(
        "datafeeds", api_key=api511.read_api_key(), operator_id=args.operator_id
    )

    # An unknown operator id comes back as a 200 with a non-zip error body.
    if not zipfile.is_zipfile(io.BytesIO(raw)):
        body = raw.decode("utf-8-sig", "replace").strip()
        sys.exit(f"511.org did not return a zip for operator {args.operator_id!r}: {body}")

    feed_dir = args.output_dir / f"{args.operator_id}_{date.today():%Y%m%d}"
    feed_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        z.extractall(feed_dir)
        names = z.namelist()
    print(f"Extracted {len(names)} files to {feed_dir}")


if __name__ == "__main__":
    main()
