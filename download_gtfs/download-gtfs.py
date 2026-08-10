#!/usr/bin/env python3
"""Download the GTFS feed for one 511.org operator.

Usage: ./download-gtfs.py SF [--output-dir feeds]

Extracts into <output-dir>/<operator_id>_<yyyymmdd>, where the date is the
date the data was fetched. Operator ids come from ./list-operators.py.
Reads the API key from the `apikey` file next to this script (gitignored).
"""

import argparse
import sys
from pathlib import Path

from gtfs_feed import FeedDownloadError, dated_feed_dir, download_feed

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

    feed_dir = dated_feed_dir(args.output_dir, args.operator_id)
    try:
        file_count = download_feed(args.operator_id, feed_dir)
    except FeedDownloadError as error:
        sys.exit(str(error))
    print(f"Extracted {file_count} files to {feed_dir}")


if __name__ == "__main__":
    main()
