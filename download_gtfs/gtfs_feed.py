"""Download and extract a 511.org GTFS feed.

Importable from other tools (see pipeline/run.py); download-gtfs.py is a thin
CLI wrapper around `download_feed`.
"""

import io
import zipfile
from datetime import date
from pathlib import Path

import api511


class FeedDownloadError(Exception):
    """Raised when 511.org does not return a usable feed."""


def download_feed(operator_id: str, feed_dir: Path) -> int:
    """Download `operator_id`'s feed and extract it into `feed_dir`.

    Returns the number of extracted files. Raises FeedDownloadError if the
    response is not a zip, which is how 511.org reports an unknown operator.
    """
    raw = api511.get("datafeeds", api_key=api511.read_api_key(), operator_id=operator_id)

    # An unknown operator id comes back as a 200 with a non-zip error body.
    if not zipfile.is_zipfile(io.BytesIO(raw)):
        body = raw.decode("utf-8-sig", "replace").strip()
        raise FeedDownloadError(
            f"511.org did not return a zip for operator {operator_id!r}: {body}"
        )

    feed_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        z.extractall(feed_dir)
        return len(z.namelist())


def dated_feed_dir(output_dir: Path, operator_id: str) -> Path:
    """The conventional <output_dir>/<operator_id>_<yyyymmdd> feed location."""
    return output_dir / f"{operator_id}_{date.today():%Y%m%d}"
