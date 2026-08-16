"""Download and extract a GTFS feed, from 511.org or from a plain zip URL.

Importable from other tools (see pipeline/gtfs_sources.py); download-gtfs.py is
a thin CLI wrapper around `download_feed`.
"""

import io
import urllib.error
import urllib.request
import zipfile
from datetime import date
from pathlib import Path

import api511


# Sent when downloading a feed by URL. VBB answers urllib's default
# "Python-urllib/3.x" with a 403, and identifying the client is the polite thing
# to do when pulling tens of megabytes off someone's server anyway.
USER_AGENT = "vats5-gtfs-pipeline (+https://github.com/marcia-pedals/vats5)"


class FeedDownloadError(Exception):
    """Raised when a feed source does not return a usable feed."""


def _extract_feed(raw: bytes, feed_dir: Path, source: str) -> int:
    """Extract the zip in `raw` into `feed_dir`; returns the file count.

    Raises FeedDownloadError if `raw` is not a zip at all, which is how a feed
    source reports an error it served with a 200.
    """
    if not zipfile.is_zipfile(io.BytesIO(raw)):
        body = raw.decode("utf-8-sig", "replace").strip()
        raise FeedDownloadError(f"{source} did not return a zip: {body}")

    feed_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        z.extractall(feed_dir)
        return len(z.namelist())


def download_feed(operator_id: str, feed_dir: Path) -> int:
    """Download `operator_id`'s 511.org feed and extract it into `feed_dir`.

    Returns the number of extracted files. Raises FeedDownloadError if the
    response is not a zip, which is how 511.org reports an unknown operator.
    """
    raw = api511.get("datafeeds", api_key=api511.read_api_key(), operator_id=operator_id)
    return _extract_feed(raw, feed_dir, f"511.org (operator {operator_id!r})")


def download_feed_from_url(url: str, feed_dir: Path) -> int:
    """Download the GTFS zip at `url` and extract it into `feed_dir`.

    Returns the number of extracted files. Feeds published this way are whole
    regional networks and run to tens of megabytes, so this holds one in memory
    only as long as it takes to extract.
    """
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(request) as response:
            raw = response.read()
    except urllib.error.HTTPError as error:
        raise FeedDownloadError(f"HTTP {error.code} {error.reason} from {url}") from error
    except urllib.error.URLError as error:
        raise FeedDownloadError(f"could not reach {url}: {error.reason}") from error

    return _extract_feed(raw, feed_dir, url)


def dated_feed_dir(output_dir: Path, operator_id: str) -> Path:
    """The conventional <output_dir>/<operator_id>_<yyyymmdd> feed location."""
    return output_dir / f"{operator_id}_{date.today():%Y%m%d}"
