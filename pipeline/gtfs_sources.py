"""The GTFS sources problems can be solved for, and how to fetch each feed.

This is the source of truth for the `gtfs_source` rows: sync_configs.py writes
one row per source here, run.py takes the id of one of them and downloads its
feed. Configs under problem_configs/ name the source they belong to, and the
solutions stored for a source's problem specs are keyed off its id, so an id
here can be added but not renamed.
"""

from __future__ import annotations

import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "download_gtfs"))

from gtfs_feed import download_feed, download_feed_from_url  # noqa: E402


class UnknownGtfsSource(Exception):
    """Raised for a gtfs_source_id that is not one of GTFS_SOURCES."""


@dataclass(frozen=True)
class GtfsSource(ABC):
    """A feed to solve problems for. Subclasses say where the feed comes from."""

    gtfs_source_id: str
    title: str

    @abstractmethod
    def download(self, feed_dir: Path) -> int:
        """Download this source's feed into `feed_dir`; returns the file count."""


@dataclass(frozen=True)
class Api511Source(GtfsSource):
    """A feed served by the 511.org API, which needs an API key and an operator."""

    operator_id: str

    def download(self, feed_dir: Path) -> int:
        return download_feed(self.operator_id, feed_dir)


@dataclass(frozen=True)
class ZipUrlSource(GtfsSource):
    """A feed published as a zip at a stable URL.

    The URL is the one the operator advertises as current; they replace the file
    behind it every few weeks rather than versioning it, so a fetch always gets
    whatever timetable is in force now.
    """

    feed_url: str

    def download(self, feed_dir: Path) -> int:
        return download_feed_from_url(self.feed_url, feed_dir)


GTFS_SOURCES: dict[str, GtfsSource] = {
    source.gtfs_source_id: source
    for source in (
        Api511Source(
            gtfs_source_id="bayarea",
            title="Bay Area 511.org Regional",
            operator_id="RG",
        ),
        ZipUrlSource(
            gtfs_source_id="berlin",
            title="Berlin-Brandenburg VBB",
            # Redirects to the current https://www.vbb.de/gtfs file.
            feed_url="https://www.vbb.de/vbbgtfs",
        ),
        ZipUrlSource(
            gtfs_source_id="munich",
            title="Munich MVV",
            # The MVV-Gesamtnetz feed: S-Bahn, U-Bahn, tram, bus and the
            # regional trains within the MVV area.
            feed_url=(
                "https://www.mvv-muenchen.de/fileadmin/mediapool/developer"
                "/opendata/gesamt_gtfs.zip"
            ),
        ),
    )
}


def get_source(gtfs_source_id: str) -> GtfsSource:
    """The source with this id, or raise UnknownGtfsSource listing the ids."""
    source = GTFS_SOURCES.get(gtfs_source_id)
    if source is None:
        raise UnknownGtfsSource(
            f"unknown gtfs_source_id {gtfs_source_id!r} -- "
            f"known sources are {', '.join(sorted(GTFS_SOURCES))}"
        )
    return source
