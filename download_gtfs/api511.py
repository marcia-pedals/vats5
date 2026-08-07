"""Shared helpers for talking to the 511.org transit API."""

import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

BASE_URL = "https://api.511.org/transit"
APIKEY_PATH = Path(__file__).parent / "apikey"


def read_api_key() -> str:
    """Return the API key from the gitignored `apikey` file, or exit."""
    if not APIKEY_PATH.exists():
        sys.exit(
            f"No API key file at {APIKEY_PATH}.\n"
            "Request a token at https://511.org/open-data/token and write it there."
        )
    key = APIKEY_PATH.read_text().strip()
    if not key:
        sys.exit(f"API key file {APIKEY_PATH} is empty.")
    return key


def get(endpoint: str, **params: str) -> bytes:
    """GET `endpoint` with the given query params, or exit with a message."""
    url = f"{BASE_URL}/{endpoint}?{urllib.parse.urlencode(params)}"
    try:
        with urllib.request.urlopen(url) as response:
            return response.read()
    except urllib.error.HTTPError as e:
        # Error bodies are sometimes a full HTML page; keep the message readable.
        body = " ".join(e.read().decode("utf-8-sig", "replace").split())
        if len(body) > 200:
            body = body[:200] + "..."
        sys.exit(f"HTTP {e.code} {e.reason} from 511.org/{endpoint}: {body}")
    except urllib.error.URLError as e:
        sys.exit(f"Could not reach 511.org: {e.reason}")
