#!/usr/bin/env python3
"""List the GTFS operators available from the 511.org transit API.

Reads the API key from the `apikey` file next to this script (gitignored).
Get a key at https://511.org/open-data/token
"""

import json
import sys

import api511


def main() -> None:
    # 511.org serves JSON with a UTF-8 BOM.
    raw = api511.get("gtfsoperators", api_key=api511.read_api_key(), format="json")
    operators = json.loads(raw.decode("utf-8-sig"))
    if not isinstance(operators, list):
        sys.exit(f"Unexpected response shape: {operators!r}")

    width = max((len(str(op.get("Id", ""))) for op in operators), default=0)
    for op in operators:
        print(f"{str(op.get('Id', '')):<{width}}  {op.get('Name', '')}")
    print(f"\n{len(operators)} operators", file=sys.stderr)


if __name__ == "__main__":
    main()
