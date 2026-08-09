## Build Instructions

- To build, run ninja in server/build-debug
- To run one test, run the specific test executable
- To run all tests, run `ctest --label-exclude slow` in server/build-debug
- Some tests are labeled "slow". Only run these when explicitly needed.

## Code Style

- Never skip or return defaults for entries that cannot be processed. Instead, encode the possibility of failure in types (e.g. use optionals), or fail (e.g. throw exception).

## Visualization Server

- Use `bin/viz` to manage the visualization dev server. Do NOT run `npm run dev` directly.
- `bin/viz status` — check if the server is already running before starting one
- `bin/viz start` — start the server (refuses if already running)
- `bin/viz stop` / `bin/viz restart` — stop or restart
- `bin/viz url` — print the URL
- `bin/viz logs` — tail the log file
- Always run `bin/viz status` before attempting to start the server.

## Solution Viewer

- `solution-viewer/` is a separate Vite/tRPC app backed by PostgreSQL (Neon in production, a checkout-local cluster in development).
- Use `bin/sv` to manage its dev server (same commands as `bin/viz`). Do NOT run `npm run dev` directly.
- Use `bin/pg` to manage the local PostgreSQL cluster: `bin/pg init` on first setup, then `start`/`stop`/`status`/`psql`.
- Always run `bin/sv status` / `bin/pg status` before attempting to start either.
- Schema changes go in a new numbered file in `solution-viewer/migrations/`, applied with `npm run migrate`. Never edit an applied migration.

## Pull Requests

- When creating a PR, always use a blank description (empty `--body ""`).
- When updating a PR, always create new commit(s) and push them. Never amend existing commits.
