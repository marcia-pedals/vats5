## Build Instructions

- To build, run ninja in server/build-debug
- To run one test, run the specific test executable
- To run all tests, run `ctest --label-exclude slow` in server/build-debug
- Some tests are labeled "slow". Only run these when explicitly needed.

## Visualization Server

- Use `bin/viz` to manage the visualization dev server. Do NOT run `npm run dev` directly.
- `bin/viz status` — check if the server is already running before starting one
- `bin/viz start` — start the server (refuses if already running)
- `bin/viz stop` / `bin/viz restart` — stop or restart
- `bin/viz url` — print the URL
- `bin/viz logs` — tail the log file
- Always run `bin/viz status` before attempting to start the server.
