## Build Instructions

- To build, run ninja in server/build-debug
- To run one test, run the specific test executable
- To run all tests, run `ctest --label-exclude slow` in server/build-debug
- Some tests are labeled "slow". Only run these when explicitly needed.
