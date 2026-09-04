#pragma once

// Thin C wrapper around Concorde's CCtsp_solve_dat.
//
// This lives in a C translation unit because concorde.h is not usable from
// C++: it has no extern "C" guards, uses `new` as a parameter name, and
// #defines NDEBUG (which would silently disable assert() in any C++ file that
// included it).

#ifdef __cplusplus
extern "C" {
#endif

// Solves the symmetric TSP on `ncount` nodes (numbered 0..ncount-1).
//
// `elist` holds `ecount` undirected edges as pairs (elist[2i], elist[2i+1])
// with length elen[i]. Node pairs not listed get length `default_len`.
//
// `upper_bound` may be NULL. When set it is passed to Concorde as the initial
// upper bound: Concorde then only searches for tours better than it, and
// *found_tour is 0 if none exists.
//
// `seed` seeds Concorde's random state, so runs are reproducible.
//
// Concorde writes checkpoint and cut-pool files relative to the current
// working directory, and prints progress unconditionally to stdout/stderr.
// For the duration of the call the process is chdir'd into `work_dir` and
// file descriptors 1 and 2 are redirected to `log_path` (appended). Both are
// restored before returning. Because of this the function is NOT
// thread-safe and must not run concurrently with anything else that uses the
// cwd or stdio.
//
// Returns 0 if Concorde ran to completion, nonzero if it reported an internal
// failure (or the redirect setup failed). On 0, *success and *found_tour
// mirror CCtsp_solve_dat's outputs, *optval is the tour value, and out_tour
// (which must have room for ncount ints) holds the tour when *found_tour is 1.
int vats5_concorde_solve(
    int ncount,
    int ecount,
    const int* elist,
    const int* elen,
    int default_len,
    const double* upper_bound,
    int seed,
    const char* work_dir,
    const char* log_path,
    int* success,
    int* found_tour,
    double* optval,
    int* out_tour
);

#ifdef __cplusplus
}
#endif
