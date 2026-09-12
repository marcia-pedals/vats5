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

// Solves only the root LP relaxation of the symmetric TSP: Concorde's
// cutting-plane loop at the root of its search tree, with no branching.
// Inputs are as for vats5_concorde_solve (including the cwd/stdio redirect
// caveats, so this is NOT thread-safe either).
//
// Returns 0 if Concorde ran to completion, nonzero on internal failure. On 0:
//   *infeasible is 1 if Concorde proved the LP infeasible (no tour exists);
//     the remaining outputs are then untouched.
//   *lp_bound is the final LP objective value (a valid lower bound on the
//     optimal tour, up to floating point).
//   *exact_bound is Concorde's exactly-priced lower bound (rigorous).
//   *xcount edges with LP value > CCtsp_INTTOL (the support) are returned as
//     node pairs (xlist[2i], xlist[2i+1]) in the caller's node numbering with
//     values x[i]. Both arrays are malloc'd and must be free()d by the caller.
int vats5_concorde_root_lp(
    int ncount,
    int ecount,
    const int* elist,
    const int* elen,
    int default_len,
    int seed,
    const char* work_dir,
    const char* log_path,
    int* infeasible,
    double* lp_bound,
    double* exact_bound,
    int* xcount,
    int** xlist,
    double** x
);

#ifdef __cplusplus
}
#endif
