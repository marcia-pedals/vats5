#include "solver/concorde_shim.h"

#include <concorde.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Saved process state while cwd and stdio are redirected into the work dir.
typedef struct {
  char* saved_cwd;
  int saved_stdout;
  int saved_stderr;
} Redirect;

// Returns 0 on success. On failure nothing is left redirected.
static int RedirectBegin(
    const char* work_dir, const char* log_path, Redirect* r
) {
  r->saved_cwd = getcwd(NULL, 0);
  if (r->saved_cwd == NULL) {
    fprintf(stderr, "concorde_shim: getcwd failed\n");
    return -1;
  }

  int log_fd = open(log_path, O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (log_fd < 0) {
    fprintf(stderr, "concorde_shim: failed to open log %s\n", log_path);
    free(r->saved_cwd);
    return -1;
  }

  // Flush C stdio so buffered output doesn't end up in the log file.
  fflush(stdout);
  fflush(stderr);

  r->saved_stdout = dup(STDOUT_FILENO);
  r->saved_stderr = dup(STDERR_FILENO);
  if (r->saved_stdout < 0 || r->saved_stderr < 0) {
    fprintf(stderr, "concorde_shim: dup failed\n");
    close(log_fd);
    free(r->saved_cwd);
    return -1;
  }

  if (chdir(work_dir) != 0) {
    fprintf(stderr, "concorde_shim: chdir to %s failed\n", work_dir);
    close(log_fd);
    close(r->saved_stdout);
    close(r->saved_stderr);
    free(r->saved_cwd);
    return -1;
  }

  dup2(log_fd, STDOUT_FILENO);
  dup2(log_fd, STDERR_FILENO);
  close(log_fd);
  return 0;
}

static void RedirectEnd(Redirect* r) {
  fflush(stdout);
  fflush(stderr);
  dup2(r->saved_stdout, STDOUT_FILENO);
  dup2(r->saved_stderr, STDERR_FILENO);
  close(r->saved_stdout);
  close(r->saved_stderr);
  if (chdir(r->saved_cwd) != 0) {
    fprintf(
        stderr, "concorde_shim: failed to chdir back to %s\n", r->saved_cwd
    );
  }
  free(r->saved_cwd);
}

// Runs CCtsp_solve_dat on an already-built datagroup with cwd and stdio
// redirected into work_dir. Takes ownership of *dat (frees it).
static int SolveDat(
    int ncount,
    CCdatagroup* dat,
    const int* in_tour,
    const double* upper_bound,
    int seed,
    const char* work_dir,
    const char* log_path,
    int* success,
    int* found_tour,
    double* optval,
    int* out_tour
) {
  Redirect redirect;
  if (RedirectBegin(work_dir, log_path, &redirect) != 0) {
    CCutil_freedatagroup(dat);
    return -1;
  }

  CCrandstate rstate;
  CCutil_sprand(seed, &rstate);

  double in_val = 0.0;
  double* in_val_ptr = NULL;
  if (upper_bound != NULL) {
    in_val = *upper_bound;
    in_val_ptr = &in_val;
  }

  // Used as the prefix for Concorde's scratch files, relative to work_dir.
  char name[] = "problem";

  // CCtsp_solve_dat only reads in_tour; the signature predates const.
  int rval = CCtsp_solve_dat(
      ncount,
      dat,
      (int*)in_tour,
      out_tour,
      in_val_ptr,
      optval,
      success,
      found_tour,
      name,
      /*timebound=*/NULL,
      /*hit_timebound=*/NULL,
      /*silent=*/0,
      &rstate
  );

  // When CCtsp_init_lp proves the LP infeasible (possible for sparse graphs:
  // no tour exists), CCtsp_solve_dat reports optval = CCtsp_LP_MAXDOUBLE
  // and leaves *found_tour as set by the initial heuristic tour without
  // writing out_tour. Report that as "no tour".
  if (rval == 0 && *found_tour && *optval >= CCtsp_LP_MAXDOUBLE) {
    *found_tour = 0;
  }

  CCutil_freedatagroup(dat);
  RedirectEnd(&redirect);
  return rval;
}

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
) {
  *success = 0;
  *found_tour = 0;
  *optval = 0.0;

  CCdatagroup dat;
  CCutil_init_datagroup(&dat);

  // CCutil_graph2dat_matrix does not modify elist/elen; the signature just
  // predates const.
  int rval = CCutil_graph2dat_matrix(
      ncount, ecount, (int*)elist, (int*)elen, default_len, &dat
  );
  if (rval != 0) {
    fprintf(stderr, "concorde_shim: CCutil_graph2dat_matrix failed\n");
    CCutil_freedatagroup(&dat);
    return rval;
  }

  return SolveDat(
      ncount,
      &dat,
      /*in_tour=*/NULL,
      upper_bound,
      seed,
      work_dir,
      log_path,
      success,
      found_tour,
      optval,
      out_tour
  );
}

int vats5_concorde_solve_sparse(
    int ncount,
    int ecount,
    const int* elist,
    const int* elen,
    int default_len,
    const int* in_tour,
    const double* upper_bound,
    int seed,
    const char* work_dir,
    const char* log_path,
    int* success,
    int* found_tour,
    double* optval,
    int* out_tour
) {
  *success = 0;
  *found_tour = 0;
  *optval = 0.0;

  if (default_len <= 0) {
    // Concorde would pick (max_len + 1) * ncount itself, which overflows or
    // gets capped for large instances; callers must choose it.
    fprintf(stderr, "concorde_shim: default_len must be positive\n");
    return -1;
  }

  CCdatagroup dat;
  CCutil_init_datagroup(&dat);

  int rval = CCutil_graph2dat_sparse(
      ncount, ecount, (int*)elist, (int*)elen, default_len, &dat
  );
  if (rval != 0) {
    fprintf(stderr, "concorde_shim: CCutil_graph2dat_sparse failed\n");
    // CCutil_graph2dat_sparse frees the datagroup on failure.
    return rval;
  }

  return SolveDat(
      ncount,
      &dat,
      in_tour,
      upper_bound,
      seed,
      work_dir,
      log_path,
      success,
      found_tour,
      optval,
      out_tour
  );
}
