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

  Redirect redirect;
  if (RedirectBegin(work_dir, log_path, &redirect) != 0) {
    return -1;
  }

  CCdatagroup dat;
  CCutil_init_datagroup(&dat);

  CCrandstate rstate;
  CCutil_sprand(seed, &rstate);

  // CCutil_graph2dat_matrix does not modify elist/elen; the signature just
  // predates const.
  int rval = CCutil_graph2dat_matrix(
      ncount, ecount, (int*)elist, (int*)elen, default_len, &dat
  );
  if (rval != 0) {
    fprintf(stderr, "concorde_shim: CCutil_graph2dat_matrix failed\n");
    CCutil_freedatagroup(&dat);
    RedirectEnd(&redirect);
    return rval;
  }

  double in_val = 0.0;
  double* in_val_ptr = NULL;
  if (upper_bound != NULL) {
    in_val = *upper_bound;
    in_val_ptr = &in_val;
  }

  // Used as the prefix for Concorde's scratch files, relative to work_dir.
  char name[] = "problem";

  rval = CCtsp_solve_dat(
      ncount,
      &dat,
      /*in_tour=*/NULL,
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

  CCutil_freedatagroup(&dat);
  RedirectEnd(&redirect);
  return rval;
}

// Absolute cutting-loop tolerances for vats5_concorde_root_lp, in edge weight
// units: a round of cuts must improve the LP by at least kRootLpRoundTol to
// keep going, and a cut family must improve it by at least kRootLpNextTol
// before the loop moves on to the next family.
static const double kRootLpRoundTol = 0.5;
static const double kRootLpNextTol = 5.0;

// Mirrors the static find_good_tour in Concorde's TSP/tsp_call.c: a
// Lin-Kernighan tour used to permute the nodes (so the LP's initial tour is
// 0, 1, ..., ncount-1) and to seed the upper bound.
static int FindGoodTour(
    int ncount,
    CCdatagroup* dat,
    int* tour,
    double* tval,
    int trials,
    CCrandstate* rstate
) {
  int rval = 0;
  CCedgegengroup plan;
  int ecount = 0;
  int* elist = NULL;
  int tcount = 0;
  int* tlist = NULL;
  int* bestcyc = NULL;
  int* cyc = NULL;
  int* tmp;
  double val, bestval;
  int i, kicks, istour;

  bestval = CCtsp_LP_MAXDOUBLE;
  kicks = (ncount > 1000 ? 500 : ncount / 2);

  cyc = CC_SAFE_MALLOC(ncount, int);
  bestcyc = CC_SAFE_MALLOC(ncount, int);
  if (!cyc || !bestcyc) {
    fprintf(stderr, "concorde_shim: out of memory in FindGoodTour\n");
    rval = 1;
    goto CLEANUP;
  }

  CCedgegen_init_edgegengroup(&plan);
  plan.quadnearest = 2;
  rval = CCedgegen_edges(&plan, ncount, dat, NULL, &ecount, &elist, 1, rstate);
  if (rval) {
    fprintf(stderr, "concorde_shim: CCedgegen_edges failed\n");
    goto CLEANUP;
  }
  plan.quadnearest = 0;

  plan.tour.greedy = 1;
  rval = CCedgegen_edges(&plan, ncount, dat, NULL, &tcount, &tlist, 1, rstate);
  if (rval) {
    fprintf(stderr, "concorde_shim: CCedgegen_edges failed\n");
    goto CLEANUP;
  }
  if (tcount != ncount) {
    fprintf(stderr, "concorde_shim: wrong edgeset from CCedgegen_edges\n");
    rval = 1;
    goto CLEANUP;
  }

  rval = CCutil_edge_to_cycle(ncount, tlist, &istour, cyc);
  if (rval) {
    fprintf(stderr, "concorde_shim: CCutil_edge_to_cycle failed\n");
    rval = 1;
    goto CLEANUP;
  }
  if (!istour) {
    fprintf(stderr, "concorde_shim: starting tour has an error\n");
    rval = 1;
    goto CLEANUP;
  }

  rval = CClinkern_tour(
      ncount,
      dat,
      ecount,
      elist,
      ncount,
      kicks,
      cyc,
      bestcyc,
      &bestval,
      0,
      0.0,
      0.0,
      NULL,
      CC_LK_GEOMETRIC_KICK,
      rstate
  );
  if (rval) {
    fprintf(stderr, "concorde_shim: CClinkern_tour failed\n");
    goto CLEANUP;
  }

  for (i = 0; i < trials; i++) {
    rval = CClinkern_tour(
        ncount,
        dat,
        ecount,
        elist,
        ncount,
        kicks,
        NULL,
        cyc,
        &val,
        1,
        0.0,
        0.0,
        NULL,
        CC_LK_GEOMETRIC_KICK,
        rstate
    );
    if (rval) {
      fprintf(stderr, "concorde_shim: CClinkern_tour failed\n");
      goto CLEANUP;
    }
    if (val < bestval) {
      CC_SWAP(cyc, bestcyc, tmp);
      bestval = val;
    }
  }

  if (trials > 0) {
    rval = CClinkern_tour(
        ncount,
        dat,
        ecount,
        elist,
        ncount,
        2 * kicks,
        bestcyc,
        tour,
        tval,
        1,
        0.0,
        0.0,
        NULL,
        CC_LK_GEOMETRIC_KICK,
        rstate
    );
    if (rval) {
      fprintf(stderr, "concorde_shim: CClinkern_tour failed\n");
      goto CLEANUP;
    }
  } else {
    for (i = 0; i < ncount; i++) {
      tour[i] = bestcyc[i];
    }
    *tval = bestval;
  }

CLEANUP:
  CC_IFFREE(cyc, int);
  CC_IFFREE(bestcyc, int);
  CC_IFFREE(elist, int);
  CC_IFFREE(tlist, int);
  return rval;
}

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
) {
  *infeasible = 0;
  *xcount = 0;
  *xlist = NULL;
  *x = NULL;

  Redirect redirect;
  if (RedirectBegin(work_dir, log_path, &redirect) != 0) {
    return -1;
  }

  int rval = 0;
  int i;
  CCdatagroup dat;
  CCrandstate rstate;
  CCtsp_cutselect sel;
  CCedgegengroup plan;
  int* tour = NULL;
  int iecount = 0;
  int* ielist = NULL;
  int* ielen = NULL;
  int lpecount = 0;
  int* lpelist = NULL;
  double* lpx = NULL;
  CCtsp_lpcuts* pool = NULL;
  CCtsp_lpcuts* dominopool = NULL;
  CCtsp_lp* lp = NULL;
  double tourval = 0.0;
  double upperbound;
  int is_infeasible = 0;
  CCbigguy bound;
  int nonzero = 0;
  int k;
  // Used as the prefix for Concorde's scratch files, relative to work_dir.
  char name[] = "problem";

  CCutil_init_datagroup(&dat);
  CCutil_sprand(seed, &rstate);
  CCtsp_init_cutselect(&sel);

  rval = CCutil_graph2dat_matrix(
      ncount, ecount, (int*)elist, (int*)elen, default_len, &dat
  );
  if (rval) {
    fprintf(stderr, "concorde_shim: CCutil_graph2dat_matrix failed\n");
    goto CLEANUP;
  }

  tour = CC_SAFE_MALLOC(ncount, int);
  if (!tour) {
    fprintf(stderr, "concorde_shim: out of memory\n");
    rval = 1;
    goto CLEANUP;
  }
  rval = FindGoodTour(ncount, &dat, tour, &tourval, 1, &rstate);
  if (rval) {
    fprintf(stderr, "concorde_shim: FindGoodTour failed\n");
    goto CLEANUP;
  }
  rval = CCutil_datagroup_perm(ncount, &dat, tour);
  if (rval) {
    fprintf(stderr, "concorde_shim: CCutil_datagroup_perm failed\n");
    goto CLEANUP;
  }

  // After the permutation, 0, 1, ..., ncount-1 is the LK tour; its length is
  // the initial upper bound (as in CCtsp_solve_dat's perm_bound).
  upperbound = CCutil_dat_edgelen(ncount - 1, 0, &dat);
  for (i = 1; i < ncount; i++) {
    upperbound += CCutil_dat_edgelen(i - 1, i, &dat);
  }
  printf("Set initial upperbound to %.0f (from tour)\n", upperbound);
  fflush(stdout);

  // Initial LP edge set, as in CCtsp_solve_dat's build_edges for a matrix
  // norm.
  CCedgegen_init_edgegengroup(&plan);
  plan.linkern.count = 10;
  plan.linkern.quadnearest = 2;
  plan.linkern.greedy_start = 0;
  plan.linkern.nkicks = (ncount / 100) + 1;
  rval =
      CCedgegen_edges(&plan, ncount, &dat, NULL, &iecount, &ielist, 1, &rstate);
  if (rval) {
    fprintf(stderr, "concorde_shim: CCedgegen_edges failed\n");
    goto CLEANUP;
  }
  ielen = CC_SAFE_MALLOC(iecount, int);
  if (!ielen) {
    fprintf(stderr, "concorde_shim: out of memory\n");
    rval = 1;
    goto CLEANUP;
  }
  for (i = 0; i < iecount; i++) {
    ielen[i] = CCutil_dat_edgelen(ielist[2 * i], ielist[2 * i + 1], &dat);
  }

  rval = CCtsp_init_cutpool(&ncount, NULL, &pool);
  if (rval) {
    fprintf(stderr, "concorde_shim: CCtsp_init_cutpool failed\n");
    goto CLEANUP;
  }
  rval = CCtsp_init_cutpool(&ncount, NULL, &dominopool);
  if (rval) {
    fprintf(stderr, "concorde_shim: CCtsp_init_cutpool failed for dominos\n");
    goto CLEANUP;
  }

  rval = CCtsp_init_lp(
      &lp,
      name,
      -1,
      NULL,
      ncount,
      &dat,
      iecount,
      ielist,
      ielen,
      0,
      NULL,
      NULL,
      0,
      tour,
      upperbound,
      pool,
      dominopool,
      /*silent=*/0,
      &rstate
  );
  if (rval == 2) {
    printf("CCtsp_init_lp reports an infeasible LP\n");
    fflush(stdout);
    rval = CCtsp_verify_infeasible_lp(lp, &is_infeasible, 0);
    if (rval) {
      fprintf(stderr, "concorde_shim: CCtsp_verify_infeasible_lp failed\n");
      goto CLEANUP;
    }
    if (!is_infeasible) {
      fprintf(stderr, "concorde_shim: couldn't verify infeasible LP\n");
      rval = 1;
      goto CLEANUP;
    }
    *infeasible = 1;
    goto CLEANUP;
  } else if (rval) {
    fprintf(stderr, "concorde_shim: CCtsp_init_lp failed\n");
    goto CLEANUP;
  }

  rval = CCtsp_cutselect_set_tols(&sel, lp, 1, 0);
  if (rval) {
    fprintf(stderr, "concorde_shim: CCtsp_cutselect_set_tols failed\n");
    goto CLEANUP;
  }
  // Concorde sets the tolerances relative to the gap between its heuristic
  // tour and the initial degree LP, computed once before any cuts. On our
  // doubled graphs that initial LP is far below any tour (the cheap
  // within-stop cycle edges dominate it), so the relative tolerances come out
  // at tens of seconds per round and the loop stops with cuts still
  // violated. Absolute tolerances make it run until cuts stop paying.
  sel.roundtol = kRootLpRoundTol;
  sel.nexttol = kRootLpNextTol;
  printf(
      "Overriding tolerances: next cuts %.4f next round %.4f\n",
      sel.nexttol,
      sel.roundtol
  );
  fflush(stdout);

  rval = CCtsp_cutting_loop(lp, &sel, 1, 0, &rstate);
  if (rval == 2) {
    printf("CCtsp_cutting_loop reports an infeasible LP\n");
    fflush(stdout);
    rval = CCtsp_verify_infeasible_lp(lp, &is_infeasible, 0);
    if (rval) {
      fprintf(stderr, "concorde_shim: CCtsp_verify_infeasible_lp failed\n");
      goto CLEANUP;
    }
    if (!is_infeasible) {
      fprintf(stderr, "concorde_shim: couldn't verify infeasible LP\n");
      rval = 1;
      goto CLEANUP;
    }
    *infeasible = 1;
    goto CLEANUP;
  } else if (rval) {
    fprintf(stderr, "concorde_shim: CCtsp_cutting_loop failed\n");
    goto CLEANUP;
  }

  *lp_bound = lp->lowerbound;

  rval = CCtsp_exact_price(lp, &bound, 0, 0, 0);
  if (rval) {
    fprintf(stderr, "concorde_shim: CCtsp_exact_price failed\n");
    goto CLEANUP;
  }
  *exact_bound = CCbigguy_bigguytod(bound);
  printf("Root LP bound: %f, exact lower bound: %f\n", *lp_bound, *exact_bound);
  fflush(stdout);

  rval = CCtsp_get_lp_result(
      lp, NULL, NULL, &lpecount, &lpelist, &lpx, NULL, NULL, NULL
  );
  if (rval) {
    fprintf(stderr, "concorde_shim: CCtsp_get_lp_result failed\n");
    goto CLEANUP;
  }
  for (i = 0; i < lpecount; i++) {
    if (lpx[i] > CCtsp_INTTOL) {
      nonzero++;
    }
  }
  *xlist = (int*)malloc(sizeof(int) * (size_t)(2 * nonzero + 1));
  *x = (double*)malloc(sizeof(double) * (size_t)(nonzero + 1));
  if (!*xlist || !*x) {
    fprintf(stderr, "concorde_shim: out of memory for support\n");
    rval = 1;
    goto CLEANUP;
  }
  // The LP numbers nodes in permuted order; lp->perm maps back to the
  // caller's numbering (as CCtsp_dump_x does).
  k = 0;
  for (i = 0; i < lpecount; i++) {
    if (lpx[i] > CCtsp_INTTOL) {
      (*xlist)[2 * k] = lp->perm[lpelist[2 * i]];
      (*xlist)[2 * k + 1] = lp->perm[lpelist[2 * i + 1]];
      (*x)[k] = lpx[i];
      k++;
    }
  }
  *xcount = nonzero;

CLEANUP:
  if (rval != 0) {
    free(*xlist);
    free(*x);
    *xlist = NULL;
    *x = NULL;
    *xcount = 0;
  }
  CC_IFFREE(lpelist, int);
  CC_IFFREE(lpx, double);
  CCtsp_free_tsp_lp_struct(&lp);
  if (pool) {
    CCtsp_free_cutpool(&pool);
  }
  if (dominopool) {
    CCtsp_free_cutpool(&dominopool);
  }
  CCutil_freedatagroup(&dat);
  CC_IFFREE(tour, int);
  CC_IFFREE(ielist, int);
  CC_IFFREE(ielen, int);
  RedirectEnd(&redirect);
  return rval;
}
