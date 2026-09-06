# Making Concorde Sparse-Friendly: Implementation Guide

Goal: solve a TSP instance with m ≈ 50n–100n allowed edges (all other edges forbidden) such that Concorde never materializes forbidden-edge weights and never performs an O(n²)-or-worse time operation. All file/line references are to the standard Concorde source tree (e.g. github.com/matthelb/concorde). Everything below was verified directly against that source.

## Background: how Concorde handles edges

Concorde maintains three layers of edges. The **core LP** holds the columns of the current LP relaxation; every simplex resolve in the cutting-plane loop works on it, so it must stay small (a few·n columns). The **full edge list** is the column universe for delayed column generation: the pricing loop (`addbad_variables` in `TSP/tsp_lp.c`) scans it in chunks against the current duals and pulls negative-reduced-cost edges into the core; `CCtsp_exact_price` (`TSP/ex_price.c`) performs the same scan in exact (bigguy) arithmetic to certify lower bounds, which is required before any pruning (`CCtsp_verify_lp_prune`) or edge elimination. When `lp->full_edges_valid` is set, all of this operates only over the supplied full edge list — no complete-graph scan anywhere. Finally, reduced-cost **edge elimination** (`CCtsp_eliminate_variables`) is a one-shot root operation: it runs after the root cutting loop and exact pricing, before branching, and is never repeated at child nodes.

The consequence: if the full edge list is your m allowed edges and `full_edges_valid = 1`, the entire cutting-plane / pricing / elimination / branch-and-bound machinery is already O(m). The dense operations all live in the *setup* phase (initial tour, initial core edge set), and all of them are avoidable.

## The entry point: CC_SPARSE norm via CCtsp_solve_sparse

Use the library, not the `concorde` binary (the binary's input paths want a datagroup/TSPLIB source for edge lengths; there is no clean sparse input mode). The entry point is `CCtsp_solve_sparse(ncount, ecount, elist, elen, in_tour, out_tour, in_val, optval, success, foundtour, name, timebound, hit_timebound, silent, rstate, ...)` in `TSP/tsp_call.c`.

What it does (verified in source):

`CCutil_graph2dat_sparse` (`UTIL/getdata.c`) builds a `CC_SPARSE`-norm datagroup: adjacency-list storage, O(m) memory. `CCutil_dat_edgelen(i,j)` for this norm (`sparse_edgelen` in `UTIL/edgelen.c`) linearly scans `adj[min(i,j)]` and returns `dat->default_len` for absent edges — forbidden weights are computed lazily as a big-M and never stored. With `defaultlen` argument ≤ 0 (which is what `CCtsp_solve_sparse` passes), default_len is set to `(max_edgelen + 1) * ncount`.

`build_extra_edges` (`TSP/tsp_call.c`) returns the entire sparse edge list as the full edge list with `valid = 1`, so `full_edges_valid` is set and pricing/elimination/exact bounds are all restricted to your m edges. Note that `CCtsp_solve_dat`'s flow calls `CCtsp_eliminate_variables(lp, 1, ...)` — i.e. with `eliminate_sparse = 1` — so root elimination also operates on the sparse list.

After the solve, `CCutil_sparse_real_tour` checks the returned tour edge-by-edge against the adjacency structure. If any artificial (default_len) edge appears, it prints "Tour uses artificial edges" and sets `foundtour = 0`. Infeasibility (no Hamilton cycle within the allowed edges) therefore surfaces honestly rather than as a silently absurd tour value.

**Integer overflow caution.** `default_len = (max+1)·n` is computed in `int` arithmetic, and tours can sum several default_len edges. Keep weights small enough that these quantities fit comfortably in 32-bit int. If the instance comes from a Noon–Bean GTSP transformation, which introduces its own big-M shifts, do not stack the two: scale/shift the transformed weights down before handing them to Concorde, and check `n · (max_edgelen + 1)` against INT_MAX explicitly at load time.

## Required change 1: always pass in_tour (and ideally in_val)

In `CCtsp_solve_dat` (`TSP/tsp_call.c`), if `in_tour` is NULL, `find_good_tour` runs. Its two `CCedgegen_edges` calls (`plan.quadnearest = 2` for a candidate set, `plan.tour.greedy = 1` for a starting tour) dispatch on norm type in `EDGEGEN/edgegen.c`: kd-tree code for `CC_KD_NORM_TYPE`, sorted-coordinate pruning for `CC_X_NORM_TYPE`, and brute-force "junk" routines for everything else — including `CC_SPARSE`, which is tagged `CC_JUNK_NORM_TYPE`. `CCedgegen_junk_node_k_nearest` (`EDGEGEN/xnear.c`) loops each node against all n−1 others, and each `edgelen` call is an O(deg) adjacency scan, so the pass is Θ(n² · avg-deg). At deg ≈ 100–200 this is the single worst dense operation in the pipeline.

Fix: pass any valid permutation as `in_tour`, which skips `find_good_tour` entirely. A tour built from problem structure (for Noon–Bean GTSP instances: any cluster ordering threaded through the zero-cost intra-cluster paths) is strictly better, because its length also supplies a meaningful `in_val` upper bound; a junk permutation works but leaves the initial upper bound at big-M levels until Concorde's own x-heuristics find a real tour, weakening cutoffs early on. If you pass a tour whose value you also pass as `in_val`, first verify it uses only real edges (the same check as `CCutil_sparse_real_tour`); a big-M-contaminated bound is technically valid but useless.

## Required change 2: patch build_edges for m > 2n

`build_edges` (`TSP/tsp_call.c`, `CC_SPARSE` branch) constructs the initial core-LP edge set. If `sparse_ecount ≤ 2·ncount` it uses the whole sparse graph; otherwise it runs `plan.nearest = 4` through edgegen — the same Θ(n²·deg) junk-norm brute force as above — and then calls `CCutil_sparse_strip_edges` to remove any artificial edges that the all-pairs scan swept in. At m = 50n–100n you always hit this branch.

Fix (~20 lines): replace the `else` branch with a direct scan of `dat->adj[i]` / `dat->len[i]`: for each node, select its 4 cheapest incident real edges (8 is also fine at these densities), dedupe with the same edge-hash pattern the file already uses, done in O(m). This is output-equivalent to the stock path — same nearest-4 subgraph — and the `sparse_strip_edges` cleanup becomes vacuous because every candidate is real. Since every node has degree ≥ 4, the padding corner case (junk k-nearest filling short lists with default_len edges) cannot arise.

Do **not** instead raise the 2n threshold so the whole 100n-edge graph becomes the core LP. That trades a one-time selection pass for heavier simplex factorizations on every resolve for the entire run. The core LP is meant to stay small; the pricing loop pulls in the rest of your edges on demand.

## Optional: a fully sparse find_good_tour replacement

If a Concorde-quality heuristic tour is wanted rather than a structure-derived one, `find_good_tour` can be reimplemented sparsely with modest effort, because the Lin-Kernighan core (`CClinkern_tour`, `LINKERN/linkern.c`) is already sparse-clean: it operates on the candidate graph passed in (`goodlist`, with per-candidate edge lengths precomputed), and it explicitly downgrades `CC_LK_GEOMETRIC_KICK` to `CC_LK_CLOSE_KICK` for non-KD norms ("Setting kick type to close"), so no kd-tree is built; close/walk kicks are random walks on the candidate lists. Its only dense-graph touches are `edgelen` calls for tentative tour-closing edges, each an O(deg) adjacency scan.

The replacement is: (1) candidate set — the ~10 cheapest incident edges per node, selected from adjacency lists in O(m); do not pass all 100–200 incident edges, since LK scans a node's candidate list at every breadth step and candidate degree ~8–12 is the standard operating point. (2) Starting tour — sort the m edges, build segments greedily under union-find with degree-≤2 caps in O(m log m), and join leftover segment endpoints arbitrarily; arbitrary joins are implicit big-M edges that LK deletes almost immediately (each removal gains ~big-M). (3) Call `CClinkern_tour` directly with these. (4) Verify the result contains no artificial edges before using its value as an upper bound.

## Optional micro-optimization: sparse_edgelen lookup

`sparse_edgelen` does a linear scan of the adjacency list. LK and pricing issue many point queries; at deg 100–200 each query touches ~150 entries. If profiling shows this hot, sort each adjacency list once at load and binary-search — O(log deg) per query, a ten-line change in `UTIL/edgelen.c` plus a sort in `build_sparse_dat` (`UTIL/getdata.c`).

## What needs no changes

The cutting-plane loop, the pricing loop (`addbad_variables`), exact pricing (`CCtsp_exact_price` with `complete_price = 0`), root edge elimination (`CCtsp_eliminate_variables` with `eliminate_sparse = 1`), bound verification at branch-and-bound nodes (`CCtsp_verify_lp_prune`), and the branchers all respect `full_edges_valid` and operate over the m supplied edges. Note that edge elimination happens once, at the root, before branching — child nodes inherit the sparsified edge list and only re-run cutting and exact bound verification, so there is no per-node dense work to worry about.

## Runtime verification checklist

Watch Concorde's output for these signals. "Using junk-norm nearest code" (from `EDGEGEN/xnear.c`) means a brute-force Θ(n²·deg) pass is running — after the two required changes, this string should never appear. "Use entire sparse graph as initial edge set" only appears in the m ≤ 2n branch, so with the patch you should instead see your replacement path taken (add your own log line). "Tour uses artificial edges" at the end means the allowed-edge graph admitted no better tour than the artificial ones found — treat as infeasible/failed, per `foundtour = 0`. Also assert at load time that `(max_edgelen + 1) * ncount` and a few multiples of it fit in int.

## Function reference

| Concern | Function | File |
|---|---|---|
| Sparse entry point | `CCtsp_solve_sparse` | `TSP/tsp_call.c` |
| Sparse datagroup construction | `CCutil_graph2dat_sparse`, `build_sparse_dat` | `UTIL/getdata.c` |
| Lazy edge length + big-M | `sparse_edgelen`, `default_len` | `UTIL/edgelen.c` |
| Initial core edge set (patch here) | `build_edges` | `TSP/tsp_call.c` |
| Full edge list wiring | `build_extra_edges` | `TSP/tsp_call.c` |
| Dense heuristic to bypass | `find_good_tour` | `TSP/tsp_call.c` |
| Brute-force junk nearest (avoid) | `CCedgegen_junk_k_nearest`, `CCedgegen_junk_node_k_nearest` | `EDGEGEN/xnear.c` |
| Sparse-clean LK core | `CClinkern_tour` | `LINKERN/linkern.c` |
| Chunked pricing over full list | `addbad_variables` | `TSP/tsp_lp.c` |
| Exact pricing / bound certification | `CCtsp_exact_price`, `CCtsp_verify_lp_prune` | `TSP/ex_price.c` |
| Root-only edge elimination | `CCtsp_eliminate_variables` | `TSP/tsp_lp.c` |
| Artificial-edge tour check | `CCutil_sparse_real_tour` | `UTIL/getdata.c` |
| Reduced-cost dump (root LP) | `CCtsp_reduced_cost_nearest`, `-z` flag | `TSP/tsp_lp.c`, `TSP/concorde.c` |
