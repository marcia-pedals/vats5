# VATS5 - "Visit All The Stops"

Computes the **exact fastest route** to visit all the stops in a transit system, including **arbitary walking transfers** and **transfers to other systems**.

The current implemented approach can solve **BART (50 stops, 5 lines)** or **VTA Light Rail (60 stops, 3 lines)**, including walking and using other systems, in **~1 minute on an M5 MacBook Pro**.

NYC Subway (~400 stops, ~30 lines) is in progress. I do not yet know if there exist computationally feasible exact algorithms. Maybe I will adapt some approximate TSP approaches to find an approximate solution.

## Why?

Fun algorithms. I like transit. See https://www.transitruns.org, https://www.bart.gov/speedrun.

## Approach outline

**Input.** Read GTFS transit schedules and extract all the trips on a specified date.

**Prune.** There exists an optimal solution using only steps that are on shortest paths between target stops, so we can prune all other steps.

**Guess a "covering set" of stops.** Choose a subset of the target stops and solve the problem for the subset. If the solution visits all the target stops, then we are done. Otherwise, add some missing stops to the subset and try again.

**Brute force.** If there are fewer than 9 stops, brute force check all permutations of the stops.

**Branch-and-bound relaxation.** Otherwise, relax the problem to vanilla TSP, solve with Concorde, then branch-and-bound.

## Approach details

### Pruning

We define a "step" to be: a single segment of a scheduled transit trip, from one stop at a specific time, to the next stop; or a walking segment from one stop at any time to any other stop.

If we find a set of shortest paths from all `(target stop, time)` pairs to all other target stops, then there exists an optimal solution to the visit-all-target-stops problem that only uses steps from these shortest paths, because if we have any optimal solution, then we can replace each segment `(s_0, t_0) -> (s_1, t_1)` with one of our shortest paths `(s_0, t_0) -> (s_1, t_1')`, and `t_1' <= t_1` because it's a shortest path.

This speeds up all our later operations by eliminating a lot of steps that can't contribute to an optimal solution.

The main idea for finding these sets of shortest paths is to run a multi-target Dijkstra's-like search from every `(target stop, time)`, where we keep track of the "current time" at each search node rather than the "current weight" that plain Dijkstra's uses.

Searching from every second-resolution time is too much work, so there is one imporant optimization: Instead of stepping forwards by one second after each query, step forwards to the next reachable departure time, which gives the same results with fewer searches. (TODO: Explain about walking, both why this is valid in light of walking and the trick about walking-reachable departures).

The implementation also has a few straightforward memory layout optimizations and parallelizations of embarassingly-parallel bits.

With all optimizations combined, this prunes the entire Bay Area Regional GTFS for BART or VTA Light Rail problem instances in seconds.

### Covering sets

TODO

### Relaxation to vanilla TSP

TODO

### Branch and bound

TODO

## Next steps

TODO

## Actually running it

There are a lot of different undocumented configuration formats and preprocessing / solving utilities that you need to chain together because I've been actively experimenting. Let me know if you want to actually run this and I can clean them up a bit.

Making a super easy interface, and maybe hosting on the web so that anyone can query it, is _future work_.
