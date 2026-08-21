# VATS5 - "Visit All The Stations"

Computes the **exact fastest route** to visit all the stations in a transit system, taking actual schedules into account.

| System | # Stations | # Lines | Other transit?[^other-transit] | Solve time[^solve-time] | |
| --- | --- | --- | --- | --- | --- |
| BART | 50 | 5 | Yes | 30 seconds | [View solutions](https://vats5.vercel.app/solutions/bart-202608)
| VTA Light Rail | 62 | 3 | Yes | 2 minutes | [View solutions](https://vats5.vercel.app/solutions/vtalr-202608)
| Munich U-Bahn | 96 | 8 | No | 30 minutes | [View solutions](https://vats5.vercel.app/solutions/munich-ubahn-202608)
| Munich U-Bahn | 96 | 8 | Yes | Too big | N/A
| NYC Subway | 400 | 30 | No | Too big | N/A

I don't yet know whether there exist algorithms that can solve NYC Subway exactly in a reasonable amount of time!

I haven't tried approximate algorithms yet. Maybe approximate TSP algorithms can be adapted to this problem.

[^other-transit]: Whether other transit is allowed on the route. This makes the problem harder because there are more routes to consider.
[^solve-time]: How long it takes the implementation to solve one instance of this problem on an M5 MacBook Pro, as of 2026-08-16.

## Why?

Fun algorithms. I like transit. See https://www.transitruns.org, https://www.bart.gov/speedrun, https://en.wikipedia.org/wiki/Subway_Challenge.

## Approach outline

**Input.** Read GTFS transit schedules and extract all the trips on a specified date.

**Prune.** There exists an optimal solution using only steps that are on shortest paths between target stations, so we can prune all other steps.

**Lazy constraints.** Choose a subset of the target stations and solve the problem for the subset. If the solution visits all the target stations, then we are done. Otherwise, add some missing stations to the subset and try again.

**Brute force.** If there are 10 or fewer stations, brute force check all permutations of the stations.

**Branch-and-bound relaxation.** If there are more than 10 stations, relax the problem to vanilla TSP, solve with Concorde, then branch-and-bound.

## Approach details

### Pruning

Pruning speeds up all later operations by eliminating many steps that can't contribute to an optimal solution.

We define a "step" to be: a single segment of a scheduled transit trip, from one station at a specific time, to the next station; or a walking segment from one station at any time to any other station.

If we have a set of shortest paths from all `(station, time)` pairs to all stations, then there exists an optimal solution to the visit-all-stations problem that only uses steps from these shortest paths, because if we have any optimal solution, then we can replace each segment `(s_0, t_0) -> (s_1, t_1)` with one of our shortest paths `(s_0, t_0) -> (s_1, t_1')`, and `t_1' <= t_1` because it's a shortest path.

The main idea for finding these sets of shortest paths is to run a Dijkstra's-like search from every `(station, time)` to every station, where we keep track of the "current time" at each search node rather than the "current weight" that vanilla Dijkstra's uses.

Searching from every second-resolution time is too much work, so there is one important optimization: Instead of stepping forwards by one second after each query, step forwards to the next reachable departure time, which gives the same results with fewer searches. (TODO: Explain about walking, both why this is valid in light of walking and the trick about walking-reachable departures).

### Lazy constraints

We don't need to constrain the path to visit all the stations, because the best path visiting a subset might happen to visit all of them! The intuition for why this works is that many systems are "tree-like" in the sense that the best path visiting all the terminal stations also visits many of the intermediate stations.

Steps:
* Find a minimum spanning tree of the weighted graph of stations where the edges are the min travel times between the stations. Set the initial constraint set to be the leaves of the tree.
* Solve the problem (using brute force or the branch and bound described below).
* Greedily insert any intermediate stations into the solution that we can reach without extending the duration of the solution. (This is very important because the solution has no incentive to hit any intermediate stations and it often skips a bunch of intermediate stations and then spends a bunch of time waiting at a station when it could have visited all those stations and still get to the next station before it has to leave).
* If the solution now visits all the stations, then we are done.
* Otherwise, find the station that is farthest from the solution, insert it into the constraint set, and loop back to solve.

This is a huge speedup: the branch and bound solver can't even solve the smallest problem instance (BART, 50 stations) in a reasonable amount of time if we use the full station set. With lazy constraints, BART ends up needing few enough constraints that we can solve it easily using brute force.

| System | # Stations | Size of final constraint set (range[^covering-set-range])
| -- | -- | -- |
| BART | 50 | 6 - 9
| VTA Light Rail | 62 | 9 - 12
| Munich U-Bahn | 96 | 21

[^covering-set-range]: Ranges observed over a few instances of the problem with varying dates and varying walking speed parameters.

### Relaxation to vanilla TSP

We can relax "Transit-TSP" to vanilla TSP[^loose-tsp] by building a graph where the vertices are the stations and the edge weight `a -> b` is the shortest possible duration between arriving at `a` and arriving at `b`, over all possible times when we can arrive at `a`. The solution to this relaxation (which we get with a modern TSP solver like [Concorde]) gives us a lower bound on Transit-TSP. We get an upper bound by computing the fastest schedule-following route along the same sequence of stations.

TODO: There are some implemented improvements that make it a bit tighter. Explain.

TODO: Add some data about the tightness of the bound, and about some other attempted improvements.

[Concorde]: https://www.math.uwaterloo.ca/tsp/concorde.html

[^loose-tsp]: I'm being a bit loose with the term "vanilla TSP". Technically, the pure vanilla TSP that Concorde solves is an *undirected* graph where each vertex must be visited *exactly once*, but the relaxed TSP that I'm describing is *directed* and we visit vertices *any positive number of times*. There is a straightforward known reduction from this to pure vanilla TSP, and the code implements this.

#### Reduction to vanilla TSP??

There is also a reduction to vanilla TSP[^loose-tsp-2]: a graph where the vertices are (station, trip departure time) and the edges are actual scheduled trips and edges representing waiting at stations.

TODO: Build this reduction so I know how big it is and whether exact solvers can solve it or approximate solvers give good solutions.

[^loose-tsp-2]: Again, this is not exactly vanilla TSP. You want to visit each set of vertices representing a station once instead of visiting each vertex once. There is a cool reduction to actual vanilla TSP: the Noon-Bean transformation.

### Branch and bound

The relaxation gives us lower and upper bounds, so we can branch and bound to find an exact solution.

The current implementation's branch constraint is: Pick an edge `a -> b` in the relaxed solution. The actual solution either **does** or **does not** use this edge.

Intuition for why this branch improves the bounds:
* In the branch where the solution **does** use `a -> b`, we combine the `a` and `b` vertices into a single vertex representing `a -> b`, and the edges out of this vertex give slightly more precise bounds on the `a -> b -> x` portion of the trip, because they take into account the actual waiting times at `b`.
* In the branch where the solution **does not** use `a -> b`, we forbid the edge, which increases the lower bound because that edge was used in the relaxation's optimal solution, so forbidding it worsens the relaxation.

TODO: Add some data about how the bound grows.

## Next steps

The tightness of the relaxation bound is not great and the branch doesn't improve it much. It feels like we might be able to do a lot better.
