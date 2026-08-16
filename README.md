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

**Guess a "covering set" of stations.** Choose a subset of the target stations and solve the problem for the subset. If the solution visits all the target stations, then we are done. Otherwise, add some missing stations to the subset and try again.

**Brute force.** If there are 10 or fewer stations, brute force check all permutations of the stations.

**Branch-and-bound relaxation.** If there are more than 10 stations, relax the problem to vanilla TSP, solve with Concorde, then branch-and-bound.

## Approach details

### Pruning

Pruning speeds up all later operations by eliminating many steps that can't contribute to an optimal solution.

We define a "step" to be: a single segment of a scheduled transit trip, from one station at a specific time, to the next station; or a walking segment from one station at any time to any other station.

If we have a set of shortest paths from all `(station, time)` pairs to all stations, then there exists an optimal solution to the visit-all-stations problem that only uses steps from these shortest paths, because if we have any optimal solution, then we can replace each segment `(s_0, t_0) -> (s_1, t_1)` with one of our shortest paths `(s_0, t_0) -> (s_1, t_1')`, and `t_1' <= t_1` because it's a shortest path.

The main idea for finding these sets of shortest paths is to run a Dijkstra's-like search from every `(station, time)` to every station, where we keep track of the "current time" at each search node rather than the "current weight" that vanilla Dijkstra's uses.

Searching from every second-resolution time is too much work, so there is one important optimization: Instead of stepping forwards by one second after each query, step forwards to the next reachable departure time, which gives the same results with fewer searches. (TODO: Explain about walking, both why this is valid in light of walking and the trick about walking-reachable departures).

### Covering sets

This "covering set" idea reduces the number of vertices we need to consider.

The idea is that many transit systems are "tree-like" in the sense that a route hitting all the terminal stations hits many of the intermediate stations. In fact, some transit systems (e.g. BART without allowing connections on other transit systems), are indeed trees, so routes hitting all of their terminals also hit all their intermediate stations. So we solve the problem of visiting all the terminal stations and check if the solution visits all the stations. If it does, then we are done. If it does not, then we add more station(s) and repeat. The current implementation adds one station at a time -- the station farthest from the optimal route from the previous iteration.

| System | # Stations | # Stations in final covering set (range[^covering-set-range])
| -- | -- | -- |
| BART | 50 | 6 - 9
| VTA Light Rail | 62 | 9 - 12
| Munich U-Bahn | 96 | 21

[^covering-set-range]: Ranges observed over a few instances of the problem with varying dates and varying walking speed parameters.

This reduction is important, because the next component of the solver takes ~10 minutes to solve a problem with ~20 stations, and gets exponentially slower from there. Even BART's 50 stations would be completely infeasible without this reduction.

### Relaxation to vanilla TSP

We can relax "Transit-TSP" to vanilla TSP[^loose-tsp] by building a graph where the vertices are the stations and the edge weight `a -> b` is the shortest possible duration between arriving at `a` and arriving at `b`, over all possible times when we can arrive at `a`. The solution to this relaxation (which we get with a modern TSP solver like [Concorde]) gives us a lower bound on Transit-TSP. We get an upper bound by computing the fastest possible route along the same sequence of stations.

TODO: Add some data about the tightness of the bound, and about some attempted improvements.

[Concorde]: https://www.math.uwaterloo.ca/tsp/concorde.html

[^loose-tsp]: I'm being a bit loose with the term "vanilla TSP". Technically, the pure vanilla TSP that Concorde solves is an *undirected* graph where each vertex must be visited *exactly once*, but the relaxed TSP that I'm describing is *directed* and we visit vertices *any positive number of times*. There is a straightforward known reduction from this to pure vanilla TSP, and the code implements this.

### Branch and bound

The relaxation gives us lower and upper bounds, so we can branch and bound to find an exact solution.

The current implementation's branch constraint is: Pick an edge `a -> b` in the relaxed solution. The actual solution either **does** or **does not** use this edge.

Intuition for why this branch improves the bounds:
* In the branch where the solution **does** use `a -> b`, we combine the `a` and `b` vertices into a single vertex representing `a -> b`, and the edges out of this vertex give slightly more precise bounds on the `a -> b -> x` portion of the trip, because they take into account the actual waiting times at `b`.
* In the branch where the solution **does not** use `a -> b`, we forbid the edge, which increases the lower bound because that edge was used in the relaxation's optimal solution, so forbidding it worsens the relaxation.

TODO: Add some data about how the bound grows.

## Next steps

The tightness of the relaxation bound is not great and the branch doesn't improve it much. It feels like there might be more structure to exploit to improve them.
