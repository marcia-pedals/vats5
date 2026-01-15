# VATS5

## Main Algorithm

Do a branch and bound, using a normal TSP solver as a subcomponent.

Upper and lower bounds. Calculate the pairwise shortest durations (taking actual schedules into
account) between all target stops, use those as weights for a complete graph on all the stops, and
solve TSP on that graph. The cost of the TSP solution is a lower bound on the cost of the actual
solution. The actual fastest time it would take to visit the stations in that order is an upper
bound on the cost of the actual solution.

Branch. Choose any ordered pair of target stations (A, B). The actual solution either does not (left
branch) or does (right branch) have a segment that stops at A then B without stopping at any other
target stations in between.

For the left branch, record A->B as a forbidden segment and disregard all paths that do A->B while
calculating pairwise shortest durations.

For the right branch, collapse A and B into a single stop AB, and combine(2) the steps(1) as follows:

* X->A->B steps are combined to X->AB steps.
* B->X steps become AB->X steps.
* X->A->Y steps are combined to X->Y steps.
* X->B->Y (where Y != A) steps are combined to X->Y steps.
* X->B->A->Y steps are combined to X->Y steps (which can be done by first combining the X->B->A
  steps and then combining those with the A->Y steps).

(1) What's a step? It's one of:

* A scheduled step (origin_stop: StopId, destination_stop: StopId, origin_time: Time,
  destination_time: Time, origin_trip_id: TripId | None, destination_trip_id: TripId | None)
  representing a single step of a scheduled trip. The trip ids are important for calculating
  required connection times between steps: e.g. if you come in and leave a stop on the same trip id,
  you're just sitting in the same vehicle so you don't need a min connection time. For steps that
  come directly from the underlying schedule, origin_trip_id and destination_trip_id are equal and
  defined. When we collapse stops though (see "right branch" above), the collapsed step might start
  (origin_trip_id) on a different trip than it ends up (destination_trip_id), and one or both could
  be None if the collapsed step starts or ends with a walking step.
* A walking step (origin_stop: StopId, destination_stop: StopId, duration: Duration) representing
  walking between stops.

(2) What's it mean to combine X->Y->Z steps? You take the cross product of all the X->Y steps with
all the Y->Z steps, filtered for pairs that you can actually ride based on the schedule and min
connection times, and each one of these pairs becomes a step X->Z. You don't need to actually
process or emit all `O(m*n)` pairs because there are `O(min(m, n))` that dominate in the sense that
for any pair you can find one of the dominating pairs that departs at a later-or-same time and
arrives at an earlier-or-same time. You can find these in `O(min(m, n))` time with the two sliding
pointer approach. (Not completely true that an optimal solution to the overall problem will always
use one of these dominating pairs: e.g. it's possible that it's worth it to arrive at Z later if you
arrive on a different train that requires less min connection time for the onward journey. A bit of
logic in the sliding pointer combiner should be able to detect these and also include them. I'll
figure it out while writing the logic.)

Selecting branch. Ideally we select a branch where the left side (forbid A->B) has a really high
lower bound that lets us quickly disprove it. Chosing an arbitrary segment from the current upper
bound route seems like a good heuristic for this, because the current upper bound route is probably
close to the optimal route and therefore many of its segments are "good ones" that are costly to
avoid.
