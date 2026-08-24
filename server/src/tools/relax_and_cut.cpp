// Relax-and-cut experiment: dualize pairwise chain-slack cuts into the tarel
// TSP edge weights and measure how much of the LB->optimum gap subgradient
// ascent recovers at the root, keeping Concorde's subproblem a vanilla TSP.
//
// Chain cut: for consecutive tarel tour edges e = (A,pA)->(B,pB) and
// f = (B,pB)->(C,pC), any realizable tour using both pays at least
// w_e + w_f + s for the A-arrival -> C-arrival span, where s is the
// event-level chained minimum minus w_e + w_f. The inequality
//   true_cost >= c'x + q * (x_e + x_f - 1)
// is edge-linear, so q folds into the two edges' weights plus a constant.
// Adjacent chains share a tour segment, so validity requires the multiplier
// coefficients of the two chains at a shared edge to sum to <= 1; we enforce
// the uniform cap q <= s/2.

#include <CLI/CLI.hpp>
#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"

using namespace vats5;

namespace {

// Scheduled (dep, arr) options plus optional flex duration for a leg,
// filtered to a destination arrival partition.
struct FilteredLeg {
  std::optional<int> flex_duration;
  std::vector<int> dep_times;
  std::vector<int> arr_min_suffix;

  std::optional<int> EarliestArrivalFrom(int t) const {
    std::optional<int> best;
    if (flex_duration.has_value()) {
      best = t + *flex_duration;
    }
    auto it = std::lower_bound(dep_times.begin(), dep_times.end(), t);
    if (it != dep_times.end()) {
      int arr = arr_min_suffix[it - dep_times.begin()];
      if (!best.has_value() || arr < *best) {
        best = arr;
      }
    }
    return best;
  }
};

FilteredLeg MakeFilteredLeg(
    std::span<const Path> paths, StepPartitionId dest_partition
) {
  FilteredLeg leg;
  std::vector<std::pair<int, int>> dep_arr;
  for (const Path& p : paths) {
    const Step& s = p.merged_step;
    if (s.is_flex) {
      // Flex legs are partitionless from the relaxation's point of view;
      // include them unconditionally so the chain minimum stays a lower bound.
      int d = s.DurationSeconds();
      if (!leg.flex_duration.has_value() || d < *leg.flex_duration) {
        leg.flex_duration = d;
      }
    } else if (s.destination.partition == dest_partition) {
      dep_arr.emplace_back(s.origin.time.seconds, s.destination.time.seconds);
    }
  }
  std::sort(dep_arr.begin(), dep_arr.end());
  leg.dep_times.reserve(dep_arr.size());
  leg.arr_min_suffix.assign(dep_arr.size(), 0);
  for (const auto& [d, a] : dep_arr) {
    leg.dep_times.push_back(d);
  }
  int min_arr = INT_MAX;
  for (int i = static_cast<int>(dep_arr.size()) - 1; i >= 0; --i) {
    min_arr = std::min(min_arr, dep_arr[i].second);
    leg.arr_min_suffix[i] = min_arr;
  }
  return leg;
}

struct ChainKey {
  TarelState a, b, c;
  bool operator<(const ChainKey& o) const {
    auto t = [](const TarelState& s) {
      return std::make_pair(s.stop.v, s.partition.v);
    };
    return std::make_tuple(t(a), t(b), t(c)) <
           std::make_tuple(t(o.a), t(o.b), t(o.c));
  }
};

struct Chain {
  int slack;  // s_k
  int lift;   // q_k, in [0, slack / 2]
};

struct EdgeKey {
  TarelState origin, destination;
  bool operator<(const EdgeKey& o) const {
    auto t = [](const TarelState& s) {
      return std::make_pair(s.stop.v, s.partition.v);
    };
    return std::make_pair(t(origin), t(destination)) <
           std::make_pair(t(o.origin), t(o.destination));
  }
  bool operator==(const EdgeKey& o) const {
    return origin == o.origin && destination == o.destination;
  }
};

}  // namespace

int main(int argc, char* argv[]) {
  CLI::App app{"Relax-and-cut chain-slack experiment on the tarel root LB"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();
  int iters = 40;
  app.add_option("--iters", iters, "Subgradient iterations")->default_val(40);

  CLI11_PARSE(app, argc, argv);

  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << "\n";
    return 1;
  }
  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState state = j.get<ProblemState>();
  in.close();

  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();
  std::vector<TarelEdge> base_edges = MakeTarelEdges(completed);

  // Base weights by edge key, for slack computation.
  std::map<EdgeKey, int> base_weight;
  for (const TarelEdge& e : base_edges) {
    base_weight[EdgeKey{e.origin, e.destination}] = e.weight;
  }

  // Arrival events per (stop, partition), from all completed paths. A flex
  // path into a state means "arrivable at any time"; chains anchored there are
  // skipped (slack 0 is the only valid claim).
  std::unordered_map<TarelState, std::vector<int>> arrivals;
  std::unordered_set<TarelState> flex_arrival;
  for (const auto& [origin_stop, path_groups] : completed.adjacent) {
    for (const auto& group : path_groups) {
      for (const Path& p : group) {
        const Step& s = p.merged_step;
        TarelState dest{s.destination.stop, s.destination.partition};
        if (s.is_flex) {
          flex_arrival.insert(dest);
        } else {
          arrivals[dest].push_back(s.destination.time.seconds);
        }
      }
    }
  }
  for (auto& [_, times] : arrivals) {
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());
  }

  std::map<std::pair<EdgeKey, EdgeKey>, FilteredLeg> leg_cache;
  auto GetLeg = [&](StopId from,
                    StopId to,
                    StepPartitionId dest_partition,
                    const EdgeKey& cache_key_a,
                    const EdgeKey& cache_key_b) -> const FilteredLeg& {
    auto key = std::make_pair(cache_key_a, cache_key_b);
    auto it = leg_cache.find(key);
    if (it == leg_cache.end()) {
      it = leg_cache
               .emplace(
                   key,
                   MakeFilteredLeg(
                       completed.PathsBetween(from, to), dest_partition
                   )
               )
               .first;
    }
    return it->second;
  };

  // Slack of chain (A,pA)->(B,pB)->(C,pC): event-level min of
  // (arrival at C) - (arrival at A) minus the two base edge weights.
  auto ComputeSlack = [&](const ChainKey& k) -> int {
    if (k.a.stop == state.boundary.start || flex_arrival.contains(k.a)) {
      return 0;
    }
    auto arr_it = arrivals.find(k.a);
    if (arr_it == arrivals.end() || arr_it->second.empty()) {
      return 0;
    }
    auto wa = base_weight.find(EdgeKey{k.a, k.b});
    auto wb = base_weight.find(EdgeKey{k.b, k.c});
    if (wa == base_weight.end() || wb == base_weight.end()) {
      return 0;
    }
    const FilteredLeg& ab = GetLeg(
        k.a.stop, k.b.stop, k.b.partition, EdgeKey{k.a, k.b}, EdgeKey{k.a, k.b}
    );
    const FilteredLeg& bc = GetLeg(
        k.b.stop, k.c.stop, k.c.partition, EdgeKey{k.b, k.c}, EdgeKey{k.b, k.c}
    );
    int chain_min = INT_MAX;
    for (int ta : arr_it->second) {
      std::optional<int> tb = ab.EarliestArrivalFrom(ta);
      if (!tb.has_value()) {
        continue;
      }
      std::optional<int> tc = bc.EarliestArrivalFrom(*tb);
      if (!tc.has_value()) {
        continue;
      }
      chain_min = std::min(chain_min, *tc - ta);
    }
    if (chain_min == INT_MAX) {
      return 0;
    }
    return std::max(0, chain_min - wa->second - wb->second);
  };

  std::map<ChainKey, Chain> pool;

  int base_lb = -1;
  int best_lb = -1;
  int best_ub = INT_MAX;

  for (int iter = 0; iter < iters; ++iter) {
    // Build lifted edges.
    std::vector<TarelEdge> lifted = base_edges;
    long long total_lift = 0;
    if (!pool.empty()) {
      std::map<EdgeKey, int> lift_by_edge;
      for (const auto& [k, chain] : pool) {
        if (chain.lift > 0) {
          lift_by_edge[EdgeKey{k.a, k.b}] += chain.lift;
          lift_by_edge[EdgeKey{k.b, k.c}] += chain.lift;
          total_lift += chain.lift;
        }
      }
      for (TarelEdge& e : lifted) {
        auto it = lift_by_edge.find(EdgeKey{e.origin, e.destination});
        if (it != lift_by_edge.end()) {
          e.weight += it->second;
        }
      }
    }

    TarelStateRemapResult remap = RemapTarelStates(lifted, state.required);
    TspGraphData graph = MakeTspGraphEdges(remap.edges, state.boundary);
    std::optional<TspTourResult> result;
    try {
      result = SolveTspAndExtractTour(
          remap.edges, graph, state.boundary, std::nullopt, nullptr, nullptr
      );
    } catch (const std::exception& e) {
      std::cerr << "iter " << iter << ": TSP solve failed: " << e.what()
                << "\n";
      continue;
    }
    if (!result.has_value()) {
      std::cerr << "iter " << iter << ": TSP infeasible\n";
      break;
    }

    // Map tour edges back to original states.
    for (TarelEdge& edge : result->tour_edges) {
      edge.origin = remap.mapped_to_original.at(edge.origin);
      edge.destination = remap.mapped_to_original.at(edge.destination);
    }

    int lb = result->optimal_value - static_cast<int>(total_lift);
    if (iter == 0) {
      base_lb = lb;
    }
    best_lb = std::max(best_lb, lb);

    // Realize the tour for an upper bound.
    std::vector<StopId> seq;
    seq.push_back(result->tour_edges[0].origin.stop);
    for (const TarelEdge& e : result->tour_edges) {
      seq.push_back(e.destination.stop);
    }
    std::vector<Path> feasible =
        ComputeMinimalFeasiblePathsAlong(seq, completed);
    int realized = INT_MAX;
    for (const Path& p : feasible) {
      realized = std::min(realized, p.DurationSeconds());
    }
    best_ub = std::min(best_ub, realized);

    // Separation: chains along the current tour.
    std::set<EdgeKey> tour_edge_set;
    for (const TarelEdge& e : result->tour_edges) {
      tour_edge_set.insert(EdgeKey{e.origin, e.destination});
    }
    for (size_t i = 0; i + 1 < result->tour_edges.size(); ++i) {
      const TarelEdge& e = result->tour_edges[i];
      const TarelEdge& f = result->tour_edges[i + 1];
      ChainKey k{e.origin, e.destination, f.destination};
      if (!pool.contains(k)) {
        pool[k] = Chain{ComputeSlack(k), 0};
      }
    }

    // Projected subgradient step with diminishing size. The gradient of
    // LB(q) in q_k is (x_e + x_f - 1) at the minimizing tour: +1 when both
    // chain edges are used, 0 when exactly one is, -1 when neither is.
    int step = std::max(1, 60 / (1 + iter / 8));
    int num_positive = 0;
    for (auto& [k, chain] : pool) {
      if (chain.slack <= 0) {
        continue;
      }
      int present = tour_edge_set.contains(EdgeKey{k.a, k.b}) +
                    tour_edge_set.contains(EdgeKey{k.b, k.c});
      int gradient = present - 1;
      chain.lift = std::clamp(chain.lift + step * gradient, 0, chain.slack / 2);
      if (chain.lift > 0) {
        num_positive += 1;
      }
    }

    std::cout << "iter " << iter << ": lb=" << TimeSinceServiceStart{lb}
              << " realized=" << TimeSinceServiceStart{realized}
              << " chains=" << pool.size() << " lifted=" << num_positive
              << " total_lift=" << total_lift << "\n";
  }

  std::cout << "\nBase LB:  " << TimeSinceServiceStart{base_lb} << "\n";
  std::cout << "Best LB:  " << TimeSinceServiceStart{best_lb} << "\n";
  std::cout << "Best UB:  " << TimeSinceServiceStart{best_ub} << "\n";
  if (best_ub > base_lb) {
    double frac = 100.0 * (best_lb - base_lb) / (best_ub - base_lb);
    std::cout << "Recovered " << (best_lb - base_lb) << "s of "
              << (best_ub - base_lb) << "s gap (" << frac << "%)\n";
  }

  return 0;
}
