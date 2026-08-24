// Arrival-partition refinement of the tarel relaxation (event-indexed DDD).
//
// The tarel LB's residual slack lives in the correlation between arrival
// events: the event at (B, pB) that minimizes the inbound span differs from
// the one minimizing the outbound span, and the flat relaxation gets to use
// both. Refinement: partition a state's arrival events into classes (any
// partition is valid, not just time windows). Every inbound edge is priced
// per target class (min span over its events) and every outbound edge per
// source class, so a tour must commit to a class and pay correlated costs.
// Weights are exact event-level minima, refinement only raises weights, and
// the LB is monotone and always valid.
//
// Split search per consecutive tour edges X -> Y -> Z: with per-event inbound
// spans i(t) and outbound spans o(t), sweep threshold pairs (ti, to) for the
// 2-partition A = {i < ti or o < to} / B = rest (which covers the optimal
// 2-partition's structure), plus phase-class candidates (t mod P) for
// periodic timetables. Accept whichever beats the pair's paid weight.
// Terminate on proof (relaxed tour realizes at its relaxed cost), no
// improving split on the optimal tour (genuinely stuck: the LB is the
// optimal tour's cost, so only raising that tour's cost can raise it), or
// --iters.

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

constexpr int kPartPartitionBase = 100000;
constexpr int kPosInf = INT_MAX / 4;

// A part is a sorted list of arrival times. nullptr in APIs below means "all
// events / unconstrained".
using Part = std::vector<int>;

struct Leg {
  std::optional<int> flex_duration;
  // Scheduled (dep, arr), sorted by dep.
  std::vector<std::pair<int, int>> steps;
};

struct StateKeyLess {
  bool operator()(const TarelState& a, const TarelState& b) const {
    return std::make_pair(a.stop.v, a.partition.v) <
           std::make_pair(b.stop.v, b.partition.v);
  }
};

bool PartContains(const Part& p, int t) {
  auto it = std::lower_bound(p.begin(), p.end(), t);
  return it != p.end() && *it == t;
}

// Latest element of p at or before t; nullopt if none.
std::optional<int> LatestAtOrBefore(const Part& p, int t) {
  auto it = std::upper_bound(p.begin(), p.end(), t);
  if (it == p.begin()) {
    return std::nullopt;
  }
  return *(it - 1);
}

}  // namespace

int main(int argc, char* argv[]) {
  CLI::App app{"Arrival-partition refinement of the tarel relaxation"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();
  int iters = 60;
  app.add_option("--iters", iters, "Max refinement iterations")
      ->default_val(60);
  bool joint = false;
  app.add_flag(
      "--joint",
      joint,
      "Enable the joint 2-state split search when pairwise splits are "
      "exhausted (off by default; candidate for bigger instances)"
  );
  bool flat_partitions = false;
  app.add_flag(
      "--flat-partitions",
      flat_partitions,
      "Collapse all step partitions to a single one before solving, to test "
      "whether event-partition refinement subsumes the line partitioning"
  );
  bool two_way = false;
  app.add_flag(
      "--two-way",
      two_way,
      "Restrict splits to 2-way threshold partitions (no phase classes or "
      "singletons), for controlled vertex growth"
  );

  CLI11_PARSE(app, argc, argv);

  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << std::endl;
    return 1;
  }
  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState state = j.get<ProblemState>();
  in.close();

  if (flat_partitions) {
    std::vector<Step> steps = state.minimal.AllSteps();
    for (Step& s : steps) {
      s.origin.partition = StepPartitionId{0};
      s.destination.partition = StepPartitionId{0};
    }
    state = MakeProblemState(
        MakeAdjacencyList(steps),
        state.boundary,
        state.required,
        state.stop_infos,
        state.step_partition_names,
        state.original_edges
    );
  }

  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();
  std::vector<TarelEdge> base_edges = MakeTarelEdges(completed);

  std::map<TarelState, std::vector<std::pair<TarelState, int>>, StateKeyLess>
      base_out;
  for (const TarelEdge& e : base_edges) {
    base_out[e.origin].push_back({e.destination, e.weight});
  }

  // Arrival events per original state; states with flex arrivals (or the
  // boundary start) admit any arrival time and cannot be partitioned.
  std::unordered_map<TarelState, std::vector<int>> arrivals;
  std::unordered_set<TarelState> any_time;
  for (const auto& [origin_stop, path_groups] : completed.adjacent) {
    for (const auto& group : path_groups) {
      for (const Path& p : group) {
        const Step& s = p.merged_step;
        TarelState dest{s.destination.stop, s.destination.partition};
        if (s.is_flex) {
          any_time.insert(dest);
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
  auto IsAnyTime = [&](const TarelState& s) {
    return s.stop == state.boundary.start || any_time.contains(s) ||
           !arrivals.contains(s);
  };

  std::map<std::pair<int, std::pair<int, int>>, Leg> leg_cache;
  auto GetLeg = [&](StopId from, const TarelState& dest) -> const Leg& {
    auto key =
        std::make_pair(from.v, std::make_pair(dest.stop.v, dest.partition.v));
    auto it = leg_cache.find(key);
    if (it != leg_cache.end()) {
      return it->second;
    }
    Leg leg;
    for (const Path& p : completed.PathsBetween(from, dest.stop)) {
      const Step& s = p.merged_step;
      if (s.is_flex) {
        int d = s.DurationSeconds();
        if (!leg.flex_duration.has_value() || d < *leg.flex_duration) {
          leg.flex_duration = d;
        }
      } else if (s.destination.partition == dest.partition) {
        leg.steps.emplace_back(
            s.origin.time.seconds, s.destination.time.seconds
        );
      }
    }
    std::sort(leg.steps.begin(), leg.steps.end());
    return leg_cache.emplace(key, std::move(leg)).first->second;
  };

  // Partition per refined original state: list of parts covering its events.
  std::map<TarelState, std::vector<Part>, StateKeyLess> partitions;

  // Build the refined edge list. For each base edge, a parts(O) x parts(D)
  // weight matrix is filled in one pass over the leg's steps, so heavily
  // refined states stay affordable.
  auto BuildEdges =
      [&](std::
              map<TarelState, std::pair<TarelState, const Part*>, StateKeyLess>&
                  synth_map) -> std::vector<TarelEdge> {
    synth_map.clear();
    struct Expansion {
      std::vector<const Part*> parts;
      std::vector<TarelState> synths;
      std::unordered_map<int, int> part_of_event;
    };
    std::map<TarelState, Expansion, StateKeyLess> exps;
    int next_id = 0;
    for (const auto& [s, parts] : partitions) {
      Expansion& ex = exps[s];
      for (const Part& p : parts) {
        TarelState synth{s.stop, StepPartitionId{kPartPartitionBase + next_id}};
        next_id += 1;
        int pi = static_cast<int>(ex.parts.size());
        ex.parts.push_back(&p);
        ex.synths.push_back(synth);
        for (int t : p) {
          ex.part_of_event[t] = pi;
        }
        synth_map[synth] = {s, &p};
      }
    }
    std::vector<TarelEdge> edges;
    std::vector<int> w;  // reused matrix buffer
    for (const auto& [o, outs] : base_out) {
      auto o_it = exps.find(o);
      const Expansion* o_ex = (o_it != exps.end()) ? &o_it->second : nullptr;
      bool o_any = IsAnyTime(o);
      const Part* o_all =
          (!o_any && o_ex == nullptr) ? &arrivals.at(o) : nullptr;
      int no = o_ex ? static_cast<int>(o_ex->parts.size()) : 1;
      for (const auto& [d, w_base] : outs) {
        auto d_it = exps.find(d);
        const Expansion* d_ex = (d_it != exps.end()) ? &d_it->second : nullptr;
        if (o_ex == nullptr && d_ex == nullptr) {
          edges.push_back(TarelEdge{o, d, w_base});
          continue;
        }
        int nd = d_ex ? static_cast<int>(d_ex->parts.size()) : 1;
        const Leg& leg = GetLeg(o.stop, d);
        w.assign(static_cast<size_t>(no) * nd, kPosInf);
        if (leg.flex_duration.has_value()) {
          // Flex legs land at any time; conservative for any partitioning.
          for (int k = 0; k < no * nd; ++k) {
            w[k] = *leg.flex_duration;
          }
        }
        for (const auto& [dep, arr] : leg.steps) {
          int dpi = 0;
          if (d_ex != nullptr) {
            auto pit = d_ex->part_of_event.find(arr);
            if (pit == d_ex->part_of_event.end()) {
              continue;
            }
            dpi = pit->second;
          }
          for (int opi = 0; opi < no; ++opi) {
            int ta;
            if (o_any) {
              ta = dep;
            } else {
              const Part& op = o_ex ? *o_ex->parts[opi] : *o_all;
              std::optional<int> t = LatestAtOrBefore(op, dep);
              if (!t.has_value()) {
                continue;
              }
              ta = *t;
            }
            int& cell = w[static_cast<size_t>(opi) * nd + dpi];
            cell = std::min(cell, arr - ta);
          }
        }
        for (int opi = 0; opi < no; ++opi) {
          for (int dpi = 0; dpi < nd; ++dpi) {
            int cell = w[static_cast<size_t>(opi) * nd + dpi];
            if (cell >= kPosInf) {
              continue;
            }
            edges.push_back(
                TarelEdge{
                    o_ex ? o_ex->synths[opi] : o,
                    d_ex ? d_ex->synths[dpi] : d,
                    std::max(cell, w_base)
                }
            );
          }
        }
      }
    }
    return edges;
  };

  int best_realized = INT_MAX;
  int base_lb = -1;
  int lb = -1;

  for (int iter = 0; iter < iters; ++iter) {
    std::map<TarelState, std::pair<TarelState, const Part*>, StateKeyLess>
        synth_map;
    std::vector<TarelEdge> edges = BuildEdges(synth_map);
    auto RealOf =
        [&](const TarelState& s) -> std::pair<TarelState, const Part*> {
      auto it = synth_map.find(s);
      if (it != synth_map.end()) {
        return it->second;
      }
      return {s, nullptr};
    };

    TarelStateRemapResult remap = RemapTarelStates(edges, state.required);
    TspGraphData graph = MakeTspGraphEdges(remap.edges, state.boundary);
    std::optional<TspTourResult> result;
    try {
      result = SolveTspAndExtractTour(
          remap.edges, graph, state.boundary, std::nullopt, nullptr, nullptr
      );
    } catch (const std::exception& e) {
      std::cerr << "iter " << iter << ": TSP failed: " << e.what() << std::endl;
      break;
    }
    if (!result.has_value()) {
      std::cerr << "iter " << iter << ": TSP infeasible\n";
      break;
    }
    for (TarelEdge& edge : result->tour_edges) {
      edge.origin = remap.mapped_to_original.at(edge.origin);
      edge.destination = remap.mapped_to_original.at(edge.destination);
    }

    lb = result->optimal_value;
    if (iter == 0) {
      base_lb = lb;
    }

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
    best_realized = std::min(best_realized, realized);

    int num_parts = 0;
    for (const auto& [_, ps] : partitions) {
      num_parts += static_cast<int>(ps.size());
    }
    std::cout << "iter " << iter << ": lb=" << TimeSinceServiceStart{lb}
              << " realized="
              << (realized == INT_MAX
                      ? std::string("inf")
                      : TimeSinceServiceStart{realized}.ToString())
              << " refined_states=" << partitions.size()
              << " parts=" << num_parts
              << " vertices=" << graph.state_by_id.size() << std::endl;

    if (realized != INT_MAX && lb > realized) {
      std::cout << "WARNING: LB exceeds realized cost of a valid tour — "
                   "validity bug\n";
      break;
    }
    if (realized != INT_MAX && lb >= best_realized) {
      std::cout << "PROVEN: relaxed tour realizes at its relaxed cost\n";
      break;
    }
    if (graph.state_by_id.size() > 2500) {
      std::cout << "BUDGET: vertex limit reached\n";
      break;
    }

    // Refinement over consecutive tour-edge pairs. Split applications are
    // deferred until after the scan: applying one mutates `partitions` and
    // would invalidate the Part pointers other pairs still hold.
    struct PendingSplit {
      TarelState state;
      Part old_part;  // empty = state was unpartitioned
      std::vector<Part> new_parts;
    };
    std::vector<PendingSplit> pending;
    int new_splits = 0;
    for (size_t idx = 0; idx + 1 < result->tour_edges.size(); ++idx) {
      const TarelEdge& e = result->tour_edges[idx];
      const TarelEdge& f = result->tour_edges[idx + 1];
      auto [x_real, x_part] = RealOf(e.origin);
      auto [y_real, y_part] = RealOf(e.destination);
      auto [z_real, z_part] = RealOf(f.destination);
      if (IsAnyTime(y_real)) {
        continue;
      }
      const Part& events = (y_part != nullptr) ? *y_part : arrivals.at(y_real);
      int n = static_cast<int>(events.size());
      if (n < 2) {
        continue;
      }
      const Leg& in_leg = GetLeg(x_real.stop, y_real);
      if (in_leg.flex_duration.has_value()) {
        continue;  // flex inbound hits any event; no correlation to exploit
      }
      const Leg& out_leg = GetLeg(y_real.stop, z_real);
      bool x_any = IsAnyTime(x_real);
      const Part* x_events =
          x_any ? nullptr : (x_part != nullptr ? x_part : &arrivals.at(x_real));

      // Per-event inbound span i(t) and outbound span o(t).
      std::unordered_map<int, int> in_span;
      for (const auto& [dep, arr] : in_leg.steps) {
        if (!PartContains(events, arr)) {
          continue;
        }
        int ta;
        if (x_any) {
          ta = dep;
        } else {
          std::optional<int> t = LatestAtOrBefore(*x_events, dep);
          if (!t.has_value()) {
            continue;
          }
          ta = *t;
        }
        auto [mit, ins] = in_span.try_emplace(arr, arr - ta);
        if (!ins) {
          mit->second = std::min(mit->second, arr - ta);
        }
      }
      std::vector<int> ivals(n), ovals(n);
      for (int k = 0; k < n; ++k) {
        auto it2 = in_span.find(events[k]);
        ivals[k] = (it2 == in_span.end()) ? kPosInf : it2->second;
        int best = kPosInf;
        if (out_leg.flex_duration.has_value()) {
          best = *out_leg.flex_duration;
        }
        for (const auto& [dep, arr] : out_leg.steps) {
          if (dep >= events[k] &&
              (z_part == nullptr || PartContains(*z_part, arr))) {
            best = std::min(best, arr - events[k]);
          }
        }
        ovals[k] = best;
      }

      auto PartsCost =
          [&](const std::vector<std::vector<int>>& groups) -> long long {
        long long worst = LLONG_MAX;
        for (const auto& g : groups) {
          if (g.empty()) {
            return -1;  // invalid candidate
          }
          int mi = kPosInf, mo = kPosInf;
          for (int k : g) {
            mi = std::min(mi, ivals[k]);
            mo = std::min(mo, ovals[k]);
          }
          worst = std::min(worst, static_cast<long long>(mi) + mo);
        }
        return worst;
      };

      long long paid = static_cast<long long>(e.weight) + f.weight;
      long long best_gain = paid;
      std::vector<std::vector<int>> best_groups;

      // Candidate family 1: threshold 2-partitions A = {i < ti or o < to}.
      std::vector<int> i_cands = ivals, o_cands = ovals;
      std::sort(i_cands.begin(), i_cands.end());
      i_cands.erase(std::unique(i_cands.begin(), i_cands.end()), i_cands.end());
      std::sort(o_cands.begin(), o_cands.end());
      o_cands.erase(std::unique(o_cands.begin(), o_cands.end()), o_cands.end());
      i_cands.push_back(kPosInf + 1);
      o_cands.push_back(kPosInf + 1);
      for (int ti : i_cands) {
        for (int to : o_cands) {
          std::vector<int> a, b;
          for (int k = 0; k < n; ++k) {
            if (ivals[k] < ti || ovals[k] < to) {
              a.push_back(k);
            } else {
              b.push_back(k);
            }
          }
          if (a.empty() || b.empty()) {
            continue;
          }
          long long g = PartsCost({a, b});
          if (g > best_gain) {
            best_gain = g;
            best_groups = {a, b};
          }
        }
      }

      // Candidate family 2: phase classes t mod P.
      for (int period : two_way ? std::initializer_list<int>{}
                                : std::initializer_list<int>{600, 1200}) {
        std::map<int, std::vector<int>> classes;
        for (int k = 0; k < n; ++k) {
          classes[((events[k] % period) + period) % period].push_back(k);
        }
        if (classes.size() < 2) {
          continue;
        }
        std::vector<std::vector<int>> groups;
        for (auto& [_, g] : classes) {
          groups.push_back(std::move(g));
        }
        long long g = PartsCost(groups);
        if (g > best_gain) {
          best_gain = g;
          best_groups = std::move(groups);
        }
      }

      // Candidate family 3: singletons — achieves the exact pairwise chain
      // minimum, which structured 2-parts can undershoot. Used only when
      // strictly better (it costs |part| vertices instead of 2).
      if (!two_way && n <= 300) {
        std::vector<std::vector<int>> groups;
        for (int k = 0; k < n; ++k) {
          groups.push_back({k});
        }
        long long g = PartsCost(groups);
        if (g > best_gain) {
          best_gain = g;
          best_groups = std::move(groups);
        }
      }

      if (best_groups.empty()) {
        continue;
      }
      PendingSplit split;
      split.state = y_real;
      if (y_part != nullptr) {
        split.old_part = *y_part;
      }
      for (const auto& g : best_groups) {
        Part p;
        for (int k : g) {
          p.push_back(events[k]);
        }
        std::sort(p.begin(), p.end());
        split.new_parts.push_back(std::move(p));
      }
      pending.push_back(std::move(split));
    }
    for (PendingSplit& split : pending) {
      std::vector<Part>& ps = partitions[split.state];
      if (ps.empty()) {
        ps = std::move(split.new_parts);
        new_splits += 1;
        continue;
      }
      for (size_t pi = 0; pi < ps.size(); ++pi) {
        if (!split.old_part.empty() && ps[pi] == split.old_part) {
          ps.erase(ps.begin() + pi);
          for (Part& p : split.new_parts) {
            ps.push_back(std::move(p));
          }
          new_splits += 1;
          break;
        }
      }
    }
    // Diagnostic for the stuck tour: which states does it enter, and does it
    // route through unrefinable any_time (flex-arrival) states — the "escape
    // hatch" no split can chase?
    auto DumpStuckTour = [&]() {
      std::cout << "  stuck tour states:" << std::endl;
      for (const TarelEdge& e : result->tour_edges) {
        auto [real, part] = RealOf(e.destination);
        std::cout << "    -> (" << real.stop.v << "," << real.partition.v
                  << ") " << state.StopName(real.stop)
                  << (IsAnyTime(real) ? "  [ANY_TIME]" : "")
                  << (part != nullptr ? "  [refined part]" : "") << std::endl;
      }
    };

    if (new_splits == 0 && !joint) {
      std::cout << "STUCK: no pairwise split on the optimal tour" << std::endl;
      DumpStuckTour();
      break;
    }
    if (new_splits == 0) {
      // Joint 2-state search: pairwise correlation is exhausted, so look for
      // slack spanning three consecutive edges e,f,g through middle states Y
      // and Z. Sweep boundary pairs (thetaY over Y's events, thetaZ over Z's);
      // a candidate's value is the min over the four quadrant combinations of
      // in-edge + mid-edge + out-edge minima (infeasible quadrants excluded).
      // Apply the best triple whose value beats the paid three-edge weight.
      struct JointPick {
        TarelState y_state, z_state;
        Part y_old, z_old;
        std::vector<Part> y_parts, z_parts;
        long long margin;
      };
      std::optional<JointPick> pick;
      for (size_t idx = 0; idx + 2 < result->tour_edges.size(); ++idx) {
        const TarelEdge& e = result->tour_edges[idx];
        const TarelEdge& f = result->tour_edges[idx + 1];
        const TarelEdge& g = result->tour_edges[idx + 2];
        auto [x_real, x_part] = RealOf(e.origin);
        auto [y_real, y_part] = RealOf(e.destination);
        auto [z_real, z_part] = RealOf(f.destination);
        auto [w_real, w_part] = RealOf(g.destination);
        if (IsAnyTime(y_real) || IsAnyTime(z_real)) {
          continue;
        }
        Part ey = (y_part != nullptr) ? *y_part : arrivals.at(y_real);
        Part ez = (z_part != nullptr) ? *z_part : arrivals.at(z_real);
        int ny = static_cast<int>(ey.size());
        int nz = static_cast<int>(ez.size());
        if (ny < 2 || nz < 2) {
          continue;
        }
        const Leg& in_leg = GetLeg(x_real.stop, y_real);
        const Leg& mid_leg = GetLeg(y_real.stop, z_real);
        const Leg& out_leg = GetLeg(z_real.stop, w_real);
        if (mid_leg.flex_duration.has_value()) {
          continue;  // would imply Z is any_time; guarded above anyway
        }
        bool x_any = IsAnyTime(x_real);
        const Part* x_events =
            x_any ? nullptr
                  : (x_part != nullptr ? x_part : &arrivals.at(x_real));

        // Inbound spans i over ey.
        std::vector<int> ivals(ny, kPosInf);
        if (in_leg.flex_duration.has_value()) {
          std::fill(ivals.begin(), ivals.end(), *in_leg.flex_duration);
        }
        for (const auto& [dep, arr] : in_leg.steps) {
          auto pit = std::lower_bound(ey.begin(), ey.end(), arr);
          if (pit == ey.end() || *pit != arr) {
            continue;
          }
          int ta;
          if (x_any) {
            ta = dep;
          } else {
            std::optional<int> t = LatestAtOrBefore(*x_events, dep);
            if (!t.has_value()) {
              continue;
            }
            ta = *t;
          }
          int k = static_cast<int>(pit - ey.begin());
          ivals[k] = std::min(ivals[k], arr - ta);
        }
        // Outbound spans o over ez.
        std::vector<int> ovals(nz, kPosInf);
        for (int k = 0; k < nz; ++k) {
          int best = kPosInf;
          if (out_leg.flex_duration.has_value()) {
            best = *out_leg.flex_duration;
          }
          for (const auto& [dep, arr] : out_leg.steps) {
            if (dep >= ez[k] &&
                (w_part == nullptr || PartContains(*w_part, arr))) {
              best = std::min(best, arr - ez[k]);
            }
          }
          ovals[k] = best;
        }
        long long paid = static_cast<long long>(e.weight) + f.weight + g.weight;

        // Mid steps restricted to arrivals in ez (dep-sorted).
        std::vector<std::pair<int, int>> mid;
        for (const auto& [dep, arr] : mid_leg.steps) {
          if (PartContains(ez, arr)) {
            mid.emplace_back(dep, arr);
          }
        }
        if (mid.empty()) {
          continue;
        }
        std::unordered_map<int, int> idx_by_tz;
        for (int k = 0; k < nz; ++k) {
          idx_by_tz[ez[k]] = k;
        }

        // Chain-through score h(tB): min over feasible continuations of
        // mid span + outbound at the landing event.
        std::vector<long long> chain_suffix(mid.size() + 1, LLONG_MAX / 4);
        for (int k = static_cast<int>(mid.size()) - 1; k >= 0; --k) {
          long long v = LLONG_MAX / 4;
          int ov = ovals[idx_by_tz.at(mid[k].second)];
          if (ov < kPosInf) {
            v = static_cast<long long>(mid[k].second) + ov;
          }
          chain_suffix[k] = std::min(chain_suffix[k + 1], v);
        }
        std::vector<long long> hvals(ny, kPosInf);
        for (int k = 0; k < ny; ++k) {
          auto sit = std::lower_bound(
              mid.begin(), mid.end(), std::make_pair(ey[k], INT_MIN)
          );
          long long v = chain_suffix[sit - mid.begin()];
          if (v < LLONG_MAX / 8) {
            hvals[k] = v - ey[k];
          }
        }
        // Chain-into score g(tC): min over feasible mid steps landing at tC
        // of inbound at the source event + mid span.
        std::vector<long long> best_src(ny);  // running max of tB - i(tB)
        for (int k = 0; k < ny; ++k) {
          long long v = LLONG_MIN / 4;
          if (ivals[k] < kPosInf) {
            v = static_cast<long long>(ey[k]) - ivals[k];
          }
          best_src[k] = std::max(k ? best_src[k - 1] : LLONG_MIN / 4, v);
        }
        std::vector<long long> gvals(nz, kPosInf);
        for (const auto& [dep, arr] : mid) {
          auto pit = std::upper_bound(ey.begin(), ey.end(), dep);
          if (pit == ey.begin()) {
            continue;
          }
          long long src = best_src[pit - ey.begin() - 1];
          if (src <= LLONG_MIN / 8) {
            continue;
          }
          int zi = idx_by_tz.at(arr);
          gvals[zi] = std::min(gvals[zi], static_cast<long long>(arr) - src);
        }

        // Candidate 2-partitions per side from threshold families over score
        // pairs. Sentinel thresholds yield single-dimension splits, so pure
        // time boundaries (the old contiguous sweep) are included.
        auto Quantiles = [](std::vector<long long> vals) {
          std::sort(vals.begin(), vals.end());
          vals.erase(std::unique(vals.begin(), vals.end()), vals.end());
          std::vector<long long> qs;
          constexpr int kMaxQ = 16;
          if (static_cast<int>(vals.size()) <= kMaxQ) {
            qs = vals;
          } else {
            for (int q = 0; q < kMaxQ; ++q) {
              qs.push_back(vals[vals.size() * q / kMaxQ]);
            }
          }
          qs.push_back(LLONG_MIN / 2);  // sentinel: predicate never fires
          return qs;
        };
        // Generate top-scoring candidate assignments for one side.
        auto GenSide = [&](
                           int n,
                           const std::vector<long long>& t,
                           const std::vector<long long>& s1,
                           const std::vector<long long>& s2
                       ) -> std::vector<std::vector<char>> {
          std::vector<long long> p6(n), p12(n);
          for (int k = 0; k < n; ++k) {
            p6[k] = ((t[k] % 600) + 600) % 600;
            p12[k] = ((t[k] % 1200) + 1200) % 1200;
          }
          std::vector<std::pair<
              const std::vector<long long>*,
              const std::vector<long long>*>>
              families = {
                  {&s1, &s2}, {&s1, &t}, {&s2, &t}, {&p6, &p6}, {&p12, &p12}
              };
          std::vector<std::pair<long long, std::vector<char>>> scored;
          std::set<std::vector<char>> seen;
          for (const auto& [fa, fb] : families) {
            for (long long ta : Quantiles(*fa)) {
              for (long long tb : Quantiles(*fb)) {
                std::vector<char> assign(n);
                int c1 = 0;
                for (int k = 0; k < n; ++k) {
                  assign[k] = ((*fa)[k] < ta || (*fb)[k] < tb) ? 1 : 0;
                  c1 += assign[k];
                }
                if (c1 == 0 || c1 == n || !seen.insert(assign).second) {
                  continue;
                }
                // Proxy score: min over parts of finite (min s1 + min s2).
                long long ms1[2] = {kPosInf, kPosInf};
                long long ms2[2] = {kPosInf, kPosInf};
                for (int k = 0; k < n; ++k) {
                  ms1[assign[k]] = std::min(ms1[assign[k]], s1[k]);
                  ms2[assign[k]] = std::min(ms2[assign[k]], s2[k]);
                }
                long long proxy = LLONG_MAX;
                for (int a = 0; a < 2; ++a) {
                  if (ms1[a] < kPosInf && ms2[a] < kPosInf) {
                    proxy = std::min(proxy, ms1[a] + ms2[a]);
                  }
                }
                if (proxy == LLONG_MAX) {
                  continue;
                }
                scored.push_back({proxy, std::move(assign)});
              }
            }
          }
          std::sort(
              scored.begin(), scored.end(), [](const auto& a, const auto& b) {
                return a.first > b.first;
              }
          );
          std::vector<std::vector<char>> out;
          for (auto& [_, assign] : scored) {
            out.push_back(std::move(assign));
            if (out.size() >= 8) {
              break;
            }
          }
          return out;
        };
        std::vector<long long> tyll(ey.begin(), ey.end());
        std::vector<long long> tzll(ez.begin(), ez.end());
        std::vector<long long> ill(ivals.begin(), ivals.end());
        std::vector<long long> oll(ovals.begin(), ovals.end());
        std::vector<std::vector<char>> y_cands = GenSide(ny, tyll, ill, hvals);
        std::vector<std::vector<char>> z_cands = GenSide(nz, tzll, gvals, oll);

        // Exact quadrant evaluation of candidate products.
        for (const auto& ay : y_cands) {
          for (const auto& az : z_cands) {
            long long minI[2] = {kPosInf, kPosInf};
            for (int k = 0; k < ny; ++k) {
              minI[ay[k]] =
                  std::min(minI[ay[k]], static_cast<long long>(ivals[k]));
            }
            long long minO[2] = {kPosInf, kPosInf};
            for (int k = 0; k < nz; ++k) {
              minO[az[k]] =
                  std::min(minO[az[k]], static_cast<long long>(ovals[k]));
            }
            std::vector<std::pair<int, int>> zs[2];
            for (const auto& [dep, arr] : mid) {
              zs[az[idx_by_tz.at(arr)]].emplace_back(dep, arr);
            }
            std::vector<int> suf[2];
            for (int jz = 0; jz < 2; ++jz) {
              suf[jz].assign(zs[jz].size() + 1, kPosInf);
              for (int k = static_cast<int>(zs[jz].size()) - 1; k >= 0; --k) {
                suf[jz][k] = std::min(suf[jz][k + 1], zs[jz][k].second);
              }
            }
            long long m[2][2] = {{kPosInf, kPosInf}, {kPosInf, kPosInf}};
            for (int k = 0; k < ny; ++k) {
              for (int jz = 0; jz < 2; ++jz) {
                auto sit = std::lower_bound(
                    zs[jz].begin(), zs[jz].end(), std::make_pair(ey[k], INT_MIN)
                );
                int si = static_cast<int>(sit - zs[jz].begin());
                if (suf[jz][si] < kPosInf) {
                  m[ay[k]][jz] = std::min(
                      m[ay[k]][jz], static_cast<long long>(suf[jz][si]) - ey[k]
                  );
                }
              }
            }
            long long value = LLONG_MAX;
            for (int a = 0; a < 2; ++a) {
              for (int jz = 0; jz < 2; ++jz) {
                if (minI[a] < kPosInf && m[a][jz] < kPosInf &&
                    minO[jz] < kPosInf) {
                  value = std::min(value, minI[a] + m[a][jz] + minO[jz]);
                }
              }
            }
            if (value == LLONG_MAX) {
              continue;
            }
            long long margin = value - paid;
            if (margin > 0 && (!pick.has_value() || margin > pick->margin)) {
              JointPick jp;
              jp.y_state = y_real;
              jp.z_state = z_real;
              if (y_part != nullptr) {
                jp.y_old = *y_part;
              }
              if (z_part != nullptr) {
                jp.z_old = *z_part;
              }
              jp.y_parts.assign(2, Part{});
              for (int k = 0; k < ny; ++k) {
                jp.y_parts[ay[k]].push_back(ey[k]);
              }
              jp.z_parts.assign(2, Part{});
              for (int k = 0; k < nz; ++k) {
                jp.z_parts[az[k]].push_back(ez[k]);
              }
              jp.margin = margin;
              pick = std::move(jp);
            }
          }
        }
      }

      if (!pick.has_value()) {
        std::cout << "STUCK: no pairwise or joint split on the optimal tour"
                  << std::endl;
        DumpStuckTour();
        break;
      }
      auto ApplyParts = [&](const TarelState& s,
                            const Part& old_part,
                            std::vector<Part> new_parts) {
        for (Part& p : new_parts) {
          std::sort(p.begin(), p.end());
        }
        std::vector<Part>& ps = partitions[s];
        if (ps.empty()) {
          ps = std::move(new_parts);
          return;
        }
        for (size_t pi = 0; pi < ps.size(); ++pi) {
          if (!old_part.empty() && ps[pi] == old_part) {
            ps.erase(ps.begin() + pi);
            for (Part& p : new_parts) {
              ps.push_back(std::move(p));
            }
            return;
          }
        }
      };
      ApplyParts(pick->y_state, pick->y_old, std::move(pick->y_parts));
      ApplyParts(pick->z_state, pick->z_old, std::move(pick->z_parts));
      std::cout << "  joint split: margin " << pick->margin << "s" << std::endl;
    }
  }

  std::cout << "\nBase LB:  " << TimeSinceServiceStart{base_lb} << std::endl;
  std::cout << "Final LB: " << TimeSinceServiceStart{lb} << std::endl;
  std::cout << "Best UB:  " << TimeSinceServiceStart{best_realized}
            << std::endl;
  if (best_realized > base_lb && best_realized != INT_MAX) {
    double frac = 100.0 * (lb - base_lb) / (best_realized - base_lb);
    std::cout << "Recovered " << (lb - base_lb) << "s of "
              << (best_realized - base_lb) << "s gap (" << frac << "%)\n";
  }
  int total_parts = 0;
  for (const auto& [_, ps] : partitions) {
    total_parts += static_cast<int>(ps.size());
  }
  std::cout << "Refined states: " << partitions.size() << " (" << total_parts
            << " parts)\n";

  return 0;
}
