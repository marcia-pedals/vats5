// Arrival-partition refinement implemented as STEP-PARTITION REWRITING.
//
// Unlike history_refine (which computes refined edge weights itself), this
// version expresses the refinement purely as a StepPartitionId assignment:
// each stop's scheduled arrival events are partitioned into parts, every
// completed merged step's destination partition is rewritten to its part
// index, and the tarel graph is built through the STANDARD pipeline
// (ComputeTarelIntermediateData + BuildTarelEdgesFromIntermediateData). Any
// weight strengthening the standard builder applies — e.g. PR-118 slack
// forwarding — therefore composes soundly and automatically.
//
// Refinement policy is the validated "final" configuration: flat start (the
// original line partitions are ignored; parts are over each stop's full event
// set), 2-way threshold pairwise splits over (inbound span, outbound span),
// and the non-contiguous joint 2-state search when pairwise is exhausted.
// Split acceptance compares span-based candidate values against the unsplit
// part's span value (not against paid edge weights), so the search heuristic
// stays meaningful when the builder's weights exceed raw spans.
//
// Flex merged paths keep partition -1 and so quarantine themselves in their
// own tarel states, exactly as in the original pipeline.

#include <CLI/CLI.hpp>
#include <algorithm>
#include <chrono>
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

constexpr int kPosInf = INT_MAX / 4;

using Part = std::vector<int>;

struct Leg {
  std::optional<int> flex_duration;
  std::vector<std::pair<int, int>> steps;  // (dep, arr) sorted by dep
};

bool PartContains(const Part& p, int t) {
  auto it = std::lower_bound(p.begin(), p.end(), t);
  return it != p.end() && *it == t;
}

std::optional<int> LatestAtOrBefore(const Part& p, int t) {
  auto it = std::upper_bound(p.begin(), p.end(), t);
  if (it == p.begin()) {
    return std::nullopt;
  }
  return *(it - 1);
}

}  // namespace

int main(int argc, char* argv[]) {
  CLI::App app{"Arrival-partition refinement via step-partition rewriting"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();
  int iters = 150;
  app.add_option("--iters", iters, "Max refinement iterations")
      ->default_val(150);
  bool seed_from_labels = false;
  app.add_flag(
      "--seed-from-labels",
      seed_from_labels,
      "Seed the initial partition from the state's existing partition labels "
      "(e.g. line partitions) instead of starting flat"
  );
  std::string emit_state_path;
  app.add_option(
      "--emit-state",
      emit_state_path,
      "After refinement, write a ProblemState JSON with the refined partition "
      "assignment baked into the minimal graph's step partitions"
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

  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();

  // Scheduled arrival events per STOP (partitions are ours to assign now).
  std::unordered_map<int, std::vector<int>> arrivals;  // stop.v -> times
  for (const auto& [origin_stop, path_groups] : completed.adjacent) {
    for (const auto& group : path_groups) {
      for (const Path& p : group) {
        const Step& s = p.merged_step;
        if (!s.is_flex) {
          arrivals[s.destination.stop.v].push_back(s.destination.time.seconds);
        }
      }
    }
  }
  for (auto& [_, times] : arrivals) {
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());
  }

  // A state from the rewritten pipeline: (stop, part index) for scheduled
  // arrivals, (stop, -1) for flex arrivals.
  auto IsAnyTime = [&](const TarelState& s) {
    return s.partition.v < 0 || s.stop == state.boundary.start ||
           !arrivals.contains(s.stop.v);
  };

  // Parts per stop; a stop absent from the map is unrefined (single part 0
  // covering all its events).
  std::map<int, std::vector<Part>> partitions;
  if (seed_from_labels) {
    // Group each stop's events by their current destination-partition labels.
    // An event occurring under several labels is assigned to the first part
    // that contains it (parts must be disjoint).
    std::map<int, std::map<int, Part>> by_label;
    for (const auto& [origin_stop, path_groups] : completed.adjacent) {
      for (const auto& group : path_groups) {
        for (const Path& p : group) {
          const Step& s = p.merged_step;
          if (!s.is_flex) {
            by_label[s.destination.stop.v][s.destination.partition.v].push_back(
                s.destination.time.seconds
            );
          }
        }
      }
    }
    for (auto& [stop_v, label_map] : by_label) {
      if (label_map.size() < 2) {
        continue;
      }
      std::set<int> assigned;
      std::vector<Part>& ps = partitions[stop_v];
      for (auto& [_, events] : label_map) {
        Part part;
        for (int t : events) {
          if (assigned.insert(t).second) {
            part.push_back(t);
          }
        }
        if (!part.empty()) {
          std::sort(part.begin(), part.end());
          ps.push_back(std::move(part));
        }
      }
      if (ps.size() < 2) {
        partitions.erase(stop_v);
      }
    }
  }
  auto PartOfEvent = [&](StopId stop, int t) -> int {
    auto it = partitions.find(stop.v);
    if (it == partitions.end()) {
      return 0;
    }
    for (size_t pi = 0; pi < it->second.size(); ++pi) {
      if (PartContains(it->second[pi], t)) {
        return static_cast<int>(pi);
      }
    }
    return 0;  // unreachable if partitions cover all events
  };
  auto EventsOfState = [&](const TarelState& s) -> const Part* {
    if (IsAnyTime(s)) {
      return nullptr;
    }
    auto it = partitions.find(s.stop.v);
    if (it == partitions.end()) {
      return &arrivals.at(s.stop.v);
    }
    return &it->second[s.partition.v];
  };

  // Legs by stop pair (partition-agnostic; part filtering is by membership).
  std::map<std::pair<int, int>, Leg> leg_cache;
  auto GetLeg = [&](StopId from, StopId to) -> const Leg& {
    auto key = std::make_pair(from.v, to.v);
    auto it = leg_cache.find(key);
    if (it != leg_cache.end()) {
      return it->second;
    }
    Leg leg;
    for (const Path& p : completed.PathsBetween(from, to)) {
      const Step& s = p.merged_step;
      if (s.is_flex) {
        int d = s.DurationSeconds();
        if (!leg.flex_duration.has_value() || d < *leg.flex_duration) {
          leg.flex_duration = d;
        }
      } else {
        leg.steps.emplace_back(
            s.origin.time.seconds, s.destination.time.seconds
        );
      }
    }
    std::sort(leg.steps.begin(), leg.steps.end());
    return leg_cache.emplace(key, std::move(leg)).first->second;
  };

  auto ApplyParts = [&](StopId stop,
                        const Part& old_part,
                        std::vector<Part> new_parts) -> bool {
    for (Part& p : new_parts) {
      std::sort(p.begin(), p.end());
    }
    std::vector<Part>& ps = partitions[stop.v];
    if (ps.empty()) {
      ps = std::move(new_parts);
      return true;
    }
    for (size_t pi = 0; pi < ps.size(); ++pi) {
      if (!old_part.empty() && ps[pi] == old_part) {
        ps.erase(ps.begin() + pi);
        for (Part& p : new_parts) {
          ps.push_back(std::move(p));
        }
        return true;
      }
    }
    return false;
  };

  int best_realized = INT_MAX;
  int base_lb = -1;
  int lb = -1;

  for (int iter = 0; iter < iters; ++iter) {
    // Rewrite destination partitions per the current parts and build the
    // tarel graph through the standard pipeline.
    std::vector<Step> steps = completed.AllMergedSteps();
    for (Step& s : steps) {
      if (!s.is_flex) {
        s.destination.partition = StepPartitionId{
            PartOfEvent(s.destination.stop, s.destination.time.seconds)
        };
      }
    }
    TarelEdgeIntermediateData data = ComputeTarelIntermediateData(steps);
    std::vector<TarelEdge> edges = BuildTarelEdgesFromIntermediateData(data);
    TarelStateRemapResult remap = RemapTarelStates(edges, state.required);
    TspGraphData graph = MakeTspGraphEdges(remap.edges, state.boundary);
    std::optional<TspTourResult> result;
    auto solve_t0 = std::chrono::steady_clock::now();
    try {
      result = SolveTspAndExtractTour(
          remap.edges, graph, state.boundary, std::nullopt, nullptr, nullptr
      );
    } catch (const std::exception& ex) {
      std::cerr << "iter " << iter << ": TSP failed: " << ex.what()
                << std::endl;
      break;
    }
    if (!result.has_value()) {
      std::cerr << "iter " << iter << ": TSP infeasible" << std::endl;
      break;
    }
    for (TarelEdge& edge : result->tour_edges) {
      edge.origin = remap.mapped_to_original.at(edge.origin);
      edge.destination = remap.mapped_to_original.at(edge.destination);
    }

    int solve_ms =
        static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - solve_t0
        )
                             .count());
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
              << " refined_stops=" << partitions.size()
              << " parts=" << num_parts
              << " vertices=" << graph.state_by_id.size()
              << " solve_ms=" << solve_ms << std::endl;

    if (realized != INT_MAX && lb > realized) {
      std::cout << "WARNING: LB exceeds realized cost of a valid tour — "
                   "validity bug"
                << std::endl;
      break;
    }
    if (realized != INT_MAX && lb >= best_realized) {
      std::cout << "PROVEN: relaxed tour realizes at its relaxed cost"
                << std::endl;
      break;
    }
    if (graph.state_by_id.size() > 2500) {
      std::cout << "BUDGET: vertex limit reached" << std::endl;
      break;
    }

    auto DumpStuckTour = [&]() {
      std::cout << "  stuck tour states:" << std::endl;
      for (const TarelEdge& e : result->tour_edges) {
        std::cout << "    -> (" << e.destination.stop.v << ","
                  << e.destination.partition.v << ") "
                  << state.StopName(e.destination.stop)
                  << (IsAnyTime(e.destination) ? "  [ANY_TIME]" : "")
                  << std::endl;
      }
    };

    // ---- Pairwise 2-way threshold splits over (inbound, outbound) spans ----
    struct PendingSplit {
      StopId stop;
      Part old_part;  // empty = stop was unrefined
      std::vector<Part> new_parts;
    };
    std::vector<PendingSplit> pending;
    for (size_t idx = 0; idx + 1 < result->tour_edges.size(); ++idx) {
      const TarelEdge& e = result->tour_edges[idx];
      const TarelEdge& f = result->tour_edges[idx + 1];
      const TarelState& x = e.origin;
      const TarelState& y = e.destination;
      const TarelState& z = f.destination;
      if (IsAnyTime(y)) {
        continue;
      }
      const Part* ey_ptr = EventsOfState(y);
      if (ey_ptr == nullptr || ey_ptr->size() < 2) {
        continue;
      }
      const Part& ey = *ey_ptr;
      int n = static_cast<int>(ey.size());
      const Leg& in_leg = GetLeg(x.stop, y.stop);
      if (in_leg.flex_duration.has_value()) {
        continue;
      }
      const Leg& out_leg = GetLeg(y.stop, z.stop);
      const Part* x_events = EventsOfState(x);
      const Part* z_events = EventsOfState(z);

      std::vector<int> ivals(n, kPosInf), ovals(n, kPosInf);
      for (const auto& [dep, arr] : in_leg.steps) {
        auto pit = std::lower_bound(ey.begin(), ey.end(), arr);
        if (pit == ey.end() || *pit != arr) {
          continue;
        }
        int ta;
        if (x_events == nullptr) {
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
      for (int k = 0; k < n; ++k) {
        int best = kPosInf;
        if (out_leg.flex_duration.has_value()) {
          best = *out_leg.flex_duration;
        }
        for (const auto& [dep, arr] : out_leg.steps) {
          if (dep >= ey[k] &&
              (z_events == nullptr || PartContains(*z_events, arr))) {
            best = std::min(best, arr - ey[k]);
          }
        }
        ovals[k] = best;
      }
      long long cur_i = kPosInf, cur_o = kPosInf;
      for (int k = 0; k < n; ++k) {
        cur_i = std::min(cur_i, static_cast<long long>(ivals[k]));
        cur_o = std::min(cur_o, static_cast<long long>(ovals[k]));
      }
      if (cur_i >= kPosInf || cur_o >= kPosInf) {
        continue;
      }
      long long current = cur_i + cur_o;

      long long best_gain = current;
      std::vector<int> best_a;
      std::vector<int> i_cands = ivals, o_cands = ovals;
      std::sort(i_cands.begin(), i_cands.end());
      i_cands.erase(std::unique(i_cands.begin(), i_cands.end()), i_cands.end());
      std::sort(o_cands.begin(), o_cands.end());
      o_cands.erase(std::unique(o_cands.begin(), o_cands.end()), o_cands.end());
      i_cands.push_back(kPosInf + 1);
      o_cands.push_back(kPosInf + 1);
      for (int ti : i_cands) {
        for (int to : o_cands) {
          std::vector<int> a;
          long long m1[2] = {kPosInf, kPosInf}, m2[2] = {kPosInf, kPosInf};
          int c1 = 0;
          a.resize(n);
          for (int k = 0; k < n; ++k) {
            a[k] = (ivals[k] < ti || ovals[k] < to) ? 1 : 0;
            c1 += a[k];
            m1[a[k]] = std::min(m1[a[k]], static_cast<long long>(ivals[k]));
            m2[a[k]] = std::min(m2[a[k]], static_cast<long long>(ovals[k]));
          }
          if (c1 == 0 || c1 == n) {
            continue;
          }
          long long value = LLONG_MAX;
          for (int s2 = 0; s2 < 2; ++s2) {
            if (m1[s2] < kPosInf && m2[s2] < kPosInf) {
              value = std::min(value, m1[s2] + m2[s2]);
            }
          }
          if (value != LLONG_MAX && value > best_gain) {
            best_gain = value;
            best_a = a;
          }
        }
      }
      if (best_a.empty()) {
        continue;
      }
      PendingSplit split;
      split.stop = y.stop;
      if (partitions.contains(y.stop.v)) {
        split.old_part = ey;
      }
      split.new_parts.assign(2, Part{});
      for (int k = 0; k < n; ++k) {
        split.new_parts[best_a[k]].push_back(ey[k]);
      }
      pending.push_back(std::move(split));
    }
    int new_splits = 0;
    for (PendingSplit& split : pending) {
      if (ApplyParts(split.stop, split.old_part, std::move(split.new_parts))) {
        new_splits += 1;
      }
    }
    if (new_splits > 0) {
      continue;
    }

    // ---- Non-contiguous joint 2-state search over edge triples ----
    struct JointPick {
      StopId y_stop, z_stop;
      Part y_old, z_old;
      std::vector<Part> y_parts, z_parts;
      long long margin;
    };
    std::optional<JointPick> pick;
    for (size_t idx = 0; idx + 2 < result->tour_edges.size(); ++idx) {
      const TarelEdge& e = result->tour_edges[idx];
      const TarelEdge& f = result->tour_edges[idx + 1];
      const TarelEdge& g = result->tour_edges[idx + 2];
      const TarelState& x = e.origin;
      const TarelState& y = e.destination;
      const TarelState& z = f.destination;
      const TarelState& w = g.destination;
      if (IsAnyTime(y) || IsAnyTime(z)) {
        continue;
      }
      const Part* ey_ptr = EventsOfState(y);
      const Part* ez_ptr = EventsOfState(z);
      if (ey_ptr == nullptr || ez_ptr == nullptr) {
        continue;
      }
      Part ey = *ey_ptr, ez = *ez_ptr;
      int ny = static_cast<int>(ey.size());
      int nz = static_cast<int>(ez.size());
      if (ny < 2 || nz < 2) {
        continue;
      }
      const Leg& in_leg = GetLeg(x.stop, y.stop);
      const Leg& mid_leg = GetLeg(y.stop, z.stop);
      const Leg& out_leg = GetLeg(z.stop, w.stop);
      if (mid_leg.flex_duration.has_value()) {
        continue;
      }
      const Part* x_events = EventsOfState(x);
      const Part* w_events = EventsOfState(w);

      std::vector<int> ivals(ny, kPosInf), ovals(nz, kPosInf);
      if (in_leg.flex_duration.has_value()) {
        std::fill(ivals.begin(), ivals.end(), *in_leg.flex_duration);
      }
      for (const auto& [dep, arr] : in_leg.steps) {
        auto pit = std::lower_bound(ey.begin(), ey.end(), arr);
        if (pit == ey.end() || *pit != arr) {
          continue;
        }
        int ta;
        if (x_events == nullptr) {
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
      for (int k = 0; k < nz; ++k) {
        int best = kPosInf;
        if (out_leg.flex_duration.has_value()) {
          best = *out_leg.flex_duration;
        }
        for (const auto& [dep, arr] : out_leg.steps) {
          if (dep >= ez[k] &&
              (w_events == nullptr || PartContains(*w_events, arr))) {
            best = std::min(best, arr - ez[k]);
          }
        }
        ovals[k] = best;
      }

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
      std::vector<long long> best_src(ny);
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
        qs.push_back(LLONG_MIN / 2);
        return qs;
      };
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
              long long ms1[2] = {kPosInf, kPosInf};
              long long ms2[2] = {kPosInf, kPosInf};
              for (int k = 0; k < n; ++k) {
                ms1[assign[k]] = std::min(ms1[assign[k]], s1[k]);
                ms2[assign[k]] = std::min(ms2[assign[k]], s2[k]);
              }
              long long proxy = LLONG_MAX;
              for (int a2 = 0; a2 < 2; ++a2) {
                if (ms1[a2] < kPosInf && ms2[a2] < kPosInf) {
                  proxy = std::min(proxy, ms1[a2] + ms2[a2]);
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

      auto ExactEval = [&](const std::vector<char>& ay,
                           const std::vector<char>& az) -> long long {
        long long minI[2] = {kPosInf, kPosInf};
        for (int k = 0; k < ny; ++k) {
          minI[static_cast<int>(ay[k])] = std::min(
              minI[static_cast<int>(ay[k])], static_cast<long long>(ivals[k])
          );
        }
        long long minO[2] = {kPosInf, kPosInf};
        for (int k = 0; k < nz; ++k) {
          minO[static_cast<int>(az[k])] = std::min(
              minO[static_cast<int>(az[k])], static_cast<long long>(ovals[k])
          );
        }
        std::vector<std::pair<int, int>> zs[2];
        for (const auto& [dep, arr] : mid) {
          zs[static_cast<int>(az[idx_by_tz.at(arr)])].emplace_back(dep, arr);
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
              m[static_cast<int>(ay[k])][jz] = std::min(
                  m[static_cast<int>(ay[k])][jz],
                  static_cast<long long>(suf[jz][si]) - ey[k]
              );
            }
          }
        }
        long long value = LLONG_MAX;
        for (int a2 = 0; a2 < 2; ++a2) {
          for (int jz = 0; jz < 2; ++jz) {
            if (minI[a2] < kPosInf && m[a2][jz] < kPosInf &&
                minO[jz] < kPosInf) {
              value = std::min(value, minI[a2] + m[a2][jz] + minO[jz]);
            }
          }
        }
        return value;
      };

      // Span-based current value: both sides unsplit.
      long long current =
          ExactEval(std::vector<char>(ny, 0), std::vector<char>(nz, 0));
      if (current == LLONG_MAX) {
        continue;
      }

      std::vector<std::vector<char>> y_cands = GenSide(ny, tyll, ill, hvals);
      std::vector<std::vector<char>> z_cands = GenSide(nz, tzll, gvals, oll);
      for (const auto& ay : y_cands) {
        for (const auto& az : z_cands) {
          long long value = ExactEval(ay, az);
          if (value == LLONG_MAX) {
            continue;
          }
          long long margin = value - current;
          if (margin > 0 && (!pick.has_value() || margin > pick->margin)) {
            JointPick jp;
            jp.y_stop = y.stop;
            jp.z_stop = z.stop;
            if (partitions.contains(y.stop.v)) {
              jp.y_old = ey;
            }
            if (partitions.contains(z.stop.v)) {
              jp.z_old = ez;
            }
            jp.y_parts.assign(2, Part{});
            for (int k = 0; k < ny; ++k) {
              jp.y_parts[static_cast<int>(ay[k])].push_back(ey[k]);
            }
            jp.z_parts.assign(2, Part{});
            for (int k = 0; k < nz; ++k) {
              jp.z_parts[static_cast<int>(az[k])].push_back(ez[k]);
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
    ApplyParts(pick->y_stop, pick->y_old, std::move(pick->y_parts));
    ApplyParts(pick->z_stop, pick->z_old, std::move(pick->z_parts));
    std::cout << "  joint split: margin " << pick->margin << "s" << std::endl;
  }

  std::cout << "\nBase LB:  " << TimeSinceServiceStart{base_lb} << std::endl;
  std::cout << "Final LB: " << TimeSinceServiceStart{lb} << std::endl;
  std::cout << "Best UB:  " << TimeSinceServiceStart{best_realized}
            << std::endl;
  if (best_realized > base_lb && best_realized != INT_MAX) {
    double frac = 100.0 * (lb - base_lb) / (best_realized - base_lb);
    std::cout << "Recovered " << (lb - base_lb) << "s of "
              << (best_realized - base_lb) << "s gap (" << frac << "%)"
              << std::endl;
  }
  int total_parts = 0;
  for (const auto& [_, ps] : partitions) {
    total_parts += static_cast<int>(ps.size());
  }
  std::cout << "Refined stops: " << partitions.size() << " (" << total_parts
            << " parts)" << std::endl;

  if (!emit_state_path.empty()) {
    std::vector<Step> min_steps = state.minimal.AllSteps();
    for (Step& s : min_steps) {
      if (!s.is_flex) {
        s.destination.partition = StepPartitionId{
            PartOfEvent(s.destination.stop, s.destination.time.seconds)
        };
        s.origin.partition = StepPartitionId{0};
      }
    }
    ProblemState out_state = MakeProblemState(
        MakeAdjacencyList(min_steps),
        state.boundary,
        state.required,
        state.stop_infos,
        state.step_partition_names,
        state.original_edges
    );
    std::ofstream os(emit_state_path);
    nlohmann::json oj = out_state;
    os << oj.dump();
    std::cout << "Emitted refined state to " << emit_state_path << std::endl;
  }

  return 0;
}
