// Experimental tool: improve the tarel lower bound with Lagrangian costs.
//
// We keep a cost lambda(stop, arrival_time) for each (stop, scheduled arrival
// time) pair. When computing tarel edge weights, a candidate that arrives at a
// stop at time t pays +lambda(stop, t), and a candidate that departs a stop
// using arrival time t pays -lambda(stop, t). ("Using arrival time t" means
// the tarel weight is measured from arrival time t, so along a feasible tour
// the arrival used by the outgoing edge equals the actual arrival time and the
// two terms cancel.) Therefore every feasible tour has unchanged total cost,
// while the relaxation's cost changes because the min-candidates determining
// each tarel edge weight may shift.
//
// Candidates involving flex arrivals are unmodified because their arrival time
// is unknown: at any tarel state with flex arrivals, neither the incoming
// +lambda nor the outgoing -lambda is applied (this keeps the telescoping
// argument valid). START/END states are fed by flex steps, so they are
// automatically unmodified.
//
// Subgradient optimization: solve the tarel TSP, then for each optimal tour
// edge, raise the cost of the (stop, time) pair its weight-determining
// candidate arrives at and lower the cost of the pair it departs from. When
// the relaxed tour is time-consistent at a stop, the two cancel.

#include <CLI/CLI.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "Highs.h"
#include "solver/held_karp_dp.h"
#include "solver/tarel_graph.h"
#include "solver/tour_paths.h"

using namespace vats5;

namespace {

using CostKey = std::pair<StopId, TimeSinceServiceStart>;
using LagrangianCosts = std::unordered_map<CostKey, double>;

double GetCost(
    const LagrangianCosts& costs, StopId stop, TimeSinceServiceStart time
) {
  auto it = costs.find({stop, time});
  return it == costs.end() ? 0.0 : it->second;
}

// Grid index range [k1, k2) of the coverage price mass a candidate is charged
// for (its "effective span").
struct CoverageSpan {
  int k1 = 0;
  int k2 = 0;
};

// Coverage prices for a subproblem with starts pinned to [t1, t2] and an
// incumbent UB: a real tour's legs (arrival-to-arrival spans) exactly tile
// [start, start + D) for some start in [t1, t2] and duration D in [LB, UB].
// Charging every candidate the pi-mass of its span and subtracting
// MaxAdmissibleSpan() — the maximum pi-sum over any admissible tour span —
// from the bound is valid for ANY signed pi, because the real tour's
// collected charge is one admissible span's sum. Candidates with unknown span
// components are charged a provable minimum (undercharging the relaxation is
// always valid).
class CoveragePrices {
 public:
  // Grid points at t1 + k * res for k in [0, n), spanning [t1, t2 + ub).
  CoveragePrices(int t1, int t2, int ub, int res)
      : grid_start_(t1),
        res_(res),
        n_(std::max(0, (t2 + ub - t1 + res - 1) / res)),
        t1_(t1),
        t2_(t2),
        lb_(0),
        pi_(n_, 0.0) {
    Rebuild();
  }

  int n() const { return n_; }
  int GridTime(int k) const { return grid_start_ + k * res_; }

  // Every tour in the subproblem is known to last at least lb; admissible
  // spans have length >= lb. Only ever grows.
  void SetLb(int lb) { lb_ = std::max(lb_, lb); }
  int lb() const { return lb_; }

  // The maximum pi-sum over admissible spans [start, start + D) with start in
  // [t1, t2] and D >= lb (conservatively allowing D up to the grid end).
  double MaxAdmissibleSpan(CoverageSpan* span) const {
    int a_lo = K(t1_);
    int a_hi = K(t2_);
    int b_min = K(t1_ + lb_);
    // suffix_max_[b] = max over b' >= b of prefix_[b'], with argmax.
    double best = -std::numeric_limits<double>::infinity();
    int best_a = a_lo;
    int best_b = a_lo;
    for (int a = a_lo; a <= a_hi && a <= n_; ++a) {
      int b_lo = std::max(a, b_min);
      double v = suffix_max_[b_lo] - prefix_[a];
      if (v > best) {
        best = v;
        best_a = a;
        best_b = suffix_max_arg_[b_lo];
      }
    }
    if (span != nullptr) {
      *span = {best_a, best_b};
    }
    return best;
  }

  // Number of grid points strictly before time t.
  int K(int t) const {
    if (t <= grid_start_) {
      return 0;
    }
    long k = (static_cast<long>(t) - grid_start_ + res_ - 1) / res_;
    return static_cast<int>(std::min<long>(k, n_));
  }

  // Charge for a fully known span [a, d).
  double ChargeKnown(int a, int d, CoverageSpan* span) const {
    int k1 = K(a);
    int k2 = std::max(K(d), k1);
    if (span != nullptr) {
      *span = {k1, k2};
    }
    return prefix_[k2] - prefix_[k1];
  }

  // Upper bounds on the (unknown) arrival time at flex-arrival states. A
  // state fed only by zero-duration flex steps from START arrives exactly at
  // the tour start, which is pinned to [t1, t2]; other flex states get no
  // bound.
  void SetArrivalUbs(std::unordered_map<TarelState, int> ubs) {
    arrival_ubs_ = std::move(ubs);
  }
  int ArrivalUb(const TarelState& state) const {
    auto it = arrival_ubs_.find(state);
    return it == arrival_ubs_.end() ? std::numeric_limits<int>::max()
                                    : it->second;
  }

  // Scheduled step departing t_o, arriving t_d, taken from a flex-arrival
  // state: the leg's span starts at the unknown arrival in
  // [grid start, min(t_o, arrival_ub)]. Charge the minimum over possible
  // arrivals.
  double ChargeFlexOrigin(
      const TarelState& origin, int t_o, int t_d, CoverageSpan* span
  ) const {
    int ko = K(std::min(t_o, ArrivalUb(origin)));
    int k1 = maxpref_arg_[ko];
    int k2 = std::max(K(t_d), k1);
    if (span != nullptr) {
      *span = {k1, k2};
    }
    return prefix_[k2] - prefix_[k1];
  }

  // Flex step of the given duration taken from a flex-arrival state: an
  // interval of that length starting at the unknown arrival, bounded by
  // arrival_ub. Charge the minimum placement.
  double ChargeFloating(
      const TarelState& origin, int duration, CoverageSpan* span
  ) const {
    int len = std::min(duration / res_, n_);
    if (len <= 0) {
      if (span != nullptr) {
        *span = {0, 0};
      }
      return 0.0;
    }
    int j_hi = std::min(K(ArrivalUb(origin)), n_ - len);
    double best = std::numeric_limits<double>::infinity();
    int best_j = 0;
    for (int j = 0; j <= j_hi; ++j) {
      double v = prefix_[j + len] - prefix_[j];
      if (v < best) {
        best = v;
        best_j = j;
      }
    }
    if (best == std::numeric_limits<double>::infinity()) {
      best = prefix_[n_] - prefix_[std::max(0, n_ - len)];
      best_j = std::max(0, n_ - len);
    }
    if (span != nullptr) {
      *span = {best_j, best_j + len};
    }
    return best;
  }

  // Prefix mass strictly before time t (for folding the destination part of a
  // known-span charge into per-step precomputations).
  double PrefixBefore(int t) const { return prefix_[K(t)]; }

  void Add(const std::vector<double>& step) {
    for (int k = 0; k < n_; ++k) {
      pi_[k] += step[k];
    }
    Rebuild();
  }

 private:
  void Rebuild() {
    prefix_.assign(n_ + 1, 0.0);
    maxpref_.assign(n_ + 1, 0.0);
    maxpref_arg_.assign(n_ + 1, 0);
    for (int k = 0; k < n_; ++k) {
      prefix_[k + 1] = prefix_[k] + pi_[k];
    }
    for (int m = 1; m <= n_; ++m) {
      if (prefix_[m] > maxpref_[m - 1]) {
        maxpref_[m] = prefix_[m];
        maxpref_arg_[m] = m;
      } else {
        maxpref_[m] = maxpref_[m - 1];
        maxpref_arg_[m] = maxpref_arg_[m - 1];
      }
    }
    suffix_max_.assign(n_ + 1, 0.0);
    suffix_max_arg_.assign(n_ + 1, n_);
    suffix_max_[n_] = prefix_[n_];
    suffix_max_arg_[n_] = n_;
    for (int b = n_ - 1; b >= 0; --b) {
      if (prefix_[b] > suffix_max_[b + 1]) {
        suffix_max_[b] = prefix_[b];
        suffix_max_arg_[b] = b;
      } else {
        suffix_max_[b] = suffix_max_[b + 1];
        suffix_max_arg_[b] = suffix_max_arg_[b + 1];
      }
    }
  }

  int grid_start_;
  int res_;
  int n_;
  int t1_;
  int t2_;
  int lb_;
  std::vector<double> pi_;
  std::vector<double> prefix_;
  std::vector<double> maxpref_;
  std::vector<int> maxpref_arg_;
  std::vector<double> suffix_max_;
  std::vector<int> suffix_max_arg_;
  std::unordered_map<TarelState, int> arrival_ubs_;
};

struct EdgeCandidate;

// Sparse "must span" prices for the span branch-and-bound: for each time t in
// the branch's must-span set, the tour spans t exactly once (a contiguous
// tour spans every contained time exactly once), so charging spanning
// candidates +pi_t and subtracting Total() from the bound telescopes for
// every tour in the branch, with pi free-signed. Candidates whose span is
// only partially known are charged a provable minimum.
class SpanPrices {
 public:
  explicit SpanPrices(std::vector<int> times) : times_(std::move(times)) {
    std::sort(times_.begin(), times_.end());
    pi_.assign(times_.size(), 0.0);
    Rebuild();
  }

  int n() const { return static_cast<int>(times_.size()); }
  int time(int i) const { return times_[i]; }
  double pi(int i) const { return pi_[i]; }
  double Total() const { return prefix_[times_.size()]; }

  // Index of the first must-time >= t.
  int Idx(int t) const {
    return static_cast<int>(
        std::lower_bound(times_.begin(), times_.end(), t) - times_.begin()
    );
  }

  // Mass of must-times strictly before t (a known span [a, d) is charged
  // PrefixBefore(d) - PrefixBefore(a)).
  double PrefixBefore(int t) const { return prefix_[Idx(t)]; }

  double ChargeKnown(int a, int d) const {
    return PrefixBefore(d) - PrefixBefore(a);
  }

  // Scheduled step departing t_o, arriving d, from a flex-arrival state: the
  // span starts at the unknown arrival <= t_o; charge the minimum.
  double ChargeFlexOrigin(int t_o, int d) const {
    int jo = Idx(t_o);
    return PrefixBefore(d) - maxpref_[jo];
  }

  // Flex step from a flex-arrival state (span placed anywhere): charge the
  // sum of negative prices (a provable minimum).
  double ChargeFloating(int duration) const {
    return duration <= 0 ? 0.0 : neg_total_;
  }

  // Adds w to g[i] for every must-time i the candidate is counted as
  // spanning under its (min-)charge.
  void CountSpanned(
      const EdgeCandidate& c, double w, std::vector<double>& g
  ) const;

  void Add(const std::vector<double>& step) {
    for (size_t i = 0; i < pi_.size(); ++i) {
      pi_[i] += step[i];
    }
    Rebuild();
  }

 private:
  void Rebuild() {
    size_t m = times_.size();
    prefix_.assign(m + 1, 0.0);
    maxpref_.assign(m + 1, 0.0);
    maxpref_arg_.assign(m + 1, 0);
    neg_total_ = 0.0;
    for (size_t i = 0; i < m; ++i) {
      prefix_[i + 1] = prefix_[i] + pi_[i];
      neg_total_ += std::min(pi_[i], 0.0);
    }
    for (size_t i = 1; i <= m; ++i) {
      if (prefix_[i] > maxpref_[i - 1]) {
        maxpref_[i] = prefix_[i];
        maxpref_arg_[i] = static_cast<int>(i);
      } else {
        maxpref_[i] = maxpref_[i - 1];
        maxpref_arg_[i] = maxpref_arg_[i - 1];
      }
    }
  }

  std::vector<int> times_;
  std::vector<double> pi_;
  std::vector<double> prefix_;
  std::vector<double> maxpref_;
  std::vector<int> maxpref_arg_;
  double neg_total_ = 0.0;
};

// Hard "which stop at which time" constraint: if require, the leg spanning
// `time` must have destination stop `stop` (equivalently: every leg into
// `stop` spans `time`, since the tour visits it once, and no other leg spans
// `time`); if !require, the leg spanning `time` must NOT have destination
// `stop`.
struct SpanStopConstraint {
  int time;
  StopId stop;
  bool require;
};

// All dual prices: the (stop, arrival time) lambdas plus optional coverage
// prices, optional sparse must-span prices, and hard span-stop filters.
struct Prices {
  LagrangianCosts stop_time;
  std::optional<CoveragePrices> coverage;
  std::optional<SpanPrices> spans;
  std::vector<SpanStopConstraint> span_stops;
};

// A tarel edge annotated with the (stop, time) pairs of the candidate that
// determined its weight. `origin_arrival` is the arrival time at origin.stop
// the weight was measured from (nullopt when the origin state has flex
// arrivals). `destination_arrival` is the arrival time at destination.stop of
// the minimizing candidate (nullopt when it is flex or the destination state
// has flex arrivals).
struct AnnotatedTarelEdge {
  TarelEdge edge;
  std::optional<TimeSinceServiceStart> origin_arrival;
  std::optional<TimeSinceServiceStart> destination_arrival;
};

// Lagrangian-cost-aware variant of BuildTarelEdgesFromIntermediateData.
// With empty prices this computes exactly the same weights.
std::vector<AnnotatedTarelEdge> BuildTarelEdgesWithCosts(
    const TarelEdgeIntermediateData& data, const Prices& prices
) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const LagrangianCosts& costs = prices.stop_time;
  const CoveragePrices* cov =
      prices.coverage.has_value() ? &*prices.coverage : nullptr;
  const SpanPrices* spans = prices.spans.has_value() ? &*prices.spans : nullptr;
  std::vector<AnnotatedTarelEdge> edges;
  for (const auto& [origin, arrival_times_to_origin] : data.arrival_times_to) {
    auto it = data.steps_from.find(origin.stop);
    if (it == data.steps_from.end()) {
      continue;
    }
    for (const auto& [dest, steps] : it->second) {
      bool dest_has_flex = data.arrival_times_to.at(dest).has_flex;
      auto dest_cost = [&](const Step& step) -> double {
        return (step.is_flex || dest_has_flex)
                   ? 0.0
                   : GetCost(costs, dest.stop, step.destination.time);
      };

      // Hard span-stop filters applicable to this destination group: a
      // candidate's span must contain every require time and no forbid time.
      std::vector<int> forbid_times;
      std::vector<int> require_times;
      for (const SpanStopConstraint& c : prices.span_stops) {
        if (c.require) {
          (c.stop == dest.stop ? require_times : forbid_times)
              .push_back(c.time);
        } else if (c.stop == dest.stop) {
          forbid_times.push_back(c.time);
        }
      }
      std::sort(forbid_times.begin(), forbid_times.end());
      std::sort(require_times.begin(), require_times.end());
      int min_r = require_times.empty() ? std::numeric_limits<int>::max()
                                        : require_times.front();
      int max_r = require_times.empty() ? std::numeric_limits<int>::min()
                                        : require_times.back();
      {
        bool group_infeasible = false;
        for (int f : forbid_times) {
          if (f >= min_r && f <= max_r) {
            group_infeasible = true;
          }
        }
        if (group_infeasible) {
          continue;
        }
      }

      double best = kInf;
      std::optional<TimeSinceServiceStart> best_origin_arrival;
      std::optional<TimeSinceServiceStart> best_destination_arrival;

      if (arrival_times_to_origin.has_flex) {
        // If the arrival is flex, we have to assume we can arrive any time, so
        // the candidate cost is simply the duration of the step out. No
        // -lambda is applied since the arrival time is unknown; the coverage
        // charge uses the provable-minimum span.
        for (const Step& step : steps) {
          if (!step.is_flex) {
            // Definite span part [dep, arr): apply the hard filters where
            // decidable (the unknown earlier part stays permissive).
            bool blocked = false;
            for (int f : forbid_times) {
              if (step.origin.time.seconds <= f &&
                  f < step.destination.time.seconds) {
                blocked = true;
              }
            }
            for (int r : require_times) {
              if (r >= step.destination.time.seconds) {
                blocked = true;
              }
            }
            if (blocked) {
              continue;
            }
          }
          double v = step.DurationSeconds() + dest_cost(step);
          if (cov != nullptr) {
            v += step.is_flex ? cov->ChargeFloating(
                                    origin, step.FlexDurationSeconds(), nullptr
                                )
                              : cov->ChargeFlexOrigin(
                                    origin,
                                    step.origin.time.seconds,
                                    step.destination.time.seconds,
                                    nullptr
                                );
          }
          if (spans != nullptr) {
            v += step.is_flex
                     ? spans->ChargeFloating(step.FlexDurationSeconds())
                     : spans->ChargeFlexOrigin(
                           step.origin.time.seconds,
                           step.destination.time.seconds
                       );
          }
          if (v < best) {
            best = v;
            best_origin_arrival = std::nullopt;
            best_destination_arrival =
                (step.is_flex || dest_has_flex)
                    ? std::nullopt
                    : std::optional(step.destination.time);
          }
        }
      } else {
        // The arrival is scheduled.
        size_t n = steps.size();
        size_t flex_offset = (n > 0 && steps[0].is_flex) ? 1 : 0;

        // Steps arriving too early to contain all require times are excluded.
        size_t v_lo = flex_offset;
        while (v_lo < n && steps[v_lo].destination.time.seconds <= max_r) {
          v_lo += 1;
        }

        // Suffix minima of (destination time + arrival cost + coverage/span
        // prefix mass before the destination) over the allowed scheduled
        // steps: with costs, the best step departing at-or-after t is no
        // longer necessarily the first one. The charges for span [t, t_d)
        // split into a destination part (in the suffix) and a
        // -PrefixBefore(t) part (per arrival time). The forbid times segment
        // the arrival loop (a candidate may not arrive past the first forbid
        // time at or after its arrival), so the suffix is rebuilt per
        // segment over [v_lo, seg_hi).
        std::vector<double> suffix_val(n + 1, kInf);
        std::vector<size_t> suffix_arg(n + 1, n);
        size_t seg_hi = n + 1;  // sentinel: force the initial build
        auto rebuild_suffix = [&](size_t hi) {
          std::fill(suffix_val.begin(), suffix_val.end(), kInf);
          std::fill(suffix_arg.begin(), suffix_arg.end(), n);
          for (size_t j = hi; j-- > v_lo;) {
            double v = steps[j].destination.time.seconds + dest_cost(steps[j]);
            if (cov != nullptr) {
              v += cov->PrefixBefore(steps[j].destination.time.seconds);
            }
            if (spans != nullptr) {
              v += spans->PrefixBefore(steps[j].destination.time.seconds);
            }
            if (v < suffix_val[j + 1]) {
              suffix_val[j] = v;
              suffix_arg[j] = j;
            } else {
              suffix_val[j] = suffix_val[j + 1];
              suffix_arg[j] = suffix_arg[j + 1];
            }
          }
          seg_hi = hi;
        };

        size_t step_idx = flex_offset;
        size_t fptr = 0;
        for (const TimeSinceServiceStart arrival_time :
             arrival_times_to_origin.times) {
          int t = arrival_time.seconds;
          if (t > min_r) {
            break;  // Later arrivals cannot contain the require times.
          }
          while (fptr < forbid_times.size() && forbid_times[fptr] < t) {
            fptr += 1;
          }
          int f_next = fptr < forbid_times.size()
                           ? forbid_times[fptr]
                           : std::numeric_limits<int>::max();
          size_t hi = n;
          if (f_next != std::numeric_limits<int>::max()) {
            hi = std::partition_point(
                     steps.begin() + flex_offset,
                     steps.end(),
                     [&](const Step& s) {
                       return s.destination.time.seconds <= f_next;
                     }
                 ) -
                 steps.begin();
          }
          if (hi != seg_hi) {
            rebuild_suffix(hi);
          }

          double lambda_out = GetCost(costs, origin.stop, arrival_time);
          if (flex_offset == 1) {
            // A flex step departs exactly at the arrival time, so it uses
            // arrival time t and pays -lambda(origin, t).
            int flex_d = t + steps[0].FlexDurationSeconds();
            bool ok =
                (require_times.empty() || flex_d > max_r) && f_next >= flex_d;
            if (ok) {
              double v = steps[0].FlexDurationSeconds() - lambda_out;
              if (cov != nullptr) {
                v += cov->ChargeKnown(t, flex_d, nullptr);
              }
              if (spans != nullptr) {
                v += spans->ChargeKnown(t, flex_d);
              }
              if (v < best) {
                best = v;
                best_origin_arrival = arrival_time;
                best_destination_arrival = std::nullopt;
              }
            }
          }
          while (step_idx < n && steps[step_idx].origin.time < arrival_time) {
            step_idx += 1;
          }
          size_t lo = std::max(step_idx, v_lo);
          if (lo < hi) {
            double v = suffix_val[lo] - t - lambda_out;
            if (cov != nullptr) {
              v -= cov->PrefixBefore(t);
            }
            if (spans != nullptr) {
              v -= spans->PrefixBefore(t);
            }
            if (v < best) {
              best = v;
              best_origin_arrival = arrival_time;
              const Step& step = steps[suffix_arg[lo]];
              best_destination_arrival =
                  dest_has_flex ? std::nullopt
                                : std::optional(step.destination.time);
            }
          }
          if (step_idx >= n && flex_offset == 0) {
            break;
          }
        }
      }

      if (best < kInf) {
        // Floor keeps the weight a valid lower bound on the real-valued
        // modified weight.
        edges.push_back(
            AnnotatedTarelEdge{
                .edge =
                    TarelEdge{
                        .origin = origin,
                        .destination = dest,
                        .weight = static_cast<int>(std::floor(best)),
                    },
                .origin_arrival = best_origin_arrival,
                .destination_arrival = best_destination_arrival,
            }
        );
      }
    }
  }
  return edges;
}

// One candidate (arrival time at origin, step out) for a tarel edge's weight,
// identified by the lambda keys it touches, plus the coverage-grid span it is
// charged for (empty when coverage prices are off) and its leg-span info in
// seconds for sparse span constraints and bunching profiles.
struct EdgeCandidate {
  std::optional<TimeSinceServiceStart> origin_arrival;
  std::optional<TimeSinceServiceStart> destination_arrival;
  double value;
  CoverageSpan span;
  // kKnown: leg spans [leg_a, leg_d). kFlexOrigin: start unknown (<= leg_a,
  // the departure), end leg_d. kFloating: duration leg_a, placed anywhere.
  enum class LegKind : int8_t { kKnown, kFlexOrigin, kFloating };
  LegKind leg_kind = LegKind::kKnown;
  int leg_a = 0;
  int leg_d = 0;
};

void SpanPrices::CountSpanned(
    const EdgeCandidate& c, double w, std::vector<double>& g
) const {
  switch (c.leg_kind) {
    case EdgeCandidate::LegKind::kKnown: {
      for (int j = Idx(c.leg_a); j < Idx(c.leg_d); ++j) {
        g[j] += w;
      }
      break;
    }
    case EdgeCandidate::LegKind::kFlexOrigin: {
      // Effective span starts at the max-prefix point (the min-charge's
      // choice of arrival).
      int jo = Idx(c.leg_a);
      for (int j = maxpref_arg_[jo]; j < Idx(c.leg_d); ++j) {
        g[j] += w;
      }
      break;
    }
    case EdgeCandidate::LegKind::kFloating: {
      if (c.leg_a > 0) {
        for (int j = 0; j < n(); ++j) {
          if (pi_[j] < 0.0) {
            g[j] += w;
          }
        }
      }
      break;
    }
  }
}

// Enumerates every candidate determining the weight of the tarel edge
// (origin, dest), with its lambda-modified value. Deduplicates candidates
// touching the same lambda keys (keeping the min value). Only used for the
// ~tour-length set of optimal edges per iteration, so the full enumeration
// cost is fine.
std::vector<EdgeCandidate> EnumerateEdgeCandidates(
    const TarelEdgeIntermediateData& data,
    const Prices& prices,
    const TarelState& origin,
    const TarelState& dest
) {
  const LagrangianCosts& costs = prices.stop_time;
  const CoveragePrices* cov =
      prices.coverage.has_value() ? &*prices.coverage : nullptr;
  const SpanPrices* spans = prices.spans.has_value() ? &*prices.spans : nullptr;
  const ArrivalTimes& arrivals = data.arrival_times_to.at(origin);
  const std::vector<Step>& steps = data.steps_from.at(origin.stop).at(dest);
  bool dest_has_flex = data.arrival_times_to.at(dest).has_flex;
  auto dest_cost = [&](const Step& step) -> double {
    return (step.is_flex || dest_has_flex)
               ? 0.0
               : GetCost(costs, dest.stop, step.destination.time);
  };

  // Hard span-stop filters for this destination group (must match the
  // filtering in BuildTarelEdgesWithCosts).
  std::vector<int> forbid_times;
  std::vector<int> require_times;
  for (const SpanStopConstraint& c : prices.span_stops) {
    if (c.require) {
      (c.stop == dest.stop ? require_times : forbid_times).push_back(c.time);
    } else if (c.stop == dest.stop) {
      forbid_times.push_back(c.time);
    }
  }
  int min_r =
      require_times.empty()
          ? std::numeric_limits<int>::max()
          : *std::min_element(require_times.begin(), require_times.end());
  int max_r =
      require_times.empty()
          ? std::numeric_limits<int>::min()
          : *std::max_element(require_times.begin(), require_times.end());
  // A known candidate span [a, d) is allowed iff it contains all require
  // times and no forbid times.
  auto span_allowed = [&](int a, int d) {
    if (!require_times.empty() && (a > min_r || d <= max_r)) {
      return false;
    }
    for (int f : forbid_times) {
      if (a <= f && f < d) {
        return false;
      }
    }
    return true;
  };
  for (int f : forbid_times) {
    if (f >= min_r && f <= max_r) {
      return {};  // Contradictory constraints: no candidates.
    }
  }
  auto dest_arrival =
      [&](const Step& step) -> std::optional<TimeSinceServiceStart> {
    return (step.is_flex || dest_has_flex)
               ? std::nullopt
               : std::optional(step.destination.time);
  };

  std::map<
      std::pair<
          std::optional<TimeSinceServiceStart>,
          std::optional<TimeSinceServiceStart>>,
      EdgeCandidate>
      by_keys;
  auto add = [&](std::optional<TimeSinceServiceStart> origin_arrival,
                 std::optional<TimeSinceServiceStart> destination_arrival,
                 double value,
                 CoverageSpan span,
                 EdgeCandidate::LegKind leg_kind,
                 int leg_a,
                 int leg_d) {
    EdgeCandidate cand{
        origin_arrival, destination_arrival, value, span, leg_kind, leg_a, leg_d
    };
    auto [it, inserted] =
        by_keys.try_emplace({origin_arrival, destination_arrival}, cand);
    if (!inserted && value < it->second.value) {
      it->second = cand;
    }
  };

  if (arrivals.has_flex) {
    for (const Step& step : steps) {
      if (!step.is_flex) {
        bool blocked = false;
        for (int f : forbid_times) {
          if (step.origin.time.seconds <= f &&
              f < step.destination.time.seconds) {
            blocked = true;
          }
        }
        for (int r : require_times) {
          if (r >= step.destination.time.seconds) {
            blocked = true;
          }
        }
        if (blocked) {
          continue;
        }
      }
      double v = step.DurationSeconds() + dest_cost(step);
      CoverageSpan span;
      if (cov != nullptr) {
        v +=
            step.is_flex
                ? cov->ChargeFloating(origin, step.FlexDurationSeconds(), &span)
                : cov->ChargeFlexOrigin(
                      origin,
                      step.origin.time.seconds,
                      step.destination.time.seconds,
                      &span
                  );
      }
      if (spans != nullptr) {
        v += step.is_flex
                 ? spans->ChargeFloating(step.FlexDurationSeconds())
                 : spans->ChargeFlexOrigin(
                       step.origin.time.seconds, step.destination.time.seconds
                   );
      }
      if (step.is_flex) {
        add(std::nullopt,
            dest_arrival(step),
            v,
            span,
            EdgeCandidate::LegKind::kFloating,
            step.FlexDurationSeconds(),
            0);
      } else {
        add(std::nullopt,
            dest_arrival(step),
            v,
            span,
            EdgeCandidate::LegKind::kFlexOrigin,
            step.origin.time.seconds,
            step.destination.time.seconds);
      }
    }
  } else {
    size_t n = steps.size();
    size_t flex_offset = (n > 0 && steps[0].is_flex) ? 1 : 0;
    for (const TimeSinceServiceStart t : arrivals.times) {
      double lambda_out = GetCost(costs, origin.stop, t);
      if (flex_offset == 1 &&
          span_allowed(t.seconds, t.seconds + steps[0].FlexDurationSeconds())) {
        double v = steps[0].FlexDurationSeconds() - lambda_out;
        int leg_d = t.seconds + steps[0].FlexDurationSeconds();
        CoverageSpan span;
        if (cov != nullptr) {
          v += cov->ChargeKnown(t.seconds, leg_d, &span);
        }
        if (spans != nullptr) {
          v += spans->ChargeKnown(t.seconds, leg_d);
        }
        add(t,
            std::nullopt,
            v,
            span,
            EdgeCandidate::LegKind::kKnown,
            t.seconds,
            leg_d);
      }
      for (size_t j = flex_offset; j < n; ++j) {
        if (steps[j].origin.time < t) {
          continue;
        }
        if (!span_allowed(t.seconds, steps[j].destination.time.seconds)) {
          continue;
        }
        double v = steps[j].destination.time.seconds + dest_cost(steps[j]) -
                   t.seconds - lambda_out;
        CoverageSpan span;
        if (cov != nullptr) {
          v += cov->ChargeKnown(
              t.seconds, steps[j].destination.time.seconds, &span
          );
        }
        if (spans != nullptr) {
          v += spans->ChargeKnown(t.seconds, steps[j].destination.time.seconds);
        }
        add(t,
            dest_arrival(steps[j]),
            v,
            span,
            EdgeCandidate::LegKind::kKnown,
            t.seconds,
            steps[j].destination.time.seconds);
      }
    }
  }

  std::vector<EdgeCandidate> out;
  out.reserve(by_keys.size());
  for (const auto& [keys, cand] : by_keys) {
    out.push_back(cand);
  }
  return out;
}

// One iteration's relaxation solve, engine-agnostic: the bound and the set of
// tarel edges "used" by the relaxed solution with their mass (1 for a tour
// edge; the fractional x value for an LP edge). Pointers are into the
// caller's annotated edge vector.
struct IterationResult {
  int lower_bound;
  std::vector<std::pair<const AnnotatedTarelEdge*, double>> active;
  std::string engine_stats;
};

std::optional<IterationResult> SolveIterationConcorde(
    const ProblemState& state,
    const std::vector<AnnotatedTarelEdge>& annotated,
    std::ostream* tsp_log
) {
  std::vector<TarelEdge> edges;
  edges.reserve(annotated.size());
  for (const AnnotatedTarelEdge& a : annotated) {
    edges.push_back(a.edge);
  }

  TarelStateRemapResult remap = RemapTarelStates(edges, state.required);
  TspGraphData graph = MakeTspGraphEdges(remap.edges, state.boundary);

  // Check that at least one representative from each group of required stops
  // appears in `graph` (same check as ComputeTarelLowerBound).
  std::unordered_set<StopId> representatives_in_graph;
  for (const TarelState& tarel_state : graph.state_by_id) {
    representatives_in_graph.insert(
        state.required.Representative(tarel_state.stop)
    );
  }
  for (StopId rep : state.required.GroupRepresentatives()) {
    if (!representatives_in_graph.contains(rep)) {
      return std::nullopt;
    }
  }

  std::optional<TspTourResult> result = SolveTspAndExtractTour(
      remap.edges, graph, state.boundary, std::nullopt, tsp_log, nullptr
  );
  if (!result.has_value()) {
    return std::nullopt;
  }

  // For each merged edge, find the original annotated edge achieving its
  // weight (RemapTarelStates takes the min over merged parallel edges).
  std::map<std::pair<TarelState, TarelState>, const AnnotatedTarelEdge*>
      best_by_merged;
  for (const AnnotatedTarelEdge& a : annotated) {
    auto key = std::pair(
        remap.original_to_mapped.at(a.edge.origin),
        remap.original_to_mapped.at(a.edge.destination)
    );
    auto [it, inserted] = best_by_merged.try_emplace(key, &a);
    if (!inserted && a.edge.weight < it->second->edge.weight) {
      it->second = &a;
    }
  }

  IterationResult out{
      .lower_bound = result->optimal_value,
      .active = {},
      .engine_stats = "tsp_n " + std::to_string(graph.state_by_id.size()),
  };
  for (const TarelEdge& tour_edge : result->tour_edges) {
    out.active.push_back(
        {best_by_merged.at({tour_edge.origin, tour_edge.destination}), 1.0}
    );
  }
  return out;
}

// Maps every state onto its required-group representative stop with dense
// per-stop partition ids. No weight-dependent merging (unlike
// RemapTarelStates' signature merging, which min-merges parallel in-edges and
// measurably weakens the bound): the result stays 1:1 with the input edges.
// Group mapping is required for "visit one member per group" semantics.
// Returns nullopt if a required group has no states.
std::optional<std::vector<TarelEdge>> BuildGroupMappedEdges(
    const std::vector<AnnotatedTarelEdge>& annotated,
    const RequiredStops& required
) {
  std::map<StopId, std::vector<TarelState>> states_by_rep;
  {
    std::set<TarelState> seen;
    for (const AnnotatedTarelEdge& a : annotated) {
      for (const TarelState& s : {a.edge.origin, a.edge.destination}) {
        if (seen.insert(s).second) {
          states_by_rep[required.Representative(s.stop)].push_back(s);
        }
      }
    }
  }
  for (StopId rep : required.GroupRepresentatives()) {
    if (!states_by_rep.contains(rep)) {
      return std::nullopt;  // A required group is unreachable; no bound.
    }
  }
  std::unordered_map<TarelState, TarelState> to_mapped;
  for (auto& [rep, states] : states_by_rep) {
    std::sort(states.begin(), states.end());
    for (int i = 0; i < states.size(); ++i) {
      to_mapped[states[i]] = TarelState{rep, StepPartitionId{i}};
    }
  }
  std::vector<TarelEdge> mapped_edges;
  mapped_edges.reserve(annotated.size());
  for (const AnnotatedTarelEdge& a : annotated) {
    mapped_edges.push_back(
        TarelEdge{
            .origin = to_mapped.at(a.edge.origin),
            .destination = to_mapped.at(a.edge.destination),
            .weight = a.edge.weight,
        }
    );
  }
  return mapped_edges;
}

// Exact TSP bound (Concorde) on the same unmerged group-mapped graph the LP
// relaxation uses. This is the relaxation the LP lower-bounds, so it is the
// right certifier for LP-mode bounds. `tour_annotated` (if non-null) receives
// the optimal tour's edges as indices into `annotated`, in tour order (the
// closing END->START edge is not included).
std::optional<int> SolveUnmergedTspBound(
    const std::vector<AnnotatedTarelEdge>& annotated,
    const RequiredStops& required,
    const ProblemBoundary& boundary,
    std::ostream* tsp_log,
    std::vector<size_t>* tour_annotated = nullptr
) {
  std::optional<std::vector<TarelEdge>> mapped =
      BuildGroupMappedEdges(annotated, required);
  if (!mapped.has_value()) {
    return std::nullopt;
  }
  TspGraphData graph = MakeTspGraphEdges(*mapped, boundary);
  std::optional<TspTourResult> result = SolveTspAndExtractTour(
      *mapped, graph, boundary, std::nullopt, tsp_log, nullptr
  );
  if (!result.has_value()) {
    return std::nullopt;
  }
  if (tour_annotated != nullptr) {
    std::map<std::pair<TarelState, TarelState>, size_t> idx_by_pair;
    for (size_t i = 0; i < mapped->size(); ++i) {
      idx_by_pair[{(*mapped)[i].origin, (*mapped)[i].destination}] = i;
    }
    for (const TarelEdge& e : result->tour_edges) {
      tour_annotated->push_back(idx_by_pair.at({e.origin, e.destination}));
    }
  }
  return result->optimal_value;
}

// Investigate the gap between the Lagrangian dual bound and the true optimum.
//
// Decomposition: OPT - bound = slack1 + slack2, where
// - slack1 = OPT - (modified cost of the OPTIMAL tour's own tarel edge
//   sequence at the best costs): how much the relaxation undercharges each leg
//   of the right tour (its min picks a cheaper arrival time / step than the
//   real schedule allows, net of lambda corrections);
// - slack2 = (that modified cost) - (relaxed TSP optimum): how much cheaper a
//   different, time-inconsistent tour is under the same costs.
void AnalyzeGap(
    const ProblemState& state,
    const TarelEdgeIntermediateData& data,
    const StepPathsAdjacencyList& completed,
    const Prices& prices,
    const std::vector<StopId>& hk_tour,
    int hk_val,
    int relaxed_bound
) {
  // The slack accounting below covers only the stop-time lambdas.
  assert(!prices.coverage.has_value());
  const LagrangianCosts& costs = prices.stop_time;
  auto fmt = [](int seconds) {
    return TimeSinceServiceStart{seconds}.ToString();
  };
  auto name = [&](StopId stop) {
    std::string n = state.StopName(stop);
    return n.size() > 24 ? n.substr(0, 24) : n;
  };

  // Reconstruct the optimal tour's best timing chain.
  std::vector<Path> paths =
      ComputeMinimalFeasiblePathsAlong(hk_tour, completed);
  if (paths.empty()) {
    std::cout << "AnalyzeGap: no path along the optimal tour?!\n";
    return;
  }
  const Path* best = &paths[0];
  for (const Path& p : paths) {
    if (p.DurationSeconds() < best->DurationSeconds()) {
      best = &p;
    }
  }
  const std::vector<Step>& legs = best->steps;
  std::cout << "\n=== Gap analysis ===\n";
  std::cout << "Optimal tour duration along legs: "
            << fmt(best->DurationSeconds()) << " (HK optimum " << fmt(hk_val)
            << ")\n";

  std::vector<AnnotatedTarelEdge> annotated =
      BuildTarelEdgesWithCosts(data, prices);
  std::map<std::tuple<int, int, int, int>, const AnnotatedTarelEdge*>
      edge_by_key;
  for (const AnnotatedTarelEdge& a : annotated) {
    edge_by_key[{
        a.edge.origin.stop.v,
        a.edge.origin.partition.v,
        a.edge.destination.stop.v,
        a.edge.destination.partition.v
    }] = &a;
  }
  auto state_has_flex = [&](const TarelState& s) {
    auto it = data.arrival_times_to.find(s);
    return it == data.arrival_times_to.end() || it->second.has_flex;
  };
  auto argmin_candidate = [&](const TarelState& o,
                              const TarelState& d) -> EdgeCandidate {
    std::vector<EdgeCandidate> cands =
        EnumerateEdgeCandidates(data, prices, o, d);
    const EdgeCandidate* m = &cands[0];
    for (const EdgeCandidate& c : cands) {
      if (c.value < m->value) {
        m = &c;
      }
    }
    return *m;
  };

  // The single START tarel state.
  std::optional<StepPartitionId> start_partition;
  for (const auto& [s, _] : data.arrival_times_to) {
    if (s.stop == hk_tour.front()) {
      start_partition = s.partition;
    }
  }
  assert(start_partition.has_value());

  int t_prev;
  if (legs[0].is_flex) {
    t_prev = legs.size() > 1 && !legs[1].is_flex
                 ? legs[1].origin.time.seconds - legs[0].FlexDurationSeconds()
                 : 0;
  } else {
    t_prev = legs[0].origin.time.seconds;
  }
  TarelState prev_state{hk_tour.front(), *start_partition};

  std::cout << "\nPer-leg comparison on the optimal tour (actual = arrival to "
               "arrival; w = modified tarel weight at best costs):\n";
  double sum_modified = 0.0;
  long sum_w = 0;
  double slack_teleport = 0.0;
  double slack_other = 0.0;
  for (size_t i = 0; i < legs.size(); ++i) {
    int arrival_next = legs[i].is_flex ? t_prev + legs[i].FlexDurationSeconds()
                                       : legs[i].destination.time.seconds;
    TarelState next_state{
        legs[i].destination.stop, legs[i].destination.partition
    };
    int actual = arrival_next - t_prev;
    bool prev_flex = state_has_flex(prev_state);
    bool next_flex = state_has_flex(next_state);
    double lambda_out =
        prev_flex ? 0.0 : GetCost(costs, prev_state.stop, {t_prev});
    double lambda_in =
        next_flex ? 0.0 : GetCost(costs, next_state.stop, {arrival_next});
    double modified = actual + lambda_in - lambda_out;
    sum_modified += modified;

    auto it = edge_by_key.find(
        {prev_state.stop.v,
         prev_state.partition.v,
         next_state.stop.v,
         next_state.partition.v}
    );
    if (it == edge_by_key.end()) {
      std::cout << "  leg " << i << ": NO TAREL EDGE for "
                << name(prev_state.stop) << " -> " << name(next_state.stop)
                << "\n";
      t_prev = arrival_next;
      prev_state = next_state;
      continue;
    }
    int w = it->second->edge.weight;
    sum_w += w;
    double slack = modified - w;

    EdgeCandidate am = argmin_candidate(prev_state, next_state);
    bool teleport =
        am.origin_arrival.has_value() && am.origin_arrival->seconds != t_prev;
    (teleport ? slack_teleport : slack_other) += slack;

    std::cout << "  " << std::setw(2) << i << " " << std::setw(24)
              << name(prev_state.stop) << " -> " << std::setw(24)
              << name(next_state.stop) << " actual " << std::setw(5) << actual
              << "  w " << std::setw(6) << w << "  slack " << std::setw(7)
              << std::fixed << std::setprecision(1) << slack << "  dep@"
              << fmt(t_prev) << " argmin_dep@"
              << (am.origin_arrival.has_value()
                      ? fmt(am.origin_arrival->seconds)
                      : (prev_flex ? "flex" : "?"))
              << " arr@" << fmt(arrival_next) << " argmin_arr@"
              << (am.destination_arrival.has_value()
                      ? fmt(am.destination_arrival->seconds)
                      : (next_flex ? "flex" : "?"))
              << (prev_flex ? " [origin-flex]" : "")
              << (next_flex ? " [dest-flex]" : "") << "\n";

    t_prev = arrival_next;
    prev_state = next_state;
  }

  // Closing END->START edge of the relaxation's cycle.
  {
    TarelState start_state{hk_tour.front(), *start_partition};
    auto it = edge_by_key.find(
        {prev_state.stop.v,
         prev_state.partition.v,
         start_state.stop.v,
         start_state.partition.v}
    );
    bool prev_flex = state_has_flex(prev_state);
    double lambda_out =
        prev_flex ? 0.0 : GetCost(costs, prev_state.stop, {t_prev});
    double modified = 0.0 - lambda_out;
    sum_modified += modified;
    if (it != edge_by_key.end()) {
      int w = it->second->edge.weight;
      sum_w += w;
      EdgeCandidate am = argmin_candidate(prev_state, start_state);
      bool teleport =
          am.origin_arrival.has_value() && am.origin_arrival->seconds != t_prev;
      double slack = modified - w;
      (teleport ? slack_teleport : slack_other) += slack;
      std::cout << "  closing " << name(prev_state.stop) << " -> "
                << name(start_state.stop) << ": w " << w << "  modified "
                << std::setprecision(1) << modified << "  slack " << slack
                << "  arr_END@" << fmt(t_prev) << " argmin_dep@"
                << (am.origin_arrival.has_value()
                        ? fmt(am.origin_arrival->seconds)
                        : "flex")
                << "\n";
    } else {
      std::cout << "  closing edge missing?!\n";
    }
  }

  double slack1 = hk_val - static_cast<double>(sum_w);
  double slack2 = static_cast<double>(sum_w) - relaxed_bound;
  std::cout << "\nDecomposition of the gap (OPT " << hk_val
            << " - relaxed TSP bound " << relaxed_bound << " = "
            << (hk_val - relaxed_bound) << "):\n";
  std::cout << "  sum of modified actual leg costs (telescoping check, should "
               "be ~OPT): "
            << std::setprecision(1) << sum_modified << "\n";
  std::cout << "  slack1 (relaxation undercharging the OPTIMAL tour's edges): "
            << slack1 << "\n";
  std::cout << "    of which on legs whose argmin uses a different departure "
               "arrival-time (teleport): "
            << slack_teleport << ", other: " << slack_other << "\n";
  std::cout << "  slack2 (a different tour is cheaper under these costs):     "
            << slack2 << "\n";

  // What does the relaxed optimal tour do?
  std::vector<size_t> relaxed_tour;
  std::optional<int> relaxed_check = SolveUnmergedTspBound(
      annotated, state.required, state.boundary, nullptr, &relaxed_tour
  );
  if (!relaxed_check.has_value()) {
    return;
  }
  std::cout << "\nRelaxed optimal tour at best costs (TSP value "
            << *relaxed_check << "):\n";
  double sum_backjump = 0.0;
  for (size_t i = 0; i < relaxed_tour.size(); ++i) {
    const AnnotatedTarelEdge& e = annotated[relaxed_tour[i]];
    EdgeCandidate am = argmin_candidate(e.edge.origin, e.edge.destination);
    std::cout << "  " << std::setw(24) << name(e.edge.origin.stop) << " -> "
              << std::setw(24) << name(e.edge.destination.stop) << " w "
              << std::setw(6) << e.edge.weight << "  dep_used@"
              << (am.origin_arrival.has_value()
                      ? fmt(am.origin_arrival->seconds)
                      : "flex")
              << " arr@"
              << (am.destination_arrival.has_value()
                      ? fmt(am.destination_arrival->seconds)
                      : "flex");
    if (i + 1 < relaxed_tour.size()) {
      const AnnotatedTarelEdge& out = annotated[relaxed_tour[i + 1]];
      EdgeCandidate out_am =
          argmin_candidate(out.edge.origin, out.edge.destination);
      if (am.destination_arrival.has_value() &&
          out_am.origin_arrival.has_value()) {
        int jump =
            out_am.origin_arrival->seconds - am.destination_arrival->seconds;
        std::cout << "  next_dep_jump " << jump;
        if (jump < 0) {
          sum_backjump += -jump;
        }
      }
    }
    std::cout << "\n";
  }
  std::cout << "Total backwards time-jumps in the relaxed tour: "
            << std::setprecision(1) << sum_backjump << " s\n";
}

// LP lower bound on the tarel TSP: degree constraints over the transformed
// digraph (the same vertex-per-state graph with cycle edges and the -1
// partition offset that MakeTspGraphEdges builds for Concorde), x in [0, 1],
// plus lazily separated subtour elimination cuts. The LP optimum lower-bounds
// the TSP optimum, which lower-bounds the true tour cost.
//
// The structure is fixed for the whole run: the tarel edge SET does not depend
// on the Lagrangian costs (only weights do), so the model and the accumulated
// cut pool persist across iterations — cuts are weight-independent — and each
// iteration only changes objective coefficients and re-solves warm-started.
class LpTarelBound {
 public:
  LpTarelBound(
      const std::vector<AnnotatedTarelEdge>& annotated0,
      const RequiredStops& required,
      const ProblemBoundary& boundary
  ) {
    std::optional<std::vector<TarelEdge>> mapped_edges_opt =
        BuildGroupMappedEdges(annotated0, required);
    if (!mapped_edges_opt.has_value()) {
      return;
    }
    const std::vector<TarelEdge>& mapped_edges = *mapped_edges_opt;

    TspGraphData graph = MakeTspGraphEdges(mapped_edges, boundary);
    num_vertices_ = static_cast<int>(graph.state_by_id.size());
    correction_ = graph.expected_num_cycle_edges * kCycleEdgeWeight;
    state_by_id_ = graph.state_by_id;

    // MakeTspGraphEdges pushes all cycle edges first, then one inter-stop
    // edge per input edge in input order.
    size_t first_inter = graph.tsp_edges.size() - mapped_edges.size();
    for (size_t k = 0; k < graph.tsp_edges.size(); ++k) {
      const WeightedEdge& e = graph.tsp_edges[k];
      if (e.origin == e.destination) {
        // Self-loop cycle edge of a single-state stop; never usable.
        assert(k < first_inter);
        continue;
      }
      col_origin_.push_back(e.origin.v);
      col_dest_.push_back(e.destination.v);
      col_annotated_.push_back(
          k >= first_inter ? std::optional(k - first_inter) : std::nullopt
      );
    }

    // Base LP: out-degree row v, in-degree row num_vertices_ + v, all == 1.
    HighsLp lp;
    int num_cols = static_cast<int>(col_origin_.size());
    lp.num_col_ = num_cols;
    lp.num_row_ = 2 * num_vertices_;
    lp.col_cost_.assign(num_cols, 0.0);
    lp.col_lower_.assign(num_cols, 0.0);
    lp.col_upper_.assign(num_cols, 1.0);
    lp.row_lower_.assign(2 * num_vertices_, 1.0);
    lp.row_upper_.assign(2 * num_vertices_, 1.0);
    lp.a_matrix_.format_ = MatrixFormat::kColwise;
    // HighsSparseMatrix default-initializes start_ to {0}; assign rather than
    // append so the leading offset isn't duplicated (which would shift every
    // column's entries to the next column).
    lp.a_matrix_.start_.assign(1, 0);
    for (int c = 0; c < num_cols; ++c) {
      lp.a_matrix_.index_.push_back(col_origin_[c]);
      lp.a_matrix_.value_.push_back(1.0);
      lp.a_matrix_.index_.push_back(num_vertices_ + col_dest_[c]);
      lp.a_matrix_.value_.push_back(1.0);
      lp.a_matrix_.start_.push_back(
          static_cast<HighsInt>(lp.a_matrix_.index_.size())
      );
    }
    highs_.setOptionValue("output_flag", false);
    if (highs_.passModel(lp) != HighsStatus::kOk) {
      return;
    }

    // Seed the statically known subtour cuts: each stop's within-stop cycle
    // of -1000 bonus edges is a subtour the LP would otherwise love.
    std::map<StopId, std::vector<int>> vertices_by_stop;
    for (int v = 0; v < num_vertices_; ++v) {
      vertices_by_stop[graph.state_by_id[v].stop].push_back(v);
    }
    for (const auto& [stop, vertices] : vertices_by_stop) {
      if (vertices.size() < 2) {
        continue;
      }
      std::vector<char> in_set(num_vertices_, 0);
      for (int v : vertices) {
        in_set[v] = 1;
      }
      AddCutForSet(in_set);
    }

    col_cost_.assign(num_cols, 0.0);
    ok_ = true;
  }

  bool ok() const { return ok_; }
  int num_vertices() const { return num_vertices_; }
  int num_cuts() const { return num_cuts_; }

  struct LpResult {
    // ceil(LP objective - cycle bonus correction): a valid integer lower
    // bound on the tarel TSP value (of the modified weights).
    int lower_bound;
    // The same before rounding, for callers that subtract further constants.
    double corrected_value;
    std::vector<std::pair<size_t, double>> active;  // (annotated index, x)
    int separation_rounds;
  };

  std::optional<LpResult> Solve(
      const std::vector<AnnotatedTarelEdge>& annotated
  ) {
    for (size_t c = 0; c < col_origin_.size(); ++c) {
      col_cost_[c] = col_annotated_[c].has_value()
                         ? annotated[*col_annotated_[c]].edge.weight
                         : kCycleEdgeWeight;
    }
    highs_.changeColsCost(
        0, static_cast<HighsInt>(col_cost_.size()) - 1, col_cost_.data()
    );

    constexpr double kSupportTol = 1e-6;
    constexpr int kMaxRounds = 100;
    int rounds = 0;
    while (true) {
      if (highs_.run() != HighsStatus::kOk ||
          highs_.getModelStatus() != HighsModelStatus::kOptimal) {
        return std::nullopt;
      }
      const std::vector<double>& x = highs_.getSolution().col_value;

      // Components of the support graph; each component that isn't everything
      // is a violated subtour cut (its outflow is 0).
      std::vector<int> parent(num_vertices_);
      std::iota(parent.begin(), parent.end(), 0);
      auto find = [&](int v) {
        while (parent[v] != v) {
          parent[v] = parent[parent[v]];
          v = parent[v];
        }
        return v;
      };
      for (size_t c = 0; c < col_origin_.size(); ++c) {
        if (x[c] > kSupportTol) {
          parent[find(col_origin_[c])] = find(col_dest_[c]);
        }
      }
      std::unordered_map<int, std::vector<int>> components;
      for (int v = 0; v < num_vertices_; ++v) {
        components[find(v)].push_back(v);
      }
      if (components.size() == 1 || rounds >= kMaxRounds) {
        break;
      }
      for (const auto& [root, members] : components) {
        std::vector<char> in_set(num_vertices_, 0);
        for (int v : members) {
          in_set[v] = 1;
        }
        AddCutForSet(in_set);
      }
      rounds += 1;
    }

    LpResult result;
    double corrected = highs_.getInfo().objective_function_value - correction_;
    result.lower_bound = static_cast<int>(std::ceil(corrected - 1e-6));
    result.corrected_value = corrected;
    result.separation_rounds = rounds;
    const std::vector<double>& x = highs_.getSolution().col_value;
    for (size_t c = 0; c < col_origin_.size(); ++c) {
      if (col_annotated_[c].has_value() && x[c] > kSupportTol) {
        result.active.push_back({*col_annotated_[c], x[c]});
      }
    }
    return result;
  }

  // Debug: represent a TSP tour (its inter-stop tarel edges) as a 0/1 column
  // assignment (cycle columns forced by the degree structure) and report any
  // violated constraint plus the assignment's corrected cost.
  void DebugCheckTour(
      const std::vector<TarelEdge>& mapped, const std::vector<TarelEdge>& tour
  ) {
    std::map<std::pair<TarelState, TarelState>, size_t> idx_by_pair;
    for (size_t i = 0; i < mapped.size(); ++i) {
      idx_by_pair[{mapped[i].origin, mapped[i].destination}] = i;
    }
    std::vector<char> chosen(mapped.size(), 0);
    for (const TarelEdge& e : tour) {
      chosen[idx_by_pair.at({e.origin, e.destination})] = 1;
    }
    // The extractor omits the closing END->START edge; add it back.
    {
      auto closing_key =
          std::pair(tour.back().destination, tour.front().origin);
      auto it = idx_by_pair.find(closing_key);
      if (it == idx_by_pair.end()) {
        std::cout << "DebugCheckTour: closing edge ("
                  << closing_key.first.stop.v << ","
                  << closing_key.first.partition.v << ") -> ("
                  << closing_key.second.stop.v << ","
                  << closing_key.second.partition.v << ") HAS NO COLUMN\n";
      } else {
        std::cout << "DebugCheckTour: closing edge weight "
                  << mapped[it->second].weight << "\n";
        chosen[it->second] = 1;
      }
    }

    size_t num_cols = col_origin_.size();
    std::vector<double> x(num_cols, 0.0);
    std::vector<int> outdeg(num_vertices_, 0);
    std::vector<int> indeg(num_vertices_, 0);
    std::vector<int> cycle_out_col(num_vertices_, -1);
    for (size_t c = 0; c < num_cols; ++c) {
      if (col_annotated_[c].has_value()) {
        if (chosen[*col_annotated_[c]]) {
          x[c] = 1.0;
          outdeg[col_origin_[c]] += 1;
          indeg[col_dest_[c]] += 1;
        }
      } else {
        cycle_out_col[col_origin_[c]] = static_cast<int>(c);
      }
    }
    for (int v = 0; v < num_vertices_; ++v) {
      if (outdeg[v] == 0) {
        if (cycle_out_col[v] < 0) {
          std::cout << "DebugCheckTour: vertex " << v
                    << " has no out edge in the assignment\n";
          continue;
        }
        size_t c = cycle_out_col[v];
        x[c] = 1.0;
        outdeg[v] += 1;
        indeg[col_dest_[c]] += 1;
      }
    }
    for (int v = 0; v < num_vertices_; ++v) {
      if (outdeg[v] != 1 || indeg[v] != 1) {
        std::cout << "DebugCheckTour: vertex " << v << " ("
                  << state_by_id_[v].stop.v << ","
                  << state_by_id_[v].partition.v << ") outdeg " << outdeg[v]
                  << " indeg " << indeg[v] << "\n";
      }
    }
    double cost = 0.0;
    int num_cycle_used = 0;
    for (size_t c = 0; c < num_cols; ++c) {
      if (x[c] > 0.5) {
        if (col_annotated_[c].has_value()) {
          cost += col_cost_[c];
        } else {
          cost += kCycleEdgeWeight;
          num_cycle_used += 1;
        }
      }
    }
    std::cout << "DebugCheckTour: corrected cost " << (cost - correction_)
              << " (raw " << cost << ", cycle edges used " << num_cycle_used
              << ", correction expects " << correction_ / kCycleEdgeWeight
              << ")" << std::endl;

    // Evaluate the seeded per-stop cut rows locally.
    {
      std::map<StopId, std::vector<char>> in_set_by_stop;
      for (int v = 0; v < num_vertices_; ++v) {
        auto [it, inserted] = in_set_by_stop.try_emplace(
            state_by_id_[v].stop, std::vector<char>(num_vertices_, 0)
        );
        it->second[v] = 1;
      }
      for (const auto& [stop, in_set] : in_set_by_stop) {
        int size = 0;
        for (char b : in_set) {
          size += b;
        }
        if (size < 2) {
          continue;
        }
        double activity = 0.0;
        for (size_t c = 0; c < num_cols; ++c) {
          if (x[c] > 0.5 && in_set[col_origin_[c]] && !in_set[col_dest_[c]]) {
            activity += 1.0;
          }
        }
        if (activity < 1.0) {
          std::cout << "DebugCheckTour: seeded cut for stop " << stop.v << " ("
                    << size << " states) VIOLATED: activity " << activity
                    << "\n";
        }
      }
    }

    // Diff the model HiGHS holds against our arrays: row activities of the
    // assignment under HiGHS's own matrix, and cost comparison.
    {
      HighsLp hlp = highs_.getLp();
      hlp.a_matrix_.ensureColwise();
      std::cout << "DebugCheckTour: HiGHS num_col " << hlp.num_col_ << " (ours "
                << num_cols << "), num_row " << hlp.num_row_ << " (ours "
                << 2 * num_vertices_ + num_cuts_ << ")\n";
      int cost_diffs = 0;
      for (int c = 0; c < hlp.num_col_ && c < static_cast<int>(num_cols); ++c) {
        if (hlp.col_cost_[c] != col_cost_[c]) {
          if (cost_diffs == 0) {
            std::cout << "DebugCheckTour: first cost diff at col " << c
                      << ": highs " << hlp.col_cost_[c] << " ours "
                      << col_cost_[c] << "\n";
          }
          cost_diffs += 1;
        }
      }
      std::cout << "DebugCheckTour: cost diffs " << cost_diffs << "\n";
      std::vector<double> activity(hlp.num_row_, 0.0);
      for (int c = 0; c < hlp.num_col_; ++c) {
        if (x[c] > 0.5) {
          for (HighsInt k = hlp.a_matrix_.start_[c];
               k < hlp.a_matrix_.start_[c + 1];
               ++k) {
            activity[hlp.a_matrix_.index_[k]] += hlp.a_matrix_.value_[k];
          }
        }
      }
      int row_violations = 0;
      for (int r = 0; r < hlp.num_row_; ++r) {
        if (activity[r] < hlp.row_lower_[r] - 1e-6 ||
            activity[r] > hlp.row_upper_[r] + 1e-6) {
          if (row_violations < 5) {
            std::cout << "DebugCheckTour: row " << r << " activity "
                      << activity[r] << " bounds [" << hlp.row_lower_[r] << ", "
                      << hlp.row_upper_[r] << "]";
            if (r < num_vertices_) {
              std::cout << " (out-degree of vertex " << r << ")";
            } else if (r < 2 * num_vertices_) {
              std::cout << " (in-degree of vertex " << (r - num_vertices_)
                        << ")";
            } else {
              std::cout << " (cut " << (r - 2 * num_vertices_) << ")";
            }
            std::cout << "\n";
          }
          row_violations += 1;
        }
      }
      std::cout << "DebugCheckTour: row violations " << row_violations
                << std::endl;
    }

    // Fix the assignment into the HiGHS model and let it judge feasibility.
    highs_.changeColsBounds(
        0, static_cast<HighsInt>(num_cols) - 1, x.data(), x.data()
    );
    HighsStatus run_status = highs_.run();
    std::cout << "DebugCheckTour: HiGHS with fixed assignment: status "
              << static_cast<int>(run_status) << ", model status "
              << highs_.modelStatusToString(highs_.getModelStatus())
              << ", corrected obj "
              << (highs_.getInfo().objective_function_value - correction_)
              << std::endl;
  }

  // Debug: solve the current model with integrality to compare against the
  // exact TSP bound on the same graph. Leaves the model integer-constrained,
  // so only call this right before exiting.
  std::optional<int> DebugSolveMip() {
    std::vector<HighsVarType> integrality(
        col_origin_.size(), HighsVarType::kInteger
    );
    highs_.changeColsIntegrality(
        0, static_cast<HighsInt>(col_origin_.size()) - 1, integrality.data()
    );
    if (highs_.run() != HighsStatus::kOk ||
        highs_.getModelStatus() != HighsModelStatus::kOptimal) {
      return std::nullopt;
    }
    double corrected = highs_.getInfo().objective_function_value - correction_;
    return static_cast<int>(std::llround(corrected));
  }

 private:
  void AddCutForSet(const std::vector<char>& in_set) {
    std::vector<HighsInt> indices;
    std::vector<double> values;
    for (size_t c = 0; c < col_origin_.size(); ++c) {
      if (in_set[col_origin_[c]] && !in_set[col_dest_[c]]) {
        indices.push_back(static_cast<HighsInt>(c));
        values.push_back(1.0);
      }
    }
    highs_.addRow(
        1.0,
        kHighsInf,
        static_cast<HighsInt>(indices.size()),
        indices.data(),
        values.data()
    );
    num_cuts_ += 1;
  }

  Highs highs_;
  int num_vertices_ = 0;
  int correction_ = 0;
  std::vector<TarelState> state_by_id_;
  int num_cuts_ = 0;
  std::vector<int> col_origin_;
  std::vector<int> col_dest_;
  std::vector<std::optional<size_t>> col_annotated_;
  std::vector<double> col_cost_;
  bool ok_ = false;
};

std::string FormatDuration(int ms) {
  if (ms < 1000) {
    return std::to_string(ms) + " ms";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1) << (ms / 1000.0) << " s";
  return ss.str();
}

// Quiet, self-contained LP-engine subgradient run for the bucket sweep. Same
// algorithm as the main loop (epsilon-spread subgradient, CFM momentum,
// adaptive target level with annealed spread), plus a stall exit: once the
// target level is at its floor and another patience window passes without
// improvement, further iterations are pointless.
struct SweepSolveOptions {
  int ub;
  int max_iterations;
  double mu;
  double initial_delta;
  int patience;
  double momentum;
  double eps;
  // Coverage prices: enabled for a subproblem whose starts are pinned to
  // [cov_t1, cov_t2] (grid spans [cov_t1, cov_t2 + ub)).
  bool coverage = false;
  int cov_t1 = 0;
  int cov_t2 = 0;
  int cov_res = 60;
  // Per-iteration progress lines when non-null.
  std::ostream* log = nullptr;
};

struct SweepSolveResult {
  bool infeasible = false;
  int best_bound = std::numeric_limits<int>::min();
  Prices best_prices;
  // The LP active set at the best bound (annotated indices with mass).
  std::vector<std::pair<size_t, double>> best_active;
  int iterations_run = 0;
};

// Arrival-time upper bounds for flex-arrival states: a state whose flex
// in-steps are all zero-duration steps from START arrives exactly at the tour
// start, which is at most the bucket end t2 (mixed-in scheduled arrivals can
// also be as late as their max). Other flex states get no bound.
std::unordered_map<TarelState, int> ComputeFlexArrivalUbs(
    const TarelEdgeIntermediateData& data, StopId start_stop, int t2
) {
  std::unordered_map<TarelState, bool> only_zero_from_start;
  for (const auto& [origin_stop, by_dest] : data.steps_from) {
    for (const auto& [dest, steps] : by_dest) {
      for (const Step& s : steps) {
        if (!s.is_flex) {
          continue;
        }
        bool zero_from_start =
            s.FlexDurationSeconds() == 0 && origin_stop == start_stop;
        auto [it, inserted] =
            only_zero_from_start.try_emplace(dest, zero_from_start);
        if (!inserted) {
          it->second = it->second && zero_from_start;
        }
      }
    }
  }
  std::unordered_map<TarelState, int> ubs;
  for (const auto& [s, arrivals] : data.arrival_times_to) {
    if (!arrivals.has_flex) {
      continue;
    }
    auto it = only_zero_from_start.find(s);
    if (it == only_zero_from_start.end() || !it->second) {
      continue;
    }
    int ub = t2;
    if (!arrivals.times.empty()) {
      ub = std::max(ub, arrivals.times.back().seconds);
    }
    ubs[s] = ub;
  }
  return ubs;
}

SweepSolveResult RunSubgradientLp(
    const ProblemState& state,
    const TarelEdgeIntermediateData& data,
    const SweepSolveOptions& opts,
    Prices warm
) {
  SweepSolveResult result;
  // Two-phase schedule: optimize the stop-time lambdas alone first (they
  // converge fast), and only once they stall attach the (dense) coverage
  // prices, warm-started from the lambda optimum. Starting jointly from
  // scratch lets the dense coverage gradient swamp the lambda components and
  // converges much slower. Sparse must-span prices (warm.spans) are few and
  // sparse, so they are active from the start.
  Prices prices = std::move(warm);
  prices.coverage.reset();

  std::vector<AnnotatedTarelEdge> annotated =
      BuildTarelEdgesWithCosts(data, prices);
  LpTarelBound lp(annotated, state.required, state.boundary);
  if (!lp.ok()) {
    result.infeasible = true;
    return result;
  }

  double delta = opts.initial_delta;
  int iterations_without_improvement = 0;
  std::unordered_map<CostKey, double> direction;
  std::vector<double> direction_cov;
  std::vector<double> direction_span;
  if (prices.spans.has_value()) {
    direction_span.assign(prices.spans->n(), 0.0);
  }

  auto activate_coverage = [&]() {
    prices.coverage.emplace(opts.cov_t1, opts.cov_t2, opts.ub, opts.cov_res);
    prices.coverage->SetArrivalUbs(
        ComputeFlexArrivalUbs(data, state.boundary.start, opts.cov_t2)
    );
    if (result.best_bound > 0) {
      prices.coverage->SetLb(result.best_bound);
    }
    direction_cov.assign(prices.coverage->n(), 0.0);
    direction.clear();
    // Restart the target level gently: a full reset makes the first coverage
    // steps huge, wrecking the converged lambda equilibrium.
    delta = std::min(opts.initial_delta, 10.0);
    iterations_without_improvement = 0;
    // Keep the incumbent snapshot coverage-enabled so a later backtrack
    // doesn't silently deactivate coverage (pi = 0 leaves its bound intact).
    result.best_prices.coverage = prices.coverage;
    if (opts.log != nullptr) {
      *opts.log << "Activating coverage prices (" << prices.coverage->n()
                << " grid points)" << std::endl;
    }
  };

  for (int iter = 0; iter < opts.max_iterations; ++iter) {
    if (iter > 0) {
      annotated = BuildTarelEdgesWithCosts(data, prices);
    }
    std::optional<LpTarelBound::LpResult> lp_result = lp.Solve(annotated);
    if (!lp_result.has_value()) {
      if (iter == 0) {
        result.infeasible = true;
      }
      return result;
    }
    result.iterations_run = iter + 1;
    CoverageSpan max_span;
    double cov_constant = prices.coverage.has_value()
                              ? prices.coverage->MaxAdmissibleSpan(&max_span)
                              : 0.0;
    double span_total = prices.spans.has_value() ? prices.spans->Total() : 0.0;
    int bound = static_cast<int>(
        std::ceil(lp_result->corrected_value - cov_constant - span_total - 1e-6)
    );
    if (bound > result.best_bound) {
      result.best_bound = bound;
      result.best_prices = prices;
      result.best_active = lp_result->active;
      iterations_without_improvement = 0;
      delta = std::min(delta * 1.2, opts.initial_delta);
      if (prices.coverage.has_value()) {
        // Every tour in this bucket lasts at least best_bound; admissible
        // spans shrink accordingly.
        prices.coverage->SetLb(result.best_bound);
      }
    } else {
      iterations_without_improvement += 1;
      if (iterations_without_improvement >= opts.patience) {
        if (delta <= 1.0) {
          // Stalled at the floor level; converged for our purposes.
          if (opts.coverage && !prices.coverage.has_value()) {
            activate_coverage();
            continue;
          }
          break;
        }
        delta = std::max(delta / 2.0, 1.0);
        iterations_without_improvement = 0;
      }
    }
    if (bound >= opts.ub) {
      break;  // Bucket cannot contain an improving tour.
    }
    if (result.best_bound > std::numeric_limits<int>::min() &&
        bound < result.best_bound - std::max(100.0, 10.0 * delta)) {
      // The prices wandered far below the incumbent (the Polyak step
      // amplifies on dips); snap back and take smaller steps.
      prices = result.best_prices;
      direction.clear();
      std::fill(direction_cov.begin(), direction_cov.end(), 0.0);
      std::fill(direction_span.begin(), direction_span.end(), 0.0);
      delta = std::max(delta / 2.0, 1.0);
      continue;
    }

    double effective_eps = std::min(opts.eps, std::max(delta, 1.0));
    std::unordered_map<CostKey, double> gradient;
    std::vector<double> cov_gradient;
    if (prices.coverage.has_value()) {
      // d(bound)/d(pi_g) = (coverage of g by the active solution)
      //                    - (1 if g is in the max admissible span).
      cov_gradient.assign(prices.coverage->n(), 0.0);
      for (int k = max_span.k1; k < max_span.k2; ++k) {
        cov_gradient[k] -= 1.0;
      }
    }
    std::vector<double> span_gradient;
    if (prices.spans.has_value()) {
      // d(bound)/d(pi_t) = (spanning mass of the active solution) - 1.
      span_gradient.assign(prices.spans->n(), -1.0);
    }
    for (const auto& [idx, mass] : lp_result->active) {
      const AnnotatedTarelEdge& a = annotated[idx];
      std::vector<EdgeCandidate> candidates = EnumerateEdgeCandidates(
          data, prices, a.edge.origin, a.edge.destination
      );
      double min_value = std::numeric_limits<double>::infinity();
      for (const EdgeCandidate& c : candidates) {
        min_value = std::min(min_value, c.value);
      }
      std::erase_if(candidates, [&](const EdgeCandidate& c) {
        return c.value > min_value + effective_eps;
      });
      if (candidates.empty()) {
        continue;
      }
      double w = mass / candidates.size();
      for (const EdgeCandidate& c : candidates) {
        if (c.destination_arrival.has_value()) {
          gradient[{a.edge.destination.stop, *c.destination_arrival}] += w;
        }
        if (c.origin_arrival.has_value()) {
          gradient[{a.edge.origin.stop, *c.origin_arrival}] -= w;
        }
        if (!cov_gradient.empty()) {
          for (int k = c.span.k1; k < c.span.k2; ++k) {
            cov_gradient[k] += w;
          }
        }
        if (!span_gradient.empty()) {
          prices.spans->CountSpanned(c, w, span_gradient);
        }
      }
    }
    std::erase_if(gradient, [](const auto& kv) {
      return std::abs(kv.second) < 1e-9;
    });
    double norm_sq = 0.0;
    for (const auto& [key, g] : gradient) {
      norm_sq += g * g;
    }
    for (double g : cov_gradient) {
      norm_sq += g * g;
    }
    for (double g : span_gradient) {
      norm_sq += g * g;
    }
    if (opts.log != nullptr) {
      *opts.log << "Iteration " << std::setw(4) << iter << ": bound " << bound
                << ", best " << result.best_bound << ", |g|^2 "
                << std::setprecision(3) << norm_sq << ", delta " << delta
                << ", cov_c " << cov_constant << std::endl;
    }
    if (norm_sq == 0.0) {
      break;
    }

    // CFM momentum over the joint (stop-time, coverage) direction.
    double direction_norm_sq = 0.0;
    for (const auto& [key, d] : direction) {
      direction_norm_sq += d * d;
    }
    for (double d : direction_cov) {
      direction_norm_sq += d * d;
    }
    for (double d : direction_span) {
      direction_norm_sq += d * d;
    }
    double beta = 0.0;
    if (opts.momentum > 0.0 && direction_norm_sq > 0.0) {
      double dot = 0.0;
      for (const auto& [key, g] : gradient) {
        auto it = direction.find(key);
        if (it != direction.end()) {
          dot += it->second * g;
        }
      }
      for (size_t k = 0; k < cov_gradient.size(); ++k) {
        dot += direction_cov[k] * cov_gradient[k];
      }
      for (size_t k = 0; k < span_gradient.size(); ++k) {
        dot += direction_span[k] * span_gradient[k];
      }
      if (dot < 0.0) {
        beta = -opts.momentum * dot / direction_norm_sq;
      }
    }
    if (beta == 0.0) {
      direction.clear();
      std::fill(direction_cov.begin(), direction_cov.end(), 0.0);
      std::fill(direction_span.begin(), direction_span.end(), 0.0);
    } else {
      for (auto& [key, d] : direction) {
        d *= beta;
      }
      for (double& d : direction_cov) {
        d *= beta;
      }
      for (double& d : direction_span) {
        d *= beta;
      }
    }
    for (const auto& [key, g] : gradient) {
      direction[key] += g;
    }
    for (size_t k = 0; k < cov_gradient.size(); ++k) {
      direction_cov[k] += cov_gradient[k];
    }
    for (size_t k = 0; k < span_gradient.size(); ++k) {
      direction_span[k] += span_gradient[k];
    }
    std::erase_if(direction, [](const auto& kv) {
      return std::abs(kv.second) < 1e-9;
    });
    double step_norm_sq = 0.0;
    for (const auto& [key, d] : direction) {
      step_norm_sq += d * d;
    }
    for (double d : direction_cov) {
      step_norm_sq += d * d;
    }
    for (double d : direction_span) {
      step_norm_sq += d * d;
    }
    if (step_norm_sq == 0.0) {
      for (const auto& [key, g] : gradient) {
        direction[key] = g;
      }
      direction_cov = cov_gradient;
      direction_span = span_gradient;
      step_norm_sq = norm_sq;
    }

    double target = std::min<double>(result.best_bound + delta, opts.ub);
    // Clamp the Polyak gap so a dip below the incumbent doesn't amplify the
    // step size.
    double gap = std::min(target - bound, 3.0 * delta + 10.0);
    double alpha = opts.mu * gap / step_norm_sq;
    if (alpha <= 0.0) {
      alpha = opts.mu / step_norm_sq;
    }
    for (const auto& [key, d] : direction) {
      prices.stop_time[key] += alpha * d;
    }
    if (prices.coverage.has_value()) {
      std::vector<double> step(direction_cov.size());
      for (size_t k = 0; k < direction_cov.size(); ++k) {
        step[k] = alpha * direction_cov[k];
      }
      prices.coverage->Add(step);
    }
    if (prices.spans.has_value()) {
      std::vector<double> step(direction_span.size());
      for (size_t k = 0; k < direction_span.size(); ++k) {
        step[k] = alpha * direction_span[k];
      }
      prices.spans->Add(step);
    }
  }
  return result;
}

struct BunchPick {
  int t_max;     // grid time with the highest span multiplicity
  double mult;   // that multiplicity
  int t_median;  // mass-weighted median span time (bisection fallback)
};

// Span profile of the active relaxed solution (mass-weighted, argmin
// candidates, definite span parts only): the bunchiest grid time (excluding
// times within `exclude_radius` of `exclude`) and the mass median. Returns
// nullopt if nothing spans anything.
std::optional<BunchPick> FindBunchiestTime(
    const TarelEdgeIntermediateData& data,
    const Prices& prices,
    const std::vector<AnnotatedTarelEdge>& annotated,
    const std::vector<std::pair<size_t, double>>& active,
    int grid_lo,
    int grid_hi,
    int res,
    int exclude_radius,
    const std::vector<int>& exclude
) {
  if (grid_hi <= grid_lo) {
    return std::nullopt;
  }
  int n = (grid_hi - grid_lo) / res + 1;
  std::vector<double> profile(n, 0.0);
  for (const auto& [idx, mass] : active) {
    const AnnotatedTarelEdge& a = annotated[idx];
    std::vector<EdgeCandidate> cands = EnumerateEdgeCandidates(
        data, prices, a.edge.origin, a.edge.destination
    );
    if (cands.empty()) {
      continue;
    }
    const EdgeCandidate* am = &cands[0];
    for (const EdgeCandidate& c : cands) {
      if (c.value < am->value) {
        am = &c;
      }
    }
    if (am->leg_kind == EdgeCandidate::LegKind::kFloating) {
      continue;
    }
    int k1 = std::max(0, (am->leg_a - grid_lo + res - 1) / res);
    int k2 = std::min(n, (am->leg_d - grid_lo + res - 1) / res);
    for (int k = k1; k < k2; ++k) {
      profile[k] += mass;
    }
  }
  std::optional<std::pair<int, double>> best;
  double total = 0.0;
  for (int k = 0; k < n; ++k) {
    total += profile[k];
    int t = grid_lo + k * res;
    bool excluded = false;
    for (int e : exclude) {
      if (std::abs(t - e) < exclude_radius) {
        excluded = true;
        break;
      }
    }
    if (excluded) {
      continue;
    }
    if (!best.has_value() || profile[k] > best->second) {
      best = {t, profile[k]};
    }
  }
  if (!best.has_value() || total <= 0.0) {
    return std::nullopt;
  }
  double cum = 0.0;
  int t_median = grid_lo;
  for (int k = 0; k < n; ++k) {
    cum += profile[k];
    if (cum >= total / 2.0) {
      t_median = grid_lo + k * res;
      break;
    }
  }
  return BunchPick{best->first, best->second, t_median};
}

}  // namespace

int main(int argc, char* argv[]) {
  CLI::App app{"Lagrangian-cost tarel lower bound (subgradient optimization)"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  int max_iterations = 200;
  app.add_option(
      "--iters", max_iterations, "Max subgradient iterations (default 200)"
  );

  std::optional<int> ub;
  app.add_option(
      "--ub",
      ub,
      "Known upper bound in seconds; caps the adaptive target and is used for "
      "gap reporting"
  );

  bool run_hk = false;
  app.add_flag(
      "--hk",
      run_hk,
      "First solve exactly with Held-Karp DP and use the optimum as the "
      "upper bound"
  );

  double mu = 1.0;
  app.add_option("--mu", mu, "Step size multiplier (default 1.0)");

  double delta = 100.0;
  app.add_option(
      "--delta",
      delta,
      "Initial target level above the best bound, in seconds (default 100)"
  );

  int patience = 10;
  app.add_option(
      "--patience",
      patience,
      "Halve the target level after this many non-improving iterations "
      "(default 10)"
  );

  double momentum = 1.5;
  app.add_option(
      "--momentum",
      momentum,
      "CFM momentum strength gamma in [0, 2]; 0 disables momentum "
      "(default 1.5)"
  );

  double eps = 30.0;
  app.add_option(
      "--eps",
      eps,
      "Spread each tour edge's subgradient mass over all candidates within "
      "this many seconds of the edge's minimum (default 30)"
  );

  bool verbose_tsp = false;
  app.add_flag("--verbose-tsp", verbose_tsp, "Log Concorde output to stderr");

  bool use_concorde = false;
  app.add_flag(
      "--concorde",
      use_concorde,
      "Solve the tarel TSP exactly with Concorde every iteration instead of "
      "the LP relaxation"
  );

  bool debug_mip = false;
  app.add_flag(
      "--debug-mip",
      debug_mip,
      "Debug: after the first LP solve, re-solve the model as a MIP, print "
      "the value (should match the exact TSP bound), and exit"
  );

  bool analyze_gap = false;
  app.add_flag(
      "--analyze-gap",
      analyze_gap,
      "After optimizing, analyze what causes the remaining gap between the "
      "dual bound and the optimum (implies --hk)"
  );

  std::optional<int> window_start;
  app.add_option(
      "--window-start",
      window_start,
      "Restrict to tours departing at or after this time (seconds since "
      "service start): delete scheduled candidates departing earlier or "
      "arriving after window-start + window-len. The bound is then valid only "
      "for tours starting in the window."
  );

  std::optional<int> window_len;
  app.add_option(
      "--window-len",
      window_len,
      "Length of the candidate window (default: the upper bound). Use "
      "bucket_width + UB to cover all starts in [window-start, window-start + "
      "bucket_width]."
  );

  bool sweep = false;
  app.add_flag(
      "--sweep",
      sweep,
      "Bisection sweep over start-time buckets: recursively split the range "
      "of possible start times, bounding each bucket's window-clipped "
      "subproblem and pruning buckets whose bound reaches the upper bound. "
      "Requires --ub or --hk."
  );

  int sweep_iters = 400;
  app.add_option(
      "--sweep-iters",
      sweep_iters,
      "Max subgradient iterations per sweep node (default 400)"
  );

  int min_bucket = 3600;
  app.add_option(
      "--min-bucket",
      min_bucket,
      "Stop splitting sweep buckets narrower than this many seconds "
      "(default 3600)"
  );

  bool coverage = false;
  app.add_flag(
      "--coverage",
      coverage,
      "Add coverage prices: a tour with a pinned start bucket covers each "
      "grid time at most once, and each grid time in the guaranteed window "
      "exactly once. Works with --sweep and --window-start; requires an "
      "upper bound."
  );

  int coverage_res = 60;
  app.add_option(
      "--coverage-res",
      coverage_res,
      "Coverage price grid resolution in seconds (default 60)"
  );

  bool span_bnb = false;
  app.add_flag(
      "--span-bnb",
      span_bnb,
      "Branch and bound on span constraints: solve the root with stop-time "
      "lambdas only, then repeatedly branch on the most-bunched time (the "
      "tour spans it / does not span it). Requires --ub or --hk; per-node "
      "budget from --sweep-iters/--patience."
  );

  int bnb_nodes = 40;
  app.add_option(
      "--bnb-nodes", bnb_nodes, "Max span-bnb nodes to expand (default 40)"
  );

  int bnb_max_depth = 12;
  app.add_option(
      "--bnb-max-depth",
      bnb_max_depth,
      "Max span-bnb branching depth (default 12)"
  );

  int bnb_seconds = 0;
  app.add_option(
      "--bnb-seconds",
      bnb_seconds,
      "Stop expanding span-bnb nodes after this many seconds (0 = no limit)"
  );

  int bnb_root_iters = 0;
  app.add_option(
      "--bnb-root-iters",
      bnb_root_iters,
      "Iteration budget for the span-bnb root node (0 = same as "
      "--sweep-iters). The root's own bound stops mattering once it branches, "
      "so a small budget saves minutes."
  );

  std::string bnb_strategy = "auto";
  app.add_option(
      "--bnb-strategy",
      bnb_strategy,
      "Span-bnb branch time selection: auto (mult while improving, else "
      "bisect), bisect (always median/gap midpoint), mult (always bunchiest)"
  );

  bool bnb_no_coverage = false;
  app.add_flag(
      "--bnb-no-coverage",
      bnb_no_coverage,
      "Disable dense coverage prices on span-bnb nodes with a bounded "
      "effective window (on by default)"
  );

  CLI11_PARSE(app, argc, argv);

  if (coverage && analyze_gap) {
    std::cerr << "--coverage does not support --analyze-gap\n";
    return 1;
  }
  if (coverage && use_concorde) {
    std::cerr << "--coverage requires the LP engine (no --concorde)\n";
    return 1;
  }
  if (coverage && !ub.has_value() && !run_hk) {
    std::cerr << "--coverage requires --ub or --hk\n";
    return 1;
  }

  if (analyze_gap) {
    run_hk = true;
  }

  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << "\n";
    return 1;
  }
  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState state = j.get<ProblemState>();
  in.close();

  std::cout << "Loaded problem state from: " << input_path << "\n";
  std::cout << "Stops: " << state.minimal.NumStops() << "\n";
  std::cout << "Required stops: " << state.required.size() << "\n";

  std::optional<HeldKarpDPResult> hk_result;
  if (run_hk) {
    std::cout << "\nSolving exactly with Held-Karp DP...\n";
    auto hk_start = std::chrono::steady_clock::now();
    HeldKarpDPResult hk = HeldKarpDPSolve(state, 0, &std::cerr);
    hk_result = hk;
    auto hk_end = std::chrono::steady_clock::now();
    int hk_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(hk_end - hk_start)
            .count();
    std::cout << "Held-Karp optimum: " << TimeSinceServiceStart{hk.best_val}
              << " (" << hk.best_val << " s) in " << FormatDuration(hk_ms)
              << std::endl;
    ub = hk.best_val;
  }

  auto start = std::chrono::steady_clock::now();

  std::cout << "\nComputing completed graph...\n";
  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();
  std::vector<Step> merged_steps = completed.AllMergedSteps();

  if (sweep) {
    if (!ub.has_value()) {
      std::cerr << "--sweep requires --ub or --hk\n";
      return 1;
    }
    // Any tour's first scheduled leg departs somewhere in [t_min, t_max]
    // (flex legs are never clipped, so all-flex prefixes and tours stay
    // representable in every bucket).
    int t_min = std::numeric_limits<int>::max();
    int t_max = std::numeric_limits<int>::min();
    for (const Step& s : merged_steps) {
      if (!s.is_flex) {
        t_min = std::min(t_min, s.origin.time.seconds);
        t_max = std::max(t_max, s.origin.time.seconds);
      }
    }
    if (t_min > t_max) {
      std::cerr << "No scheduled steps; nothing to sweep\n";
      return 1;
    }
    std::cout << "Sweeping start times in [" << TimeSinceServiceStart{t_min}
              << ", " << TimeSinceServiceStart{t_max} << "] against UB "
              << TimeSinceServiceStart{*ub} << " (" << *ub << " s)"
              << std::endl;

    struct SweepNode {
      int t1;
      int t2;
      int depth;
      LagrangianCosts warm;
    };
    struct SweepLeaf {
      int t1;
      int t2;
      int bound;
    };
    std::vector<SweepNode> node_stack;
    node_stack.push_back(SweepNode{t_min, t_max, 0, {}});
    std::vector<SweepLeaf> leaves;
    int num_nodes = 0;
    int num_pruned = 0;
    int num_infeasible = 0;

    while (!node_stack.empty()) {
      SweepNode node = std::move(node_stack.back());
      node_stack.pop_back();
      num_nodes += 1;
      auto node_start = std::chrono::steady_clock::now();

      // Tours starting in [t1, t2] with duration < UB use scheduled steps
      // departing >= t1 and arriving <= t2 + UB.
      std::vector<Step> clipped = merged_steps;
      std::erase_if(clipped, [&](const Step& s) {
        return !s.is_flex && (s.origin.time.seconds < node.t1 ||
                              s.destination.time.seconds > node.t2 + *ub);
      });
      TarelEdgeIntermediateData node_data =
          ComputeTarelIntermediateData(clipped);
      SweepSolveOptions node_opts{
          .ub = *ub,
          .max_iterations = sweep_iters,
          .mu = mu,
          .initial_delta = delta,
          .patience = patience,
          .momentum = momentum,
          .eps = eps,
          .coverage = coverage,
          .cov_t1 = node.t1,
          .cov_t2 = node.t2,
          .cov_res = coverage_res,
      };
      SweepSolveResult r = RunSubgradientLp(
          state,
          node_data,
          node_opts,
          Prices{std::move(node.warm), std::nullopt, std::nullopt}
      );

      std::string decision;
      if (r.infeasible) {
        num_infeasible += 1;
        num_pruned += 1;
        decision = "infeasible -> pruned";
      } else if (r.best_bound >= *ub) {
        num_pruned += 1;
        decision = "pruned";
      } else if (node.t2 - node.t1 <= min_bucket) {
        // Leaf: lift the LP-strength bound with one exact TSP solve (on the
        // modified weights; subtract the coverage constant back out).
        std::vector<AnnotatedTarelEdge> annotated_best =
            BuildTarelEdgesWithCosts(node_data, r.best_prices);
        std::optional<int> cert = SolveUnmergedTspBound(
            annotated_best, state.required, state.boundary, nullptr
        );
        int leaf_bound = r.best_bound;
        if (cert.has_value()) {
          double cov_c =
              r.best_prices.coverage.has_value()
                  ? r.best_prices.coverage->MaxAdmissibleSpan(nullptr)
                  : 0.0;
          leaf_bound = std::max(
              leaf_bound, static_cast<int>(std::ceil(*cert - cov_c - 1e-6))
          );
        }
        if (leaf_bound >= *ub) {
          num_pruned += 1;
          decision = "pruned (certified " + std::to_string(leaf_bound) + ")";
        } else {
          leaves.push_back(SweepLeaf{node.t1, node.t2, leaf_bound});
          decision = "leaf, certified bound " + std::to_string(leaf_bound);
        }
      } else {
        int mid = (node.t1 + node.t2) / 2;
        node_stack.push_back(
            SweepNode{node.t1, mid, node.depth + 1, r.best_prices.stop_time}
        );
        node_stack.push_back(
            SweepNode{
                mid, node.t2, node.depth + 1, std::move(r.best_prices.stop_time)
            }
        );
        decision = "split";
      }

      auto node_end = std::chrono::steady_clock::now();
      int node_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        node_end - node_start
      )
                        .count();
      std::cout << std::string(2 * node.depth, ' ') << "["
                << TimeSinceServiceStart{node.t1} << ", "
                << TimeSinceServiceStart{node.t2} << "] bound "
                << (r.best_bound == std::numeric_limits<int>::min()
                        ? std::string("-")
                        : std::to_string(r.best_bound))
                << " (" << r.iterations_run << " iters, "
                << FormatDuration(node_ms) << "): " << decision << std::endl;
    }

    auto end = std::chrono::steady_clock::now();
    int total_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();

    std::sort(leaves.begin(), leaves.end(), [](const auto& a, const auto& b) {
      return a.t1 < b.t1;
    });
    std::cout << "\nSweep: " << num_nodes << " nodes, " << num_pruned
              << " pruned (" << num_infeasible << " infeasible), "
              << leaves.size() << " unresolved leaves\n";
    int global_lb = *ub;
    for (const SweepLeaf& leaf : leaves) {
      std::cout << "  leaf [" << TimeSinceServiceStart{leaf.t1} << ", "
                << TimeSinceServiceStart{leaf.t2}
                << "]: " << TimeSinceServiceStart{leaf.bound} << " ("
                << leaf.bound << " s)\n";
      global_lb = std::min(global_lb, leaf.bound);
    }
    if (leaves.empty()) {
      std::cout << "All buckets pruned: no tour beats the upper bound; UB "
                << TimeSinceServiceStart{*ub} << " (" << *ub
                << " s) is optimal.\n";
    } else {
      std::cout << "Global lower bound: " << TimeSinceServiceStart{global_lb}
                << " (" << global_lb << " s), gap " << std::fixed
                << std::setprecision(2) << 100.0 * (*ub - global_lb) / *ub
                << "%\n";
    }
    std::cout << "Total time: " << FormatDuration(total_ms) << "\n";
    return 0;
  }

  if (window_start.has_value()) {
    if (!window_len.has_value()) {
      if (!ub.has_value()) {
        std::cerr << "--window-start requires --window-len or --ub/--hk\n";
        return 1;
      }
      window_len = *ub;
    }
    int wstart = *window_start;
    int wend = wstart + *window_len;
    if (ub.has_value() && *window_len < *ub) {
      std::cout << "WARNING: window-len (" << *window_len << ") < ub (" << *ub
                << "): this restricts to tours of duration <= " << *window_len
                << ", which excludes longer tours entirely. For a start "
                   "bucket of width W covering all durations, use "
                   "window-len = W + ub.\n";
    }
    size_t before = merged_steps.size();
    // A tour departing in the window uses only scheduled steps departing at
    // or after wstart and arriving by wend. Flex steps have relative times
    // and stay. Filtering a sorted-and-minimal group by a time window keeps
    // it sorted and minimal.
    std::erase_if(merged_steps, [&](const Step& s) {
      return !s.is_flex && (s.origin.time.seconds < wstart ||
                            s.destination.time.seconds > wend);
    });
    std::cout << "Window [" << TimeSinceServiceStart{wstart} << ", "
              << TimeSinceServiceStart{wend} << "]: " << before << " -> "
              << merged_steps.size() << " completed steps\n";
  }
  // Range of possible tour starts (first scheduled leg departures), for
  // whole-range coverage mode.
  int sched_t_min = std::numeric_limits<int>::max();
  int sched_t_max = std::numeric_limits<int>::min();
  for (const Step& s : merged_steps) {
    if (!s.is_flex) {
      sched_t_min = std::min(sched_t_min, s.origin.time.seconds);
      sched_t_max = std::max(sched_t_max, s.origin.time.seconds);
    }
  }

  if (span_bnb) {
    if (!ub.has_value()) {
      std::cerr << "--span-bnb requires --ub or --hk\n";
      return 1;
    }
    auto fmt = [](long seconds) {
      return TimeSinceServiceStart{static_cast<int>(seconds)}.ToString();
    };
    // A node constrains the tour to span every must-time (sorted) and to lie
    // within the half-open window (wlo, whi]. Branching on time t* is
    // ternary: the tour spans t*, lies entirely at or before t*, or entirely
    // after t* (a contiguous tour admits no other case).
    // Tour-structure constraint: b immediately follows a (require), or the
    // leg a->b is unused (forbid). Implemented as completed-step deletion.
    struct EdgeConstraint {
      StopId a;
      StopId b;
      bool require;
    };
    struct BnbNode {
      std::vector<int> must;  // sorted
      long wlo;
      long whi;
      std::vector<SpanStopConstraint> span_stops;
      std::vector<EdgeConstraint> edges;
      int bound;  // inherited valid lower bound
      int depth;
      LagrangianCosts warm_lambda;
      std::vector<double> warm_pi;  // parallel to must
    };
    auto cmp = [](const BnbNode& a, const BnbNode& b) {
      return a.bound > b.bound;
    };
    std::priority_queue<BnbNode, std::vector<BnbNode>, decltype(cmp)> pq(cmp);
    pq.push(
        BnbNode{
            {},
            std::numeric_limits<long>::min() / 2,
            std::numeric_limits<long>::max() / 2,
            {},
            {},
            std::numeric_limits<int>::min(),
            0,
            {},
            {}
        }
    );
    struct BnbLeaf {
      int bound;
      int depth;
    };
    std::vector<BnbLeaf> leaves;
    int processed = 0;
    int pruned = 0;

    auto describe = [&](const BnbNode& n) {
      std::string s = "must{";
      for (size_t i = 0; i < n.must.size(); ++i) {
        s += (i ? "," : "") + fmt(n.must[i]);
      }
      s += "} win(";
      s += n.wlo < 0 ? "-inf" : fmt(n.wlo);
      s += ", ";
      s += n.whi > 240000 ? "+inf" : fmt(n.whi);
      s += "]";
      for (const SpanStopConstraint& c : n.span_stops) {
        s += (c.require ? " at(" : " not_at(") + fmt(c.time) + "=" +
             std::to_string(c.stop.v) + ")";
      }
      for (const EdgeConstraint& c : n.edges) {
        s += (c.require ? " req(" : " forb(") + std::to_string(c.a.v) + "->" +
             std::to_string(c.b.v) + ")";
      }
      return s;
    };

    while (!pq.empty() && processed < bnb_nodes) {
      if (bnb_seconds > 0) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start)
                .count() >= bnb_seconds) {
          std::cout << "Time budget reached.\n";
          break;
        }
      }
      BnbNode node = pq.top();
      pq.pop();
      processed += 1;
      auto node_start = std::chrono::steady_clock::now();

      // Effective window: the node's window sharpened by the must-times (the
      // tour contains [minS, maxS] and lasts at most UB).
      long wlo = node.wlo;
      long whi = node.whi;
      if (!node.must.empty()) {
        wlo = std::max(wlo, static_cast<long>(node.must.back()) - *ub);
        whi = std::min(whi, static_cast<long>(node.must.front()) + *ub);
      }
      std::vector<Step> node_steps = merged_steps;
      std::erase_if(node_steps, [&](const Step& s) {
        if (!s.is_flex && (s.origin.time.seconds <= wlo ||
                           s.destination.time.seconds > whi)) {
          return true;
        }
        for (const EdgeConstraint& ec : node.edges) {
          if (ec.require) {
            // b immediately follows a: no other leg leaves a or enters b.
            if ((s.origin.stop == ec.a && s.destination.stop != ec.b) ||
                (s.destination.stop == ec.b && s.origin.stop != ec.a)) {
              return true;
            }
          } else if (s.origin.stop == ec.a && s.destination.stop == ec.b) {
            return true;
          }
        }
        return false;
      });
      TarelEdgeIntermediateData node_data =
          ComputeTarelIntermediateData(node_steps);

      Prices warm{
          std::move(node.warm_lambda),
          std::nullopt,
          std::nullopt,
          node.span_stops
      };
      if (!node.must.empty()) {
        warm.spans.emplace(node.must);
        if (node.warm_pi.size() == node.must.size()) {
          warm.spans->Add(node.warm_pi);
        }
      }
      SweepSolveOptions node_opts{
          .ub = *ub,
          .max_iterations = node.depth == 0 && bnb_root_iters > 0
                                ? bnb_root_iters
                                : sweep_iters,
          .mu = mu,
          .initial_delta = delta,
          .patience = patience,
          .momentum = momentum,
          .eps = eps,
      };
      if (!bnb_no_coverage && wlo > std::numeric_limits<long>::min() / 4 &&
          whi < std::numeric_limits<long>::max() / 4) {
        // The tour start lies in (wlo, min(whi, first must-time)]: dense
        // coverage prices apply with that start bucket.
        node_opts.coverage = true;
        node_opts.cov_t1 = static_cast<int>(wlo + 1);
        node_opts.cov_t2 = static_cast<int>(
            node.must.empty()
                ? whi
                : std::min(whi, static_cast<long>(node.must.front()))
        );
        node_opts.cov_res = coverage_res;
      }
      SweepSolveResult r =
          RunSubgradientLp(state, node_data, node_opts, std::move(warm));
      int node_bound = r.infeasible ? *ub : std::max(node.bound, r.best_bound);
      auto node_end = std::chrono::steady_clock::now();
      int node_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        node_end - node_start
      )
                        .count();

      std::string decision;
      if (node_bound >= *ub) {
        pruned += 1;
        decision = r.infeasible ? "infeasible -> pruned" : "pruned";
      } else {
        std::vector<AnnotatedTarelEdge> annotated_best =
            BuildTarelEdgesWithCosts(node_data, r.best_prices);
        int grid_lo =
            static_cast<int>(std::max(wlo + 1, static_cast<long>(sched_t_min)));
        int grid_hi = static_cast<int>(
            std::min(whi, static_cast<long>(sched_t_max) + *ub)
        );
        std::optional<BunchPick> pick = FindBunchiestTime(
            node_data,
            r.best_prices,
            annotated_best,
            r.best_active,
            grid_lo,
            grid_hi,
            60,
            300,
            node.must
        );
        // Branch on the bunchiest time while the bound is improving and the
        // bunching signal is real; otherwise bisect at the mass median (the
        // ternary branch at the median is a window bisection). A candidate
        // branch time must keep clear of existing must-times.
        bool improved = node.bound == std::numeric_limits<int>::min() ||
                        node_bound > node.bound + 1;
        auto usable = [&](int t) {
          if (t <= grid_lo || t >= grid_hi) {
            return false;
          }
          for (int m : node.must) {
            if (std::abs(t - m) < 60) {
              return false;
            }
          }
          return true;
        };
        // Midpoint of the largest gap between constraint points, for when the
        // median lands too close to an existing must-time.
        auto largest_gap_mid = [&]() -> std::optional<int> {
          std::vector<int> pts = node.must;
          pts.push_back(grid_lo);
          pts.push_back(grid_hi);
          std::sort(pts.begin(), pts.end());
          std::optional<int> best_mid;
          int best_gap = 240;  // require a meaningful gap
          for (size_t i = 0; i + 1 < pts.size(); ++i) {
            int gap = pts[i + 1] - pts[i];
            int mid = pts[i] + gap / 2;
            if (gap > best_gap && usable(mid)) {
              best_gap = gap;
              best_mid = mid;
            }
          }
          return best_mid;
        };
        // Fractional successor pair (a, b) closest to mass 0.5 for edge
        // branching: either the tour uses leg a->b or it doesn't.
        std::optional<EdgeConstraint> edge_pick;
        double edge_frac = 0.0;
        if (bnb_strategy == "edge" || bnb_strategy == "winedge") {
          std::map<std::pair<int, int>, double> pair_mass;
          for (const auto& [aidx, amass] : r.best_active) {
            const AnnotatedTarelEdge& ae = annotated_best[aidx];
            pair_mass[{ae.edge.origin.stop.v, ae.edge.destination.stop.v}] +=
                amass;
          }
          double best_dist = 0.21;
          for (const auto& [pr, m] : pair_mass) {
            double d = std::abs(m - 0.5);
            if (m < 0.3 || m > 0.7 || d >= best_dist) {
              continue;
            }
            bool dup = false;
            for (const EdgeConstraint& ec : node.edges) {
              if (ec.a.v == pr.first && ec.b.v == pr.second) {
                dup = true;
              }
            }
            if (dup) {
              continue;
            }
            best_dist = d;
            edge_pick =
                EdgeConstraint{StopId{pr.first}, StopId{pr.second}, true};
            edge_frac = m;
          }
        }
        bool do_edge = edge_pick.has_value() &&
                       (bnb_strategy == "edge" ||
                        (bnb_strategy == "winedge" && !improved)) &&
                       node.depth < bnb_max_depth;

        std::optional<int> tstar_opt;
        std::string rule;
        if (pick.has_value() &&
            (bnb_strategy == "window" || bnb_strategy == "edge" ||
             bnb_strategy == "winedge")) {
          // Maximize the guaranteed window shrink across the children: split
          // at the midpoint of the largest gap between the window edges and
          // the must-cluster.
          if (std::optional<int> mid = largest_gap_mid()) {
            tstar_opt = mid;
            rule = "gapmid";
          } else if (usable(pick->t_median)) {
            tstar_opt = pick->t_median;
            rule = "bisect";
          } else if (usable(pick->t_max)) {
            tstar_opt = pick->t_max;
            rule = "mult " + std::to_string(pick->mult).substr(0, 4);
          }
        } else if (pick.has_value()) {
          bool want_mult;
          if (bnb_strategy == "bisect") {
            want_mult = false;
          } else if (bnb_strategy == "mult") {
            want_mult = pick->mult > 1.05;
          } else {
            want_mult = improved && pick->mult > 1.05;
          }
          if (want_mult && usable(pick->t_max)) {
            tstar_opt = pick->t_max;
            rule = "mult " + std::to_string(pick->mult).substr(0, 4);
          } else if (usable(pick->t_median)) {
            tstar_opt = pick->t_median;
            rule = "bisect";
          } else if (std::optional<int> mid = largest_gap_mid()) {
            tstar_opt = mid;
            rule = "gapmid";
          } else if (usable(pick->t_max)) {
            tstar_opt = pick->t_max;
            rule = "mult " + std::to_string(pick->mult).substr(0, 4);
          }
        }
        // Align parent span prices to child must-lists by time.
        std::map<int, double> pi_by_time;
        if (r.best_prices.spans.has_value()) {
          for (int i = 0; i < r.best_prices.spans->n(); ++i) {
            pi_by_time[r.best_prices.spans->time(i)] =
                r.best_prices.spans->pi(i);
          }
        }
        auto make_pi = [&](const std::vector<int>& times) {
          std::vector<double> pi;
          for (int t : times) {
            auto it = pi_by_time.find(t);
            pi.push_back(it == pi_by_time.end() ? 0.0 : it->second);
          }
          return pi;
        };

        if (do_edge) {
          decision = "branch edge " + std::to_string(edge_pick->a.v) + "->" +
                     std::to_string(edge_pick->b.v) +
                     " (x=" + std::to_string(edge_frac).substr(0, 4) + ")";
          std::vector<EdgeConstraint> e_req = node.edges;
          e_req.push_back(*edge_pick);
          std::vector<EdgeConstraint> e_forb = node.edges;
          e_forb.push_back(EdgeConstraint{edge_pick->a, edge_pick->b, false});
          pq.push(
              BnbNode{
                  node.must,
                  node.wlo,
                  node.whi,
                  node.span_stops,
                  std::move(e_req),
                  node_bound,
                  node.depth + 1,
                  r.best_prices.stop_time,
                  make_pi(node.must)
              }
          );
          pq.push(
              BnbNode{
                  node.must,
                  node.wlo,
                  node.whi,
                  node.span_stops,
                  std::move(e_forb),
                  node_bound,
                  node.depth + 1,
                  r.best_prices.stop_time,
                  make_pi(node.must)
              }
          );
        } else if (!tstar_opt.has_value() || node.depth >= bnb_max_depth) {
          leaves.push_back(BnbLeaf{node_bound, node.depth});
          decision =
              "leaf" + (pick.has_value()
                            ? " (max mult " +
                                  std::to_string(pick->mult).substr(0, 4) + ")"
                            : "");
        } else {
          int tstar = *tstar_opt;
          decision = "branch on " + fmt(tstar) + " (" + rule + ")";
          // Which-stop mixture at t*: mass by destination stop over active
          // argmin candidates definitely spanning t*.
          std::map<int, double> mass_by_stop;
          double span_mass = 0.0;
          for (const auto& [aidx, amass] : r.best_active) {
            const AnnotatedTarelEdge& ae = annotated_best[aidx];
            std::vector<EdgeCandidate> cands = EnumerateEdgeCandidates(
                node_data, r.best_prices, ae.edge.origin, ae.edge.destination
            );
            if (cands.empty()) {
              continue;
            }
            const EdgeCandidate* am = &cands[0];
            for (const EdgeCandidate& c : cands) {
              if (c.value < am->value) {
                am = &c;
              }
            }
            if (am->leg_kind == EdgeCandidate::LegKind::kFloating) {
              continue;
            }
            if (am->leg_a <= tstar && tstar < am->leg_d) {
              mass_by_stop[ae.edge.destination.stop.v] += amass;
              span_mass += amass;
            }
          }
          StopId top_stop{-1};
          double top_mass = 0.0;
          for (const auto& [stop_v, m] : mass_by_stop) {
            if (m > top_mass) {
              top_mass = m;
              top_stop = StopId{stop_v};
            }
          }
          bool stop_split =
              top_mass > 0.3 && span_mass - top_mass > 0.3 && top_stop.v >= 0;

          // Child A: the tour spans t* (split by the spanning leg's
          // destination stop when the relaxation mixes several).
          std::vector<int> must_a = node.must;
          must_a.insert(
              std::lower_bound(must_a.begin(), must_a.end(), tstar), tstar
          );
          if (stop_split) {
            decision += " stop-split @" + state.StopName(top_stop);
            std::vector<SpanStopConstraint> ss_req = node.span_stops;
            ss_req.push_back(SpanStopConstraint{tstar, top_stop, true});
            std::vector<SpanStopConstraint> ss_for = node.span_stops;
            ss_for.push_back(SpanStopConstraint{tstar, top_stop, false});
            pq.push(
                BnbNode{
                    must_a,
                    node.wlo,
                    node.whi,
                    std::move(ss_req),
                    node.edges,
                    node_bound,
                    node.depth + 1,
                    r.best_prices.stop_time,
                    make_pi(must_a)
                }
            );
            pq.push(
                BnbNode{
                    must_a,
                    node.wlo,
                    node.whi,
                    std::move(ss_for),
                    node.edges,
                    node_bound,
                    node.depth + 1,
                    r.best_prices.stop_time,
                    make_pi(must_a)
                }
            );
          } else {
            pq.push(
                BnbNode{
                    must_a,
                    node.wlo,
                    node.whi,
                    node.span_stops,
                    node.edges,
                    node_bound,
                    node.depth + 1,
                    r.best_prices.stop_time,
                    make_pi(must_a)
                }
            );
          }
          // Child B1: the tour lies entirely at or before t* (only possible
          // when every must-time is before t*).
          if (node.must.empty() || node.must.back() < tstar) {
            pq.push(
                BnbNode{
                    node.must,
                    node.wlo,
                    std::min(node.whi, static_cast<long>(tstar)),
                    node.span_stops,
                    node.edges,
                    node_bound,
                    node.depth + 1,
                    r.best_prices.stop_time,
                    make_pi(node.must)
                }
            );
          }
          // Child B2: the tour lies entirely after t* (only possible when
          // every must-time is after t*).
          if (node.must.empty() || node.must.front() > tstar) {
            pq.push(
                BnbNode{
                    node.must,
                    std::max(node.wlo, static_cast<long>(tstar)),
                    node.whi,
                    node.span_stops,
                    node.edges,
                    node_bound,
                    node.depth + 1,
                    r.best_prices.stop_time,
                    make_pi(node.must)
                }
            );
          }
        }
      }
      int lb_so_far = *ub;
      for (const BnbLeaf& leaf : leaves) {
        lb_so_far = std::min(lb_so_far, leaf.bound);
      }
      if (!pq.empty()) {
        lb_so_far = std::min(lb_so_far, pq.top().bound);
      }
      auto now = std::chrono::steady_clock::now();
      long elapsed_s =
          std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
      std::cout << std::string(2 * node.depth, ' ') << describe(node)
                << ": bound "
                << (node_bound == std::numeric_limits<int>::min()
                        ? std::string("-")
                        : std::to_string(node_bound))
                << " (" << r.iterations_run << " iters, "
                << FormatDuration(node_ms) << "): " << decision << " [lb "
                << lb_so_far << " @ " << elapsed_s << "s]" << std::endl;
    }

    auto end = std::chrono::steady_clock::now();
    int total_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    int global_lb = *ub;
    for (const BnbLeaf& leaf : leaves) {
      global_lb = std::min(global_lb, leaf.bound);
    }
    int frontier = 0;
    while (!pq.empty()) {
      global_lb = std::min(global_lb, pq.top().bound);
      frontier += 1;
      pq.pop();
    }
    std::cout << "\nSpan B&B: " << processed << " nodes expanded, " << pruned
              << " pruned, " << leaves.size() << " leaves, " << frontier
              << " unexpanded frontier nodes\n";
    if (global_lb >= *ub && leaves.empty() && frontier == 0) {
      std::cout << "All branches pruned: UB " << TimeSinceServiceStart{*ub}
                << " (" << *ub << " s) is optimal.\n";
    } else {
      std::cout << "Global lower bound: " << TimeSinceServiceStart{global_lb}
                << " (" << global_lb << " s), gap " << std::fixed
                << std::setprecision(2) << 100.0 * (*ub - global_lb) / *ub
                << "%\n";
    }
    std::cout << "Total time: " << FormatDuration(total_ms) << "\n";
    return 0;
  }

  TarelEdgeIntermediateData data =
      ComputeTarelIntermediateData(std::move(merged_steps));
  {
    int num_states = 0;
    int num_flex_states = 0;
    int num_mixed_states = 0;
    long discarded_scheduled_arrivals = 0;
    for (const auto& [s, arrivals] : data.arrival_times_to) {
      num_states += 1;
      if (arrivals.has_flex) {
        num_flex_states += 1;
        if (!arrivals.times.empty()) {
          num_mixed_states += 1;
          discarded_scheduled_arrivals += arrivals.times.size();
        }
      }
    }
    std::cout << "Tarel states: " << num_states << " (" << num_flex_states
              << " with flex arrivals, of which " << num_mixed_states
              << " also have scheduled arrivals; "
              << discarded_scheduled_arrivals
              << " scheduled arrival times unpriced due to has_flex)\n";
  }

  if (coverage) {
    // Coverage-price single run. With --window-start the starts are pinned to
    // [window_start, window_start + bucket_width]; otherwise the "bucket" is
    // the whole range of possible starts (no candidate clipping).
    int t1;
    int t2;
    if (window_start.has_value()) {
      t1 = *window_start;
      t2 = t1 + std::max(0, *window_len - *ub);
    } else {
      if (sched_t_min > sched_t_max) {
        std::cerr << "No scheduled steps; cannot run coverage\n";
        return 1;
      }
      t1 = sched_t_min;
      t2 = sched_t_max;
      std::cout << "Whole-range coverage: starts in ["
                << TimeSinceServiceStart{t1} << ", "
                << TimeSinceServiceStart{t2} << "]" << std::endl;
    }
    SweepSolveOptions cov_opts{
        .ub = *ub,
        .max_iterations = max_iterations,
        .mu = mu,
        .initial_delta = delta,
        .patience = patience,
        .momentum = momentum,
        .eps = eps,
        .coverage = true,
        .cov_t1 = t1,
        .cov_t2 = t2,
        .cov_res = coverage_res,
        .log = &std::cout,
    };
    SweepSolveResult r = RunSubgradientLp(state, data, cov_opts, {});
    if (r.infeasible) {
      std::cout << "Infeasible window (no bound)\n";
      return 0;
    }
    std::cout << "\nCertifying best prices with Concorde..." << std::endl;
    std::vector<AnnotatedTarelEdge> annotated_best =
        BuildTarelEdgesWithCosts(data, r.best_prices);
    std::vector<size_t> tour_idx;
    std::optional<int> cert = SolveUnmergedTspBound(
        annotated_best, state.required, state.boundary, nullptr, &tour_idx
    );
    int final_bound = r.best_bound;

    // Diagnostic: the certified tour's edges in tour order with their argmin
    // spans, chain jumps, and the resulting coverage profile.
    if (cert.has_value() && r.best_prices.coverage.has_value()) {
      const CoveragePrices& cov = *r.best_prices.coverage;
      auto fmt = [](int seconds) {
        return TimeSinceServiceStart{seconds}.ToString();
      };
      auto name = [&](StopId stop) {
        std::string n = state.StopName(stop);
        return n.size() > 20 ? n.substr(0, 20) : n;
      };
      CoverageSpan max_span;
      double cov_c = cov.MaxAdmissibleSpan(&max_span);
      std::cout << "\nCoverage constant C = " << std::setprecision(1)
                << std::fixed << cov_c << " on span ["
                << fmt(cov.GridTime(max_span.k1)) << ", "
                << fmt(cov.GridTime(max_span.k2)) << "), lb " << cov.lb()
                << "\n";
      std::vector<double> profile(cov.n(), 0.0);
      std::cout << "Certified tour edges (tour order):\n";
      std::optional<TimeSinceServiceStart> prev_arr;
      for (size_t i = 0; i < tour_idx.size(); ++i) {
        const AnnotatedTarelEdge& a = annotated_best[tour_idx[i]];
        std::vector<EdgeCandidate> cands = EnumerateEdgeCandidates(
            data, r.best_prices, a.edge.origin, a.edge.destination
        );
        const EdgeCandidate* am = &cands[0];
        for (const EdgeCandidate& c : cands) {
          if (c.value < am->value) {
            am = &c;
          }
        }
        for (int k = am->span.k1; k < am->span.k2; ++k) {
          profile[k] += 1.0;
        }
        std::cout << "  " << std::setw(20) << name(a.edge.origin.stop) << " -> "
                  << std::setw(20) << name(a.edge.destination.stop) << " w "
                  << std::setw(6) << a.edge.weight << " span ["
                  << fmt(cov.GridTime(am->span.k1)) << ", "
                  << fmt(cov.GridTime(am->span.k2)) << ") dep@"
                  << (am->origin_arrival.has_value()
                          ? fmt(am->origin_arrival->seconds)
                          : "flex")
                  << " arr@"
                  << (am->destination_arrival.has_value()
                          ? fmt(am->destination_arrival->seconds)
                          : "flex");
        if (prev_arr.has_value() && am->origin_arrival.has_value()) {
          std::cout << " chain_jump "
                    << (am->origin_arrival->seconds - prev_arr->seconds);
        }
        prev_arr = am->destination_arrival;
        std::cout << "\n";
      }
      // Coverage profile: report holes inside the max span and multiply
      // covered runs anywhere.
      auto report_runs = [&](auto pred, const char* label) {
        int run_start = -1;
        for (int k = 0; k <= cov.n(); ++k) {
          bool in = k < cov.n() && pred(k);
          if (in && run_start < 0) {
            run_start = k;
          } else if (!in && run_start >= 0) {
            std::cout << "  " << label << " [" << fmt(cov.GridTime(run_start))
                      << ", " << fmt(cov.GridTime(k)) << ")\n";
            run_start = -1;
          }
        }
      };
      std::cout << "Coverage profile anomalies:\n";
      report_runs(
          [&](int k) {
            return k >= max_span.k1 && k < max_span.k2 && profile[k] < 0.5;
          },
          "hole"
      );
      report_runs([&](int k) { return profile[k] > 1.5; }, "multi");
    }
    if (cert.has_value()) {
      double cov_c = r.best_prices.coverage.has_value()
                         ? r.best_prices.coverage->MaxAdmissibleSpan(nullptr)
                         : 0.0;
      int certified = static_cast<int>(std::ceil(*cert - cov_c - 1e-6));
      std::cout << "Certified TSP bound at best prices: " << certified
                << " s\n";
      final_bound = std::max(final_bound, certified);
    }
    auto cov_end = std::chrono::steady_clock::now();
    int cov_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(cov_end - start)
            .count();
    if (window_start.has_value() && window_len.has_value() &&
        *window_len < *ub && final_bound > *window_len) {
      std::cout << "\nNOTE: the bound (" << final_bound
                << ") exceeds the window's implied duration cap ("
                << *window_len
                << "), so NO tour exists in this window at all; the bound is "
                   "vacuous for the unrestricted problem.\n";
    }
    std::cout << "\nBest bound with coverage prices: "
              << TimeSinceServiceStart{final_bound} << " (" << final_bound
              << " s)\n";
    std::cout << "Upper bound:                     "
              << TimeSinceServiceStart{*ub} << " (" << *ub << " s)\n";
    std::cout << std::fixed << std::setprecision(2)
              << "Gap: " << 100.0 * (*ub - final_bound) / *ub << "%\n";
    std::cout << "Total time: " << FormatDuration(cov_ms) << "\n";
    return 0;
  }

  Prices costs;
  Prices best_costs;
  const double initial_delta = delta;
  int initial_bound = std::numeric_limits<int>::min();
  int best_bound = std::numeric_limits<int>::min();
  int iterations_without_improvement = 0;
  std::optional<LpTarelBound> lp;

  // CFM momentum direction: an exponentially decayed sum of past subgradients,
  // with the decay chosen each iteration to keep it an ascent direction.
  std::unordered_map<CostKey, double> direction;

  for (int iter = 0; iter < max_iterations; ++iter) {
    auto iter_start = std::chrono::steady_clock::now();
    std::vector<AnnotatedTarelEdge> annotated =
        BuildTarelEdgesWithCosts(data, costs);

    std::optional<IterationResult> result;
    if (use_concorde) {
      result = SolveIterationConcorde(
          state, annotated, verbose_tsp ? &std::cerr : nullptr
      );
    } else {
      if (!lp.has_value()) {
        // The tarel edge set doesn't depend on the costs, so the LP structure
        // built from this iteration's edges is valid for the whole run.
        lp.emplace(annotated, state.required, state.boundary);
        if (!lp->ok()) {
          std::cerr << "Failed to build tarel LP, stopping\n";
          break;
        }
        std::cout << "Tarel LP: " << lp->num_vertices() << " vertices, "
                  << annotated.size() << " edges, " << lp->num_cuts()
                  << " seeded cuts" << std::endl;
      }
      std::optional<LpTarelBound::LpResult> lp_result = lp->Solve(annotated);
      if (debug_mip) {
        std::cout << "LP bound: "
                  << (lp_result.has_value()
                          ? std::to_string(lp_result->lower_bound)
                          : "none")
                  << "\n";
        std::optional<int> mip = lp->DebugSolveMip();
        std::cout << "MIP bound: "
                  << (mip.has_value() ? std::to_string(*mip) : "none") << "\n";
        std::optional<std::vector<TarelEdge>> mapped =
            BuildGroupMappedEdges(annotated, state.required);
        TspGraphData dbg_graph = MakeTspGraphEdges(*mapped, state.boundary);
        std::optional<TspTourResult> tsp = SolveTspAndExtractTour(
            *mapped, dbg_graph, state.boundary, std::nullopt, nullptr, nullptr
        );
        if (tsp.has_value()) {
          long tour_sum = 0;
          for (const TarelEdge& e : tsp->tour_edges) {
            tour_sum += e.weight;
          }
          std::cout << "Concorde bound (same graph): " << tsp->optimal_value
                    << ", sum of extracted tour edge weights: " << tour_sum
                    << " over " << tsp->tour_edges.size() << " edges"
                    << std::endl;
          // Count parallel (u, v) pairs in the TSP edge list; the doubled
          // Concorde transform keeps only one weight per pair.
          std::map<std::pair<int, int>, int> pair_count;
          for (const WeightedEdge& e : dbg_graph.tsp_edges) {
            pair_count[{e.origin.v, e.destination.v}] += 1;
          }
          int num_parallel = 0;
          for (const auto& [key, count] : pair_count) {
            if (count > 1) {
              num_parallel += 1;
            }
          }
          std::cout << "Parallel (u,v) pairs in tsp_edges: " << num_parallel
                    << std::endl;
          lp->DebugCheckTour(*mapped, tsp->tour_edges);
        } else {
          std::cout << "Concorde bound (same graph): none" << std::endl;
        }
        return 0;
      }
      if (lp_result.has_value()) {
        IterationResult r;
        r.lower_bound = lp_result->lower_bound;
        for (const auto& [idx, x] : lp_result->active) {
          r.active.push_back({&annotated[idx], x});
        }
        r.engine_stats = "cuts " + std::to_string(lp->num_cuts()) + " (+" +
                         std::to_string(lp_result->separation_rounds) +
                         " rounds)";
        result = std::move(r);
      }
    }
    auto iter_end = std::chrono::steady_clock::now();
    int iter_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      iter_end - iter_start
    )
                      .count();
    if (!result.has_value()) {
      std::cerr << "Iteration " << iter
                << ": tarel relaxation infeasible / no bound, stopping\n";
      break;
    }

    if (iter == 0) {
      initial_bound = result->lower_bound;
    }
    if (result->lower_bound > best_bound) {
      best_bound = result->lower_bound;
      best_costs = costs;
      iterations_without_improvement = 0;
      delta = std::min(delta * 1.2, initial_delta);
    } else {
      iterations_without_improvement += 1;
      if (iterations_without_improvement >= patience) {
        delta = std::max(delta / 2.0, 1.0);
        iterations_without_improvement = 0;
      }
    }
    // Anneal the spread width along with the target level: a wide spread
    // converges quickly to within ~eps of the dual optimum but then blurs the
    // direction; sharpen it as the target closes in.
    double effective_eps = std::min(eps, std::max(delta, 1.0));

    // Subgradient: each optimal tour edge contributes +1 mass over the
    // (stop, time) pairs its near-minimal candidates arrive at, and -1 mass
    // over the pairs they depart from. Spreading over all candidates within
    // eps of the minimum (an epsilon-subgradient) avoids ping-ponging between
    // near-tied candidates on the same edge.
    std::unordered_map<CostKey, double> gradient;
    long total_candidates = 0;
    int max_candidates = 0;
    for (const auto& [a, mass] : result->active) {
      std::vector<EdgeCandidate> candidates = EnumerateEdgeCandidates(
          data, costs, a->edge.origin, a->edge.destination
      );
      double min_value = std::numeric_limits<double>::infinity();
      for (const EdgeCandidate& c : candidates) {
        min_value = std::min(min_value, c.value);
      }
      std::erase_if(candidates, [&](const EdgeCandidate& c) {
        return c.value > min_value + effective_eps;
      });
      total_candidates += candidates.size();
      max_candidates = std::max(max_candidates, (int)candidates.size());
      double w = mass / candidates.size();
      for (const EdgeCandidate& c : candidates) {
        if (c.destination_arrival.has_value()) {
          gradient[{a->edge.destination.stop, *c.destination_arrival}] += w;
        }
        if (c.origin_arrival.has_value()) {
          gradient[{a->edge.origin.stop, *c.origin_arrival}] -= w;
        }
      }
    }
    std::erase_if(gradient, [](const auto& kv) {
      return std::abs(kv.second) < 1e-9;
    });
    double norm_sq = 0.0;
    int num_inconsistent = 0;
    for (const auto& [key, g] : gradient) {
      norm_sq += g * g;
      num_inconsistent += 1;
    }
    double avg_candidates =
        result->active.empty()
            ? 0.0
            : static_cast<double>(total_candidates) / result->active.size();

    std::cout << "Iteration " << std::setw(4) << iter << ": bound "
              << TimeSinceServiceStart{result->lower_bound} << " ("
              << result->lower_bound << " s), best "
              << TimeSinceServiceStart{best_bound} << ", |g|^2 "
              << std::setprecision(3) << norm_sq << " (" << num_inconsistent
              << " keys), cand avg " << std::setprecision(3) << avg_candidates
              << " max " << max_candidates << ", delta " << delta << ", eps "
              << effective_eps << ", " << result->engine_stats << ", "
              << FormatDuration(iter_ms) << std::endl;

    if (norm_sq == 0.0) {
      std::cout << "Subgradient is zero (time-consistent relaxed tour); "
                   "bound is optimal for this relaxation.\n";
      break;
    }
    if (ub.has_value() && result->lower_bound >= *ub) {
      std::cout << "Bound reached the upper bound.\n";
      break;
    }

    // CFM momentum: mix the previous direction in when it disagrees with the
    // new subgradient, damping oscillation between alternating optimal tours.
    double direction_norm_sq = 0.0;
    for (const auto& [key, d] : direction) {
      direction_norm_sq += d * d;
    }
    double beta = 0.0;
    if (momentum > 0.0 && direction_norm_sq > 0.0) {
      double dot = 0.0;
      for (const auto& [key, g] : gradient) {
        auto it = direction.find(key);
        if (it != direction.end()) {
          dot += it->second * g;
        }
      }
      if (dot < 0.0) {
        beta = -momentum * dot / direction_norm_sq;
      }
    }
    if (beta == 0.0) {
      direction.clear();
    } else {
      for (auto& [key, d] : direction) {
        d *= beta;
      }
    }
    for (const auto& [key, g] : gradient) {
      direction[key] += g;
    }
    std::erase_if(direction, [](const auto& kv) {
      return std::abs(kv.second) < 1e-9;
    });
    double step_norm_sq = 0.0;
    for (const auto& [key, d] : direction) {
      step_norm_sq += d * d;
    }
    if (step_norm_sq == 0.0) {
      // Momentum cancelled the subgradient exactly; fall back to the raw
      // subgradient.
      for (const auto& [key, g] : gradient) {
        direction[key] = g;
      }
      step_norm_sq = norm_sq;
    }

    // Step toward an adaptive target level a bit above the best bound so far
    // (capped at the upper bound if we have one).
    double target = best_bound + delta;
    if (ub.has_value()) {
      target = std::min(target, static_cast<double>(*ub));
    }
    double alpha = mu * (target - result->lower_bound) / step_norm_sq;
    if (alpha <= 0.0) {
      alpha = mu / step_norm_sq;
    }
    for (const auto& [key, d] : direction) {
      costs.stop_time[key] += alpha * d;
    }
  }

  // In LP mode the per-iteration bounds are LP-strength; solve the TSP
  // exactly once at the best costs to get the full TSP-strength bound.
  std::optional<int> certified_bound;
  if (!use_concorde && best_bound > std::numeric_limits<int>::min()) {
    std::cout << "\nCertifying best costs with Concorde..." << std::endl;
    auto cert_start = std::chrono::steady_clock::now();
    std::vector<AnnotatedTarelEdge> annotated_best =
        BuildTarelEdgesWithCosts(data, best_costs);
    std::optional<int> cert = SolveUnmergedTspBound(
        annotated_best, state.required, state.boundary, nullptr
    );
    auto cert_end = std::chrono::steady_clock::now();
    int cert_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      cert_end - cert_start
    )
                      .count();
    if (cert.has_value()) {
      certified_bound = *cert;
      std::cout << "Certified TSP bound at best costs: "
                << TimeSinceServiceStart{*cert} << " (" << *cert << " s) in "
                << FormatDuration(cert_ms) << std::endl;
      best_bound = std::max(best_bound, *cert);
    } else {
      std::cout << "Certification solve failed; keeping LP bound." << std::endl;
    }
  }

  if (analyze_gap && hk_result.has_value() &&
      best_bound > std::numeric_limits<int>::min()) {
    AnalyzeGap(
        state,
        data,
        completed,
        best_costs,
        hk_result->best_tour,
        hk_result->best_val,
        certified_bound.value_or(best_bound)
    );
  }

  auto end = std::chrono::steady_clock::now();
  int total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  std::cout << "\nInitial (lambda=0) bound: "
            << TimeSinceServiceStart{initial_bound} << " (" << initial_bound
            << " s)\n";
  std::cout << "Best Lagrangian bound:    " << TimeSinceServiceStart{best_bound}
            << " (" << best_bound << " s)\n";
  if (ub.has_value()) {
    std::cout << "Upper bound:              " << TimeSinceServiceStart{*ub}
              << " (" << *ub << " s)\n";
    double gap0 = 100.0 * (*ub - initial_bound) / *ub;
    double gap = 100.0 * (*ub - best_bound) / *ub;
    std::cout << std::fixed << std::setprecision(2) << "Gap: " << gap0
              << "% -> " << gap << "%\n";
  }
  std::cout << "Total time: " << FormatDuration(total_ms) << "\n";

  return 0;
}
