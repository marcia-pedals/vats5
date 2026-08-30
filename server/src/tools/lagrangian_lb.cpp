// Computes an improved lower bound for the tour problem by strengthening the
// tarel relaxation with Lagrangian costs. Instead of subgradient iteration,
// the Lagrangian multipliers are folded directly into a single LP
// (candidate-split formulation + consistency rows + lazily separated subtour
// cuts) solved with HiGHS, then certified with one Concorde solve at the
// extracted multipliers. A few cheap subgradient iterations warm-start the
// LP's column seed and cut pool.

#include <CLI/CLI.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Highs.h"
#include "algorithm/union_find.h"
#include "solver/concorde.h"
#include "solver/held_karp_dp.h"
#include "solver/tarel_graph.h"

using namespace vats5;

namespace {

// A (stop, scheduled arrival time) pair that a Lagrangian multiplier attaches
// to. Keys are interned to dense indices so candidates can carry plain ints
// and lambda vectors can be dense arrays.
using LambdaKey = std::pair<StopId, TimeSinceServiceStart>;

struct KeyInterner {
  std::unordered_map<LambdaKey, int> index;
  std::vector<LambdaKey> keys;

  int Intern(const LambdaKey& k) {
    auto [it, inserted] = index.try_emplace(k, static_cast<int>(keys.size()));
    if (inserted) {
      keys.push_back(k);
    }
    return it->second;
  }

  int size() const { return static_cast<int>(keys.size()); }
};

// One candidate of a tarel edge: an (arrival time, outgoing step) evaluation
// whose min over the edge's candidates is the edge weight. `dep`/`arr` are
// interned key indices whose lambdas modify this candidate's value (-1 = the
// flex validity rules forbid modification on that side).
struct Candidate {
  int value;
  int dep = -1;
  int arr = -1;
};

struct CandidateEdge {
  TarelState origin;
  TarelState destination;
  std::vector<Candidate> candidates;
  int min_value = std::numeric_limits<int>::max();
};

// Lambda-modified candidate value in the natural convention:
// value + lambda(arrival side) - lambda(departure side).
// An empty lambda vector means lambda = 0.
double ModifiedValue(const Candidate& c, const std::vector<double>& lambda) {
  double m = c.value;
  if (!lambda.empty()) {
    if (c.arr >= 0) m += lambda[c.arr];
    if (c.dep >= 0) m -= lambda[c.dep];
  }
  return m;
}

// Enumerates the deduped candidates of every tarel edge, mirroring
// BuildTarelEdgesFromIntermediateData's weight computation but keeping every
// candidate (with costs the first step per arrival time is not always best).
std::vector<CandidateEdge> EnumerateCandidateEdges(
    const TarelEdgeIntermediateData& data, KeyInterner& keys
) {
  std::vector<CandidateEdge> result;
  for (const auto& [origin, arrivals] : data.arrival_times_to) {
    auto it = data.steps_from.find(origin.stop);
    if (it == data.steps_from.end()) {
      continue;
    }
    for (const auto& [dest, steps] : it->second) {
      bool dest_has_flex = data.arrival_times_to.at(dest).has_flex;

      CandidateEdge edge{.origin = origin, .destination = dest};
      std::unordered_map<int64_t, size_t> dedup;
      auto add = [&](int dep, int arr, int value) {
        int64_t key = (static_cast<int64_t>(dep + 1) << 32) |
                      static_cast<uint32_t>(arr + 1);
        auto [dit, inserted] = dedup.try_emplace(key, edge.candidates.size());
        if (inserted) {
          edge.candidates.push_back(Candidate{value, dep, arr});
        } else if (value < edge.candidates[dit->second].value) {
          edge.candidates[dit->second].value = value;
        }
        if (value < edge.min_value) {
          edge.min_value = value;
        }
      };

      if (arrivals.has_flex) {
        // Arrival time at the origin is unknown: no departure-side lambda,
        // value is the bare step duration.
        for (const Step& step : steps) {
          int arr = -1;
          if (!dest_has_flex && !step.is_flex) {
            arr = keys.Intern(
                LambdaKey{step.destination.stop, step.destination.time}
            );
          }
          add(-1, arr, step.DurationSeconds());
        }
      } else {
        size_t first_scheduled = 0;
        std::optional<int> flex_duration;
        if (!steps.empty() && steps[0].is_flex) {
          flex_duration = steps[0].FlexDurationSeconds();
          first_scheduled = 1;
        }
        size_t step_idx = first_scheduled;
        for (const TimeSinceServiceStart t : arrivals.times) {
          int dep = keys.Intern(LambdaKey{origin.stop, t});
          if (flex_duration.has_value()) {
            add(dep, -1, *flex_duration);
          }
          while (step_idx < steps.size() && steps[step_idx].origin.time < t) {
            step_idx += 1;
          }
          if (dest_has_flex) {
            // All these candidates share (dep, none); only the min value
            // matters, and by minimality the first departure after t arrives
            // earliest.
            if (step_idx < steps.size()) {
              add(dep,
                  -1,
                  steps[step_idx].destination.time.seconds - t.seconds);
            }
          } else {
            for (size_t si = step_idx; si < steps.size(); ++si) {
              const Step& step = steps[si];
              int arr = keys.Intern(
                  LambdaKey{step.destination.stop, step.destination.time}
              );
              add(dep, arr, step.destination.time.seconds - t.seconds);
            }
          }
        }
      }

      if (!edge.candidates.empty()) {
        result.push_back(std::move(edge));
      }
    }
  }
  return result;
}

// Cross-check: at lambda=0 the per-edge min candidate value must equal the
// existing tarel edge weight for every edge.
void CheckCandidatesAgainstTarelEdges(
    const std::vector<CandidateEdge>& edges,
    const TarelEdgeIntermediateData& data
) {
  std::vector<TarelEdge> reference = BuildTarelEdgesFromIntermediateData(data);
  std::map<std::pair<TarelState, TarelState>, int> reference_weight;
  for (const TarelEdge& e : reference) {
    reference_weight[{e.origin, e.destination}] = e.weight;
  }
  if (reference.size() != edges.size()) {
    throw std::runtime_error(
        "Candidate enumeration produced " + std::to_string(edges.size()) +
        " edges but BuildTarelEdgesFromIntermediateData produced " +
        std::to_string(reference.size())
    );
  }
  for (const CandidateEdge& e : edges) {
    auto it = reference_weight.find({e.origin, e.destination});
    if (it == reference_weight.end()) {
      throw std::runtime_error("Candidate edge missing from reference edges");
    }
    if (it->second != e.min_value) {
      throw std::runtime_error(
          "Candidate min value " + std::to_string(e.min_value) +
          " != reference weight " + std::to_string(it->second)
      );
    }
  }
}

constexpr double kFlowEps = 1e-6;
constexpr double kPriceEps = 1e-6;

// Small BFS-augmenting max-flow with early termination, used for exact
// subtour-cut separation on the aggregate LP flow.
class MaxFlow {
 public:
  explicit MaxFlow(int n) : adj_(n) {}

  void AddArc(int u, int v, double cap) {
    adj_[u].push_back({v, cap, static_cast<int>(adj_[v].size())});
    adj_[v].push_back({u, 0.0, static_cast<int>(adj_[u].size()) - 1});
  }

  // Pushes flow from s to t until it reaches `limit` or no augmenting path
  // remains. Returns the flow found; if it is < limit, `reachable` is set to
  // the residual-reachable set from s (a min cut of that value).
  double Solve(int s, int t, double limit, std::vector<char>* reachable) {
    double total = 0.0;
    while (total < limit) {
      std::vector<int> prev_arc(adj_.size(), -1);
      std::vector<int> prev_node(adj_.size(), -1);
      std::vector<char> visited(adj_.size(), 0);
      std::vector<int> queue{s};
      visited[s] = 1;
      for (size_t qi = 0; qi < queue.size() && !visited[t]; ++qi) {
        int u = queue[qi];
        for (size_t a = 0; a < adj_[u].size(); ++a) {
          const Arc& arc = adj_[u][a];
          if (!visited[arc.to] && arc.cap > 1e-9) {
            visited[arc.to] = 1;
            prev_node[arc.to] = u;
            prev_arc[arc.to] = static_cast<int>(a);
            queue.push_back(arc.to);
          }
        }
      }
      if (!visited[t]) {
        if (reachable != nullptr) {
          *reachable = visited;
        }
        return total;
      }
      double push = limit - total;
      for (int v = t; v != s; v = prev_node[v]) {
        push = std::min(push, adj_[prev_node[v]][prev_arc[v]].cap);
      }
      for (int v = t; v != s; v = prev_node[v]) {
        Arc& arc = adj_[prev_node[v]][prev_arc[v]];
        arc.cap -= push;
        adj_[arc.to][arc.rev].cap += push;
      }
      total += push;
    }
    return total;
  }

 private:
  struct Arc {
    int to;
    double cap;
    int rev;
  };
  std::vector<std::vector<Arc>> adj_;
};

// Subtour-cut separation on an aggregate flow: at several support thresholds,
// take the connected components and collect the out-cut of every component
// whose actual out-flow is below 1; if that finds nothing, run exact min-cut
// separation (a subtour cut is violated iff some vertex pair has max-flow < 1,
// checked root->t and t->root for every t). Returns membership vectors of
// violated cuts not already in `existing`.
std::vector<std::vector<char>> FindViolatedSubtourCuts(
    int num_vertices,
    const std::vector<std::pair<int, int>>& col_arc,
    const std::vector<double>& col_value,
    const std::set<std::vector<char>>& existing
) {
  std::vector<double> flow(
      static_cast<size_t>(num_vertices) * num_vertices, 0.0
  );
  for (size_t c = 0; c < col_arc.size(); ++c) {
    flow
        [static_cast<size_t>(col_arc[c].first) * num_vertices +
         col_arc[c].second] += col_value[c];
  }

  std::vector<std::vector<char>> found;
  std::set<std::vector<char>> seen;
  auto take = [&](std::vector<char> in_S) {
    if (!existing.contains(in_S) && !seen.contains(in_S)) {
      seen.insert(in_S);
      found.push_back(std::move(in_S));
    }
  };

  for (double threshold : {0.999, 0.9, 0.75, 0.5, 0.25, 0.1, kFlowEps}) {
    UnionFind uf(num_vertices);
    for (int u = 0; u < num_vertices; ++u) {
      for (int v = 0; v < num_vertices; ++v) {
        if (flow[static_cast<size_t>(u) * num_vertices + v] > threshold) {
          uf.Unite(u, v);
        }
      }
    }
    std::unordered_map<int, std::vector<int>> components;
    for (int v = 0; v < num_vertices; ++v) {
      components[uf.Find(v)].push_back(v);
    }
    if (components.size() <= 1) {
      continue;
    }
    for (const auto& [root, vertices] : components) {
      std::vector<char> in_S(num_vertices, 0);
      for (int v : vertices) {
        in_S[v] = 1;
      }
      double out_flow = 0.0;
      for (int u = 0; u < num_vertices; ++u) {
        if (!in_S[u]) continue;
        for (int v = 0; v < num_vertices; ++v) {
          if (!in_S[v]) {
            out_flow += flow[static_cast<size_t>(u) * num_vertices + v];
          }
        }
      }
      if (out_flow < 1.0 - 1e-6) {
        take(std::move(in_S));
      }
    }
  }
  if (!found.empty()) {
    return found;
  }

  std::vector<std::tuple<int, int, double>> arcs;
  for (int u = 0; u < num_vertices; ++u) {
    for (int v = 0; v < num_vertices; ++v) {
      double f = flow[static_cast<size_t>(u) * num_vertices + v];
      if (f > 1e-9) {
        arcs.push_back({u, v, f});
      }
    }
  }
  constexpr int kRoot = 0;
  for (int t = 0; t < num_vertices; ++t) {
    if (t == kRoot) continue;
    for (const auto& [s, d] : {std::pair{kRoot, t}, std::pair{t, kRoot}}) {
      MaxFlow mf(num_vertices);
      for (const auto& [u, v, f] : arcs) {
        mf.AddArc(u, v, f);
      }
      std::vector<char> in_S;
      double f = mf.Solve(s, d, 1.0 - 1e-6, &in_S);
      if (f < 1.0 - 1e-6) {
        take(std::move(in_S));
      }
    }
  }
  return found;
}

void CheckHighsOptimal(Highs& highs, const std::string& what) {
  if (highs.run() == HighsStatus::kError) {
    throw std::runtime_error("HiGHS run failed (" + what + ")");
  }
  if (highs.getModelStatus() != HighsModelStatus::kOptimal) {
    throw std::runtime_error(
        what +
        " LP not optimal: " + highs.modelStatusToString(highs.getModelStatus())
    );
  }
}

// The plain degree+cuts LP with one z column per edge whose costs can be
// swapped cheaply (warm-started re-solves). Used for the subgradient
// warm-start phase and for the dual-sign check on the extracted lambdas.
class FixedWeightLp {
 public:
  FixedWeightLp(
      int num_vertices,
      const std::vector<std::pair<int, int>>& cycle_arcs,
      const std::vector<std::pair<int, int>>& edge_arc
  )
      : num_vertices_(num_vertices),
        num_cycle_(static_cast<int>(cycle_arcs.size())),
        num_edges_(static_cast<int>(edge_arc.size())) {
    highs_.setOptionValue("output_flag", false);

    HighsLp lp;
    lp.num_row_ = 2 * num_vertices_;
    lp.row_lower_.assign(lp.num_row_, 1.0);
    lp.row_upper_.assign(lp.num_row_, 1.0);
    lp.a_matrix_.format_ = MatrixFormat::kColwise;
    // HighsLp default-constructs start_ as {0} already; make that explicit
    // instead of appending a second leading 0.
    lp.a_matrix_.start_.assign(1, 0);

    auto push_col = [&](double cost, int u, int v) {
      lp.col_cost_.push_back(cost);
      lp.col_lower_.push_back(0.0);
      lp.col_upper_.push_back(1.0);
      lp.a_matrix_.index_.push_back(u);
      lp.a_matrix_.value_.push_back(1.0);
      lp.a_matrix_.index_.push_back(num_vertices_ + v);
      lp.a_matrix_.value_.push_back(1.0);
      lp.a_matrix_.start_.push_back(
          static_cast<HighsInt>(lp.a_matrix_.index_.size())
      );
      col_arc_.push_back({u, v});
    };
    for (const auto& [u, v] : cycle_arcs) {
      push_col(kCycleEdgeWeight, u, v);
    }
    for (const auto& [u, v] : edge_arc) {
      push_col(0.0, u, v);
    }
    lp.num_col_ = static_cast<HighsInt>(col_arc_.size());
    lp.sense_ = ObjSense::kMinimize;
    if (highs_.passModel(lp) == HighsStatus::kError) {
      throw std::runtime_error("HiGHS passModel failed (fixed-weight LP)");
    }
  }

  void SetEdgeWeights(const std::vector<double>& weights) {
    if (static_cast<int>(weights.size()) != num_edges_) {
      throw std::runtime_error("SetEdgeWeights size mismatch");
    }
    if (highs_.changeColsCost(
            num_cycle_, num_cycle_ + num_edges_ - 1, weights.data()
        ) == HighsStatus::kError) {
      throw std::runtime_error("HiGHS changeColsCost failed");
    }
  }

  double Solve() {
    CheckHighsOptimal(highs_, "fixed-weight");
    highs_.setOptionValue("presolve", "off");
    return highs_.getObjectiveValue();
  }

  int SeparateCuts() {
    std::vector<std::vector<char>> cuts = FindViolatedSubtourCuts(
        num_vertices_, col_arc_, highs_.getSolution().col_value, cut_set_
    );
    for (std::vector<char>& in_S : cuts) {
      AddCutRow(std::move(in_S));
    }
    return static_cast<int>(cuts.size());
  }

  void AddCutRow(std::vector<char> in_S) {
    std::vector<HighsInt> idx;
    std::vector<double> val;
    for (size_t c = 0; c < col_arc_.size(); ++c) {
      const auto& [u, v] = col_arc_[c];
      if (in_S[u] && !in_S[v]) {
        idx.push_back(static_cast<HighsInt>(c));
        val.push_back(1.0);
      }
    }
    if (highs_.addRow(
            1.0,
            kHighsInf,
            static_cast<HighsInt>(idx.size()),
            idx.data(),
            val.data()
        ) == HighsStatus::kError) {
      throw std::runtime_error("HiGHS addRow failed (fixed-weight LP)");
    }
    cut_set_.insert(in_S);
    cut_membership_.push_back(std::move(in_S));
  }

  // Aggregate flow per edge (z column values).
  std::vector<double> EdgeFlows() const {
    const std::vector<double>& v = highs_.getSolution().col_value;
    return std::vector<double>(
        v.begin() + num_cycle_, v.begin() + num_cycle_ + num_edges_
    );
  }

  const std::vector<std::vector<char>>& Cuts() const { return cut_membership_; }

 private:
  int num_vertices_;
  int num_cycle_;
  int num_edges_;
  Highs highs_;
  std::vector<std::pair<int, int>> col_arc_;
  std::vector<std::vector<char>> cut_membership_;
  std::set<std::vector<char>> cut_set_;
};

// The one-shot Lagrangian LP: y columns for within-stop cycle edges, z columns
// for (edge, candidate) pairs, degree rows, consistency rows (whose duals are
// the Lagrangian multipliers), and lazily added subtour cuts.
class LagrangianLp {
 public:
  LagrangianLp(
      const std::vector<CandidateEdge>& edges,
      const std::vector<std::pair<int, int>>& edge_arc,
      const std::vector<std::pair<int, int>>& cycle_arcs,
      int num_vertices,
      int num_keys,
      bool use_consistency,
      double seed_eps,
      bool seed_all,
      const std::vector<double>& seed_lambda
  )
      : edges_(edges),
        edge_arc_(edge_arc),
        num_vertices_(num_vertices),
        num_keys_(num_keys),
        use_consistency_(use_consistency) {
    highs_.setOptionValue("output_flag", false);
    // Interior point (with crossover for a basis) is much faster than simplex
    // on the big initial solve; Solve() switches to warm-started simplex
    // afterwards.
    highs_.setOptionValue("solver", "ipm");

    first_cut_row_ = 2 * num_vertices_ + (use_consistency_ ? num_keys_ : 0);

    HighsLp lp;
    lp.num_row_ = first_cut_row_;
    lp.row_lower_.assign(lp.num_row_, 0.0);
    lp.row_upper_.assign(lp.num_row_, 0.0);
    for (int v = 0; v < 2 * num_vertices_; ++v) {
      lp.row_lower_[v] = 1.0;
      lp.row_upper_[v] = 1.0;
    }
    lp.a_matrix_.format_ = MatrixFormat::kColwise;
    // HighsLp default-constructs start_ as {0} already; make that explicit
    // instead of appending a second leading 0.
    lp.a_matrix_.start_.assign(1, 0);

    auto push_col = [&](double cost, int u, int v, const Candidate* cand) {
      lp.col_cost_.push_back(cost);
      lp.col_lower_.push_back(0.0);
      lp.col_upper_.push_back(1.0);
      for (const auto& [row, coef] : ColumnEntries(u, v, cand)) {
        lp.a_matrix_.index_.push_back(row);
        lp.a_matrix_.value_.push_back(coef);
      }
      lp.a_matrix_.start_.push_back(
          static_cast<HighsInt>(lp.a_matrix_.index_.size())
      );
      col_arc_.push_back({u, v});
    };

    for (const auto& [u, v] : cycle_arcs) {
      push_col(kCycleEdgeWeight, u, v, nullptr);
    }
    z_col_.resize(edges_.size());
    for (size_t i = 0; i < edges_.size(); ++i) {
      const CandidateEdge& e = edges_[i];
      z_col_[i].assign(e.candidates.size(), -1);
      // Seed one min lambda-modified candidate per edge, plus everything
      // strictly within seed_eps of that min (both measured at the seed
      // lambda); pricing brings in the rest as needed. The seed lambda only
      // shapes the starting column set, never validity.
      double min_mod = std::numeric_limits<double>::infinity();
      size_t argmin = 0;
      if (!seed_all) {
        for (size_t j = 0; j < e.candidates.size(); ++j) {
          double mod = ModifiedValue(e.candidates[j], seed_lambda);
          if (mod < min_mod) {
            min_mod = mod;
            argmin = j;
          }
        }
      }
      for (size_t j = 0; j < e.candidates.size(); ++j) {
        const Candidate& c = e.candidates[j];
        bool seed = seed_all || j == argmin ||
                    ModifiedValue(c, seed_lambda) < min_mod + seed_eps;
        if (seed) {
          z_col_[i][j] = static_cast<int>(col_arc_.size());
          push_col(c.value, edge_arc_[i].first, edge_arc_[i].second, &c);
        }
      }
    }
    lp.num_col_ = static_cast<HighsInt>(col_arc_.size());
    lp.sense_ = ObjSense::kMinimize;
    if (highs_.passModel(lp) == HighsStatus::kError) {
      throw std::runtime_error("HiGHS passModel failed");
    }
  }

  int NumCols() const { return static_cast<int>(col_arc_.size()); }
  int NumCuts() const { return static_cast<int>(cut_membership_.size()); }
  const std::vector<std::vector<char>>& Cuts() const { return cut_membership_; }

  // Adds the out-cut of vertex set S: sum of columns crossing out of S >= 1.
  void AddCut(std::vector<char> in_S) {
    std::vector<HighsInt> idx;
    std::vector<double> val;
    for (size_t c = 0; c < col_arc_.size(); ++c) {
      const auto& [u, v] = col_arc_[c];
      if (in_S[u] && !in_S[v]) {
        idx.push_back(static_cast<HighsInt>(c));
        val.push_back(1.0);
      }
    }
    if (highs_.addRow(
            1.0,
            kHighsInf,
            static_cast<HighsInt>(idx.size()),
            idx.data(),
            val.data()
        ) == HighsStatus::kError) {
      throw std::runtime_error("HiGHS addRow failed");
    }
    cut_set_.insert(in_S);
    cut_membership_.push_back(std::move(in_S));
  }

  double Solve() {
    CheckHighsOptimal(highs_, "main");
    // After the first solve a basis exists; re-solves warm-start with
    // simplex and skip presolve instead of solving from scratch.
    highs_.setOptionValue("solver", "simplex");
    highs_.setOptionValue("presolve", "off");
    return highs_.getObjectiveValue();
  }

  // Adds violated subtour cuts found on the aggregate flow. Returns the
  // number added.
  int SeparateComponentCuts() {
    std::vector<std::vector<char>> cuts = FindViolatedSubtourCuts(
        num_vertices_, col_arc_, highs_.getSolution().col_value, cut_set_
    );
    for (std::vector<char>& in_S : cuts) {
      AddCut(std::move(in_S));
    }
    return static_cast<int>(cuts.size());
  }

  // Column generation pricing: for each edge, excluded candidates with
  // negative reduced cost are added (capped per round). All z columns of an
  // edge share every dual except the consistency rows, so the shared part
  // pi_e is recovered from an included candidate's reduced cost:
  //   rc = (value_c + lambda(arr_c) - lambda(dep_c)) - pi_e
  // with lambda = -row_dual (the z column has +1 in its arr consistency row
  // and -1 in its dep row).
  int PriceCandidates() {
    // Copy what we need: adding columns invalidates the solution.
    std::vector<double> lambda = ExtractLambda(-1.0);
    std::vector<double> col_dual = highs_.getSolution().col_dual;
    struct Entry {
      double rc;
      int edge;
      int cand;
    };
    std::vector<Entry> violated;
    for (size_t i = 0; i < edges_.size(); ++i) {
      const auto& cands = edges_[i].candidates;
      int j0 = -1;
      for (size_t j = 0; j < cands.size(); ++j) {
        if (z_col_[i][j] >= 0) {
          j0 = static_cast<int>(j);
          break;
        }
      }
      if (j0 < 0) {
        throw std::runtime_error("Edge with no seeded candidate");
      }
      double pi = ModifiedValue(cands[j0], lambda) - col_dual[z_col_[i][j0]];
      for (size_t j = 0; j < cands.size(); ++j) {
        if (z_col_[i][j] >= 0) {
          continue;
        }
        double rc = ModifiedValue(cands[j], lambda) - pi;
        if (rc < -kPriceEps) {
          violated.push_back({rc, static_cast<int>(i), static_cast<int>(j)});
        }
      }
    }
    // Cap the columns added per round to the most negative ones so early
    // (badly distorted) duals cannot flood the LP.
    constexpr size_t kMaxColumnsPerRound = 25000;
    if (violated.size() > kMaxColumnsPerRound) {
      std::nth_element(
          violated.begin(),
          violated.begin() + kMaxColumnsPerRound,
          violated.end(),
          [](const Entry& a, const Entry& b) { return a.rc < b.rc; }
      );
      violated.resize(kMaxColumnsPerRound);
    }
    for (const Entry& e : violated) {
      AddZColumn(e.edge, e.cand);
    }
    return static_cast<int>(violated.size());
  }

  // The duals of the consistency rows are the optimal lambdas (up to a sign
  // convention resolved empirically by the caller). Indexed by interned key.
  std::vector<double> ExtractLambda(double sign) const {
    std::vector<double> lambda(num_keys_, 0.0);
    if (!use_consistency_) {
      return lambda;
    }
    const std::vector<double>& row_dual = highs_.getSolution().row_dual;
    for (int k = 0; k < num_keys_; ++k) {
      lambda[k] = sign * row_dual[2 * num_vertices_ + k];
    }
    return lambda;
  }

 private:
  // Row entries of a column with TSP arc (u, v): degree rows, consistency
  // rows (z only), and every cut it crosses. Coefficients are accumulated so
  // an arr/dep collision cancels instead of producing duplicate entries.
  std::map<int, double> ColumnEntries(
      int u, int v, const Candidate* cand
  ) const {
    std::map<int, double> entries;
    entries[u] += 1.0;
    entries[num_vertices_ + v] += 1.0;
    if (cand != nullptr && use_consistency_) {
      if (cand->arr >= 0) {
        entries[2 * num_vertices_ + cand->arr] += 1.0;
      }
      if (cand->dep >= 0) {
        entries[2 * num_vertices_ + cand->dep] -= 1.0;
      }
    }
    for (size_t k = 0; k < cut_membership_.size(); ++k) {
      const std::vector<char>& in_S = cut_membership_[k];
      if (in_S[u] && !in_S[v]) {
        entries[first_cut_row_ + static_cast<int>(k)] += 1.0;
      }
    }
    std::erase_if(entries, [](const auto& e) { return e.second == 0.0; });
    return entries;
  }

  void AddZColumn(int edge_idx, int cand_idx) {
    const Candidate& c = edges_[edge_idx].candidates[cand_idx];
    const auto& [u, v] = edge_arc_[edge_idx];
    std::vector<HighsInt> idx;
    std::vector<double> val;
    for (const auto& [row, coef] : ColumnEntries(u, v, &c)) {
      idx.push_back(row);
      val.push_back(coef);
    }
    if (highs_.addCol(
            c.value,
            0.0,
            1.0,
            static_cast<HighsInt>(idx.size()),
            idx.data(),
            val.data()
        ) == HighsStatus::kError) {
      throw std::runtime_error("HiGHS addCol failed");
    }
    z_col_[edge_idx][cand_idx] = static_cast<int>(col_arc_.size());
    col_arc_.push_back({u, v});
  }

  const std::vector<CandidateEdge>& edges_;
  const std::vector<std::pair<int, int>>& edge_arc_;
  int num_vertices_;
  int num_keys_;
  bool use_consistency_;
  Highs highs_;
  std::vector<std::pair<int, int>> col_arc_;
  std::vector<std::vector<char>> cut_membership_;
  std::set<std::vector<char>> cut_set_;
  int first_cut_row_;
  // z_col_[edge][candidate] = LP column index, or -1 if not (yet) included.
  std::vector<std::vector<int>> z_col_;
};

// Per-edge floored min lambda-modified candidate value (over ALL candidates).
// Flooring keeps the bound valid.
std::vector<int> FlooredModifiedWeights(
    const std::vector<CandidateEdge>& edges, const std::vector<double>& lambda
) {
  std::vector<int> weights;
  weights.reserve(edges.size());
  for (const CandidateEdge& e : edges) {
    double m = std::numeric_limits<double>::infinity();
    for (const Candidate& c : e.candidates) {
      m = std::min(m, ModifiedValue(c, lambda));
    }
    weights.push_back(static_cast<int>(std::floor(m + 1e-9)));
  }
  return weights;
}

int CeilBound(double corrected) {
  return static_cast<int>(std::ceil(corrected - 1e-6));
}

std::string Secs(std::chrono::steady_clock::time_point start) {
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start
  )
                .count();
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << (ms / 1000.0) << " s";
  return ss.str();
}

}  // namespace

int main(int argc, char* argv[]) {
  CLI::App app{"Lagrangian-strengthened tarel lower bound via one-shot LP"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();
  std::optional<int> ub_arg;
  app.add_option(
      "--ub", ub_arg, "Known upper bound in seconds for gap reporting"
  );
  bool run_hk = false;
  app.add_flag(
      "--hk", run_hk, "Run Held-Karp DP first and use its optimum as the UB"
  );
  double seed_eps = 60.0;
  app.add_option(
      "--seed-eps",
      seed_eps,
      "Column-generation seed width in seconds around the min lambda-modified "
      "candidate per edge"
  );
  int sg_iters = 30;
  app.add_option(
      "--sg-iters",
      sg_iters,
      "Subgradient warm-start iterations for the seed lambda (0 = off)"
  );
  bool no_consistency = false;
  app.add_flag(
      "--no-consistency",
      no_consistency,
      "Drop consistency rows (forces lambda = 0), for validation"
  );
  int max_rounds = 100;
  app.add_option("--max-rounds", max_rounds, "Max cut/pricing rounds");

  CLI11_PARSE(app, argc, argv);

  auto t_start = std::chrono::steady_clock::now();

  std::ifstream in(input_path);
  if (!in.is_open()) {
    std::cerr << "Error: could not open " << input_path << std::endl;
    return 1;
  }
  nlohmann::json j = nlohmann::json::parse(in);
  ProblemState state = j.get<ProblemState>();
  in.close();

  std::cout << "Loaded problem state from: " << input_path << std::endl;
  std::cout << "Stops: " << state.minimal.NumStops()
            << ", required: " << state.required.size() << std::endl;

  std::optional<int> ub = ub_arg;
  if (run_hk) {
    std::cout << "Running Held-Karp DP for the upper bound..." << std::endl;
    auto t_hk = std::chrono::steady_clock::now();
    HeldKarpDPResult hk = HeldKarpDPSolve(state, 0, &std::cerr);
    if (hk.best_val == std::numeric_limits<int>::max()) {
      std::cout << "Held-Karp: infeasible, no upper bound" << std::endl;
    } else {
      ub = hk.best_val;
      std::cout << "Held-Karp optimum: " << hk.best_val << " ("
                << TimeSinceServiceStart{hk.best_val} << ") in " << Secs(t_hk)
                << std::endl;
    }
  }

  // ---- Candidate enumeration ----
  auto t_enum = std::chrono::steady_clock::now();
  StepPathsAdjacencyList completed = state.ComputeCompletedGraph();
  TarelEdgeIntermediateData data =
      ComputeTarelIntermediateData(completed.AllMergedSteps());
  KeyInterner keys;
  std::vector<CandidateEdge> edges = EnumerateCandidateEdges(data, keys);
  CheckCandidatesAgainstTarelEdges(edges, data);

  size_t total_candidates = 0;
  for (const CandidateEdge& e : edges) {
    total_candidates += e.candidates.size();
  }
  std::cout << "Tarel edges: " << edges.size()
            << ", deduped candidates: " << total_candidates
            << ", lambda keys: " << keys.size() << " (" << Secs(t_enum) << ")"
            << std::endl;

  // Size gate: build the full LP directly when small enough, otherwise seed a
  // window around each edge's min and price the rest.
  constexpr size_t kMaxDirectColumns = 2'000'000;
  bool seed_all = total_candidates <= kMaxDirectColumns;
  std::cout << (seed_all ? "Direct build (all candidates seeded)"
                         : "Column generation (seed-eps = " +
                               std::to_string(seed_eps) + ")")
            << std::endl;

  // ---- Fixed TSP structure on 1:1-mapped states ----
  std::set<TarelState> all_states;
  for (const CandidateEdge& e : edges) {
    all_states.insert(e.origin);
    all_states.insert(e.destination);
  }
  std::map<StopId, std::vector<TarelState>> by_rep;
  for (const TarelState& s : all_states) {
    by_rep[state.required.Representative(s.stop)].push_back(s);
  }
  for (StopId rep : state.required.GroupRepresentatives()) {
    if (!by_rep.contains(rep)) {
      std::cout << "Required group representative " << rep
                << " missing from tarel graph; no bound." << std::endl;
      return 1;
    }
  }
  std::unordered_map<TarelState, TarelState> to_mapped;
  for (const auto& [rep, members] : by_rep) {
    for (size_t i = 0; i < members.size(); ++i) {
      to_mapped[members[i]] =
          TarelState{rep, StepPartitionId{static_cast<int>(i)}};
    }
  }
  std::vector<TarelEdge> mapped_edges;
  mapped_edges.reserve(edges.size());
  for (const CandidateEdge& e : edges) {
    mapped_edges.push_back(
        TarelEdge{
            .origin = to_mapped.at(e.origin),
            .destination = to_mapped.at(e.destination),
            .weight = e.min_value,
        }
    );
  }
  TspGraphData graph = MakeTspGraphEdges(mapped_edges, state.boundary);
  int num_cycle_slots = 0;
  for (const auto& [stop, n] : graph.num_states_by_stop) {
    num_cycle_slots += n;
  }
  if (graph.tsp_edges.size() !=
      static_cast<size_t>(num_cycle_slots) + mapped_edges.size()) {
    throw std::runtime_error("Unexpected MakeTspGraphEdges edge layout");
  }
  int num_vertices = static_cast<int>(graph.state_by_id.size());
  std::vector<std::pair<int, int>> cycle_arcs;
  for (int k = 0; k < num_cycle_slots; ++k) {
    const WeightedEdge& e = graph.tsp_edges[k];
    if (e.origin != e.destination) {
      cycle_arcs.push_back({e.origin.v, e.destination.v});
    }
  }
  std::vector<std::pair<int, int>> edge_arc;
  edge_arc.reserve(edges.size());
  for (size_t i = 0; i < edges.size(); ++i) {
    const WeightedEdge& e = graph.tsp_edges[num_cycle_slots + i];
    edge_arc.push_back({e.origin.v, e.destination.v});
  }
  std::cout << "TSP structure: " << num_vertices << " vertices, "
            << cycle_arcs.size() << " cycle edges, "
            << graph.expected_num_cycle_edges << " expected in a tour"
            << std::endl;

  auto corrected = [&](double obj) {
    return obj + 1000.0 * graph.expected_num_cycle_edges;
  };

  // One subtour cut per multi-state stop: its -1000 within-stop cycle is a
  // subtour the LPs would otherwise exploit.
  std::vector<std::vector<char>> stop_cuts;
  {
    std::map<StopId, std::vector<int>> vertices_by_stop;
    for (size_t id = 0; id < graph.state_by_id.size(); ++id) {
      vertices_by_stop[graph.state_by_id[id].stop].push_back(
          static_cast<int>(id)
      );
    }
    for (const auto& [stop, vertices] : vertices_by_stop) {
      if (vertices.size() < 2) {
        continue;
      }
      std::vector<char> in_S(num_vertices, 0);
      for (int v : vertices) {
        in_S[v] = 1;
      }
      stop_cuts.push_back(std::move(in_S));
    }
  }

  // ---- Subgradient warm start ----
  // A few Polyak-style iterations on the small fixed-weight LP produce a
  // near-optimal lambda to seed the big LP's columns with, plus a cut pool
  // discovered along the way. This only shapes the starting point; the big
  // LP's own pricing and duals determine the final (valid) bound.
  std::vector<double> seed_lambda;
  std::vector<std::vector<char>> initial_cuts = stop_cuts;
  if (!seed_all && !no_consistency && sg_iters > 0) {
    auto t_sg = std::chrono::steady_clock::now();
    FixedWeightLp sg_lp(num_vertices, cycle_arcs, edge_arc);
    for (const std::vector<char>& in_S : stop_cuts) {
      sg_lp.AddCutRow(in_S);
    }
    std::vector<double> lambda(keys.size(), 0.0);
    std::vector<double> best_lambda = lambda;
    double best_dual = -std::numeric_limits<double>::infinity();
    std::vector<double> weights(edges.size());
    std::vector<double> momentum(keys.size(), 0.0);
    double margin = 500.0;
    int no_improvement_streak = 0;
    for (int iter = 0; iter < sg_iters; ++iter) {
      for (size_t i = 0; i < edges.size(); ++i) {
        double min_mod = std::numeric_limits<double>::infinity();
        for (const Candidate& c : edges[i].candidates) {
          min_mod = std::min(min_mod, ModifiedValue(c, lambda));
        }
        weights[i] = min_mod;
      }
      sg_lp.SetEdgeWeights(weights);
      double obj = sg_lp.Solve();
      while (sg_lp.SeparateCuts() > 0) {
        obj = sg_lp.Solve();
      }
      double dual = corrected(obj);
      if (dual > best_dual) {
        best_dual = dual;
        best_lambda = lambda;
        no_improvement_streak = 0;
      } else {
        no_improvement_streak += 1;
        if (no_improvement_streak >= 3) {
          margin *= 0.5;
          no_improvement_streak = 0;
        }
      }
      // Subgradient of the dual: per unit of flow on edge i, its min-modified
      // candidate creates its arr event and consumes its dep event. Spread
      // over all near-min candidates (only support edges need a rescan),
      // which smooths the subgradient considerably.
      constexpr double kSpreadEps = 30.0;
      std::vector<double> flows = sg_lp.EdgeFlows();
      std::vector<double> g(lambda.size(), 0.0);
      for (size_t i = 0; i < edges.size(); ++i) {
        if (flows[i] <= 1e-9) {
          continue;
        }
        int num_near = 0;
        for (const Candidate& c : edges[i].candidates) {
          if (ModifiedValue(c, lambda) < weights[i] + kSpreadEps) {
            num_near += 1;
          }
        }
        double w = flows[i] / num_near;
        for (const Candidate& c : edges[i].candidates) {
          if (ModifiedValue(c, lambda) < weights[i] + kSpreadEps) {
            if (c.arr >= 0) g[c.arr] += w;
            if (c.dep >= 0) g[c.dep] -= w;
          }
        }
      }
      for (size_t k = 0; k < g.size(); ++k) {
        g[k] += 0.7 * momentum[k];
      }
      double norm2 = 0.0;
      for (double v : g) {
        norm2 += v * v;
      }
      std::cout << "Subgradient iter " << iter << ": dual " << CeilBound(dual)
                << std::endl;
      if (norm2 < 1e-12) {
        break;
      }
      // Polyak step toward an adaptive optimistic target; oscillation is fine
      // since only the best lambda is kept.
      double step = std::max(0.0, best_dual + margin - dual) / norm2;
      for (size_t k = 0; k < lambda.size(); ++k) {
        lambda[k] += step * g[k];
      }
      momentum = std::move(g);
    }
    seed_lambda = std::move(best_lambda);
    initial_cuts = sg_lp.Cuts();
    std::cout << "Subgradient warm start: best dual " << CeilBound(best_dual)
              << ", " << initial_cuts.size() << " cuts (" << Secs(t_sg) << ")"
              << std::endl;
  }

  // ---- Build and solve the LP with cut (and pricing) rounds ----
  auto t_lp = std::chrono::steady_clock::now();
  LagrangianLp lp(
      edges,
      edge_arc,
      cycle_arcs,
      num_vertices,
      keys.size(),
      !no_consistency,
      seed_eps,
      seed_all,
      seed_lambda
  );
  for (const std::vector<char>& in_S : initial_cuts) {
    lp.AddCut(in_S);
  }

  double obj = lp.Solve();
  std::cout << "Initial LP (" << lp.NumCols() << " cols): bound "
            << CeilBound(corrected(obj)) << " (" << Secs(t_lp) << ")"
            << std::endl;

  int round = 0;
  bool converged = false;
  while (round < max_rounds) {
    round += 1;
    int num_cuts = lp.SeparateComponentCuts();
    int num_priced = 0;
    if (num_cuts == 0 && !seed_all) {
      num_priced = lp.PriceCandidates();
    }
    if (num_cuts == 0 && num_priced == 0) {
      converged = true;
      break;
    }
    obj = lp.Solve();
    std::cout << "Round " << round << ": "
              << (num_cuts > 0
                      ? std::to_string(num_cuts) + " cuts added"
                      : std::to_string(num_priced) + " columns priced in")
              << ", bound " << CeilBound(corrected(obj)) << " (" << Secs(t_lp)
              << ")" << std::endl;
  }
  if (!converged) {
    std::cout << "Hit max rounds (" << max_rounds
              << ") before cut convergence; bound stays valid with a partial "
                 "cut pool."
              << std::endl;
    // Cut convergence is optional for validity, but pricing convergence is
    // not: with columns missing, the restricted LP's minimum can exceed the
    // full LP's minimum and would not be a valid lower bound. Price to
    // convergence (finite: every round adds at least one column).
    if (!seed_all) {
      int pricing_rounds = 0;
      while (true) {
        int num_priced = lp.PriceCandidates();
        if (num_priced == 0) {
          break;
        }
        obj = lp.Solve();
        pricing_rounds += 1;
      }
      if (pricing_rounds > 0) {
        std::cout << "Post-loop pricing: " << pricing_rounds
                  << " rounds to restore bound validity, bound "
                  << CeilBound(corrected(obj)) << std::endl;
      }
    }
  }

  int lp_bound = CeilBound(corrected(obj));
  std::cout << "LP bound: " << lp_bound << " ("
            << TimeSinceServiceStart{lp_bound} << "), " << lp.NumCuts()
            << " cuts, " << lp.NumCols() << " cols, total LP time "
            << Secs(t_lp) << std::endl;

  // ---- Lambda extraction with empirical dual-sign check ----
  // Both signs give valid bounds (any lambda does); pick the one whose plain
  // degree+cuts LP over the floored modified weights best reproduces the big
  // LP's objective.
  double best_sign = 1.0;
  int best_check_bound = std::numeric_limits<int>::min();
  std::vector<int> best_weights;
  for (double sign : {1.0, -1.0}) {
    std::vector<double> lambda = lp.ExtractLambda(sign);
    std::vector<int> weights = FlooredModifiedWeights(edges, lambda);
    FixedWeightLp check_lp(num_vertices, cycle_arcs, edge_arc);
    for (const std::vector<char>& in_S : lp.Cuts()) {
      check_lp.AddCutRow(in_S);
    }
    check_lp.SetEdgeWeights(
        std::vector<double>(weights.begin(), weights.end())
    );
    int check_bound = CeilBound(corrected(check_lp.Solve()));
    std::cout << "Dual sign " << (sign > 0 ? "+1" : "-1") << ": check LP bound "
              << check_bound << std::endl;
    if (check_bound > best_check_bound) {
      best_check_bound = check_bound;
      best_sign = sign;
      best_weights = std::move(weights);
    }
  }
  std::cout << "Using dual sign " << (best_sign > 0 ? "+1" : "-1")
            << " (check bound " << best_check_bound
            << ", should be ~= LP bound " << lp_bound << " up to flooring)"
            << std::endl;
  if (best_check_bound > lp_bound + 1) {
    std::cout << "Warning: check LP bound exceeds the big LP bound; "
                 "dual extraction is suspect."
              << std::endl;
  }

  // ---- Certification: Concorde at the extracted lambda ----
  std::optional<int> certified_bound;
  {
    auto t_cert = std::chrono::steady_clock::now();
    std::vector<TarelEdge> modified_edges;
    modified_edges.reserve(edges.size());
    for (size_t i = 0; i < edges.size(); ++i) {
      modified_edges.push_back(
          TarelEdge{
              .origin = edges[i].origin,
              .destination = edges[i].destination,
              .weight = best_weights[i],
          }
      );
    }
    try {
      TarelStateRemapResult remap =
          RemapTarelStates(modified_edges, state.required);
      TspGraphData cert_graph = MakeTspGraphEdges(remap.edges, state.boundary);
      std::unordered_set<StopId> representatives_in_graph;
      for (const TarelState& ts : cert_graph.state_by_id) {
        representatives_in_graph.insert(state.required.Representative(ts.stop));
      }
      bool all_present = true;
      for (StopId rep : state.required.GroupRepresentatives()) {
        if (!representatives_in_graph.contains(rep)) {
          all_present = false;
        }
      }
      if (!all_present) {
        std::cout << "Certification skipped: representative missing from "
                     "remapped graph."
                  << std::endl;
      } else {
        std::optional<TspTourResult> result;
        for (int attempt = 0; attempt < 3; ++attempt) {
          try {
            result = SolveTspAndExtractTour(
                remap.edges, cert_graph, state.boundary, std::nullopt, nullptr
            );
            break;
          } catch (const ConcordeCrash& e) {
            std::cout << "Concorde crashed (attempt " << attempt + 1
                      << "): " << e.what() << std::endl;
          }
        }
        if (result.has_value()) {
          certified_bound = result->optimal_value;
          std::cout << "Certified Concorde bound at lambda: "
                    << *certified_bound << " ("
                    << TimeSinceServiceStart{*certified_bound} << ") in "
                    << Secs(t_cert) << std::endl;
        } else {
          std::cout << "Certification failed: no Concorde solution."
                    << std::endl;
        }
      }
    } catch (const std::exception& e) {
      std::cout << "Certification failed: " << e.what() << std::endl;
    }
  }

  // ---- Summary ----
  int final_bound = lp_bound;
  if (certified_bound.has_value()) {
    final_bound = std::max(final_bound, *certified_bound);
  }
  std::cout << std::endl;
  std::cout << "LP bound:        " << lp_bound << " ("
            << TimeSinceServiceStart{lp_bound} << ")" << std::endl;
  if (certified_bound.has_value()) {
    std::cout << "Concorde bound:  " << *certified_bound << " ("
              << TimeSinceServiceStart{*certified_bound} << ")" << std::endl;
  }
  std::cout << "Final bound:     " << final_bound << " ("
            << TimeSinceServiceStart{final_bound} << ")" << std::endl;
  if (ub.has_value()) {
    double gap = 100.0 * (*ub - final_bound) / *ub;
    std::cout << "Upper bound:     " << *ub << " ("
              << TimeSinceServiceStart{*ub} << "), gap " << std::fixed
              << std::setprecision(2) << gap << "%" << std::endl;
    if (final_bound > *ub) {
      std::cout << "ERROR: lower bound exceeds the upper bound!" << std::endl;
      return 1;
    }
  }
  std::cout << "Total time: " << Secs(t_start) << std::endl;

  return 0;
}
