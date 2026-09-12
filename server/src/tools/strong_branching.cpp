// Experiment: strong branching at the root node of branch and bound.
//
// Loads a ProblemState, solves the tarel lower bound's Concorde root LP (no
// branching), and derives two kinds of candidate branches from the LP support:
//
//  - Edge branches: the primitive (minimal-graph) edges underlying every
//    support edge, branched as Require / Forbid.
//  - Critical time branches: for every original tarel edge behind a support
//    edge, the arrival times at its origin that determine its weight, branched
//    as "only these arrival times at the stop" / "none of these".
//
// For each candidate it solves the root LP of both children and reports their
// lift over the root bound as it goes, along with a running top-N of branches
// by score = 10 * min_lift + max_lift.

#include <CLI/CLI.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
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
#include <variant>
#include <vector>

#include "solver/branch_and_bound.h"
#include "solver/tarel_graph.h"

using namespace vats5;

namespace {

std::string FormatDuration(int ms) {
  if (ms < 1000) {
    return std::to_string(ms) + " ms";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1) << (ms / 1000.0) << " s";
  return ss.str();
}

int ElapsedMs(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - start
  )
      .count();
}

// A candidate branch: two complementary constraints.
struct Candidate {
  std::string label;
  // Name of each child, e.g. "require" / "forbid".
  std::string left_name;
  ProblemConstraint left;
  std::string right_name;
  ProblemConstraint right;
  // Sum of x over the support edges this candidate was derived from.
  double support_mass = 0.0;
  // Number of support edges this candidate was derived from.
  int support_count = 0;
};

// The relaxation of a child was proven infeasible: the branch is closed.
struct ChildInfeasible {};

// The root LP of a child: its bound, or a proof of infeasibility.
using ChildBound = std::variant<int, ChildInfeasible>;

struct ChildResult {
  ChildBound bound;
  int ms;
  // Change in the number of minimal steps from applying the constraint
  // (Require adds merged steps, so this can be positive).
  int constraint_step_delta;
  // Minimal steps removed by the --ub narrowing (0 without --ub).
  int steps_removed;
};

// A child's lift over the root bound. Infeasible children lift infinitely.
double Lift(const ChildBound& bound, int root_lb) {
  if (std::holds_alternative<ChildInfeasible>(bound)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::get<int>(bound) - root_lb;
}

std::string FormatLift(double lift) {
  if (std::isinf(lift)) {
    return "inf (infeasible)";
  }
  std::ostringstream ss;
  ss << std::showpos << static_cast<int>(lift);
  return ss.str();
}

std::string FormatScore(double score) {
  if (std::isinf(score)) {
    return "inf";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(0) << score;
  return ss.str();
}

struct BranchEvaluation {
  Candidate candidate;
  ChildResult left;
  ChildResult right;

  double LeftLift(int root_lb) const { return Lift(left.bound, root_lb); }
  double RightLift(int root_lb) const { return Lift(right.bound, root_lb); }
  double Score(int root_lb) const {
    double a = LeftLift(root_lb);
    double b = RightLift(root_lb);
    return 10.0 * std::min(a, b) + std::max(a, b);
  }
};

// How a problem's bound is computed for scoring: Concorde's root LP only, or
// the exact (integral) tarel bound.
enum class BoundMethod { kLp, kExact };

// The tarel bound of `state` by `method`, or nullopt if infeasible.
std::optional<int> ComputeBound(const ProblemState& state, BoundMethod method) {
  switch (method) {
    case BoundMethod::kLp: {
      std::optional<TarelRootLpResult> lp = ComputeTarelRootLp(state);
      if (!lp.has_value()) {
        return std::nullopt;
      }
      return lp->lower_bound;
    }
    case BoundMethod::kExact: {
      std::optional<TspTourResult> exact = ComputeTarelLowerBound(state);
      if (!exact.has_value()) {
        return std::nullopt;
      }
      return exact->optimal_value;
    }
  }
  __builtin_unreachable();
}

ChildResult EvaluateChild(
    const ProblemState& state,
    const ProblemConstraint& constraint,
    std::optional<int> ub,
    BoundMethod method
) {
  auto start = std::chrono::steady_clock::now();
  ProblemState child = ApplyConstraints(state, {constraint});
  int delta = static_cast<int>(child.minimal.AllSteps().size()) -
              static_cast<int>(state.minimal.AllSteps().size());
  int steps_removed = 0;
  if (ub.has_value()) {
    std::optional<ArrivalWindowNarrowing> narrowed =
        NarrowArrivalTimes(child, *ub);
    if (!narrowed.has_value()) {
      return ChildResult{ChildInfeasible{}, ElapsedMs(start), delta, 0};
    }
    steps_removed = narrowed->num_steps_removed;
    child = std::move(narrowed->state);
  }
  std::optional<int> bound = ComputeBound(child, method);
  int ms = ElapsedMs(start);
  if (!bound.has_value()) {
    return ChildResult{ChildInfeasible{}, ms, delta, steps_removed};
  }
  return ChildResult{*bound, ms, delta, steps_removed};
}

std::string FormatChild(const ChildResult& r, std::optional<int> ub) {
  std::ostringstream s;
  s << FormatDuration(r.ms) << ", constraint " << std::showpos
    << r.constraint_step_delta << std::noshowpos << " steps";
  if (ub.has_value()) {
    s << ", window -" << r.steps_removed << " steps";
  }
  return s.str();
}

void PrintTop(
    const std::vector<BranchEvaluation>& evaluations, int root_lb, int top_n
) {
  std::vector<const BranchEvaluation*> sorted;
  for (const BranchEvaluation& e : evaluations) {
    sorted.push_back(&e);
  }
  std::stable_sort(
      sorted.begin(),
      sorted.end(),
      [&](const BranchEvaluation* a, const BranchEvaluation* b) {
        return a->Score(root_lb) > b->Score(root_lb);
      }
  );
  std::cout << "  top " << std::min<int>(top_n, sorted.size()) << " of "
            << sorted.size() << " by score:\n";
  for (int i = 0; i < top_n && i < static_cast<int>(sorted.size()); ++i) {
    const BranchEvaluation& e = *sorted[i];
    std::cout << "    " << std::setw(2) << (i + 1) << ". score " << std::setw(6)
              << FormatScore(e.Score(root_lb)) << "  " << e.candidate.label
              << "  (" << e.candidate.left_name << " "
              << FormatLift(e.LeftLift(root_lb)) << ", "
              << e.candidate.right_name << " "
              << FormatLift(e.RightLift(root_lb)) << ")\n";
  }
}

// Orders candidates by descending support mass, then by label for
// determinism.
void SortCandidates(std::vector<Candidate>& candidates) {
  std::sort(
      candidates.begin(),
      candidates.end(),
      [](const Candidate& a, const Candidate& b) {
        if (a.support_mass != b.support_mass) {
          return a.support_mass > b.support_mass;
        }
        return a.label < b.label;
      }
  );
}

// Primitive edges of the minimal graph underlying the LP support: every step
// of every completed-graph path between a support edge's stops.
std::vector<Candidate> EdgeCandidates(
    const ProblemState& state,
    const StepPathsAdjacencyList& completed,
    const TarelRootLpResult& root
) {
  std::unordered_map<BranchEdge, Candidate> by_edge;
  for (const TarelSupportEdge& s : root.support) {
    StopId a = s.edge.origin.stop;
    StopId b = s.edge.destination.stop;
    if (a == state.boundary.end && b == state.boundary.start) {
      // The artificial cycle-closing edge; nothing to branch on.
      continue;
    }
    std::set<BranchEdge, decltype([](const BranchEdge& x, const BranchEdge& y) {
               return std::pair(x.a, x.b) < std::pair(y.a, y.b);
             })>
        seen_in_this_support;
    for (const Path& path : completed.PathsBetween(a, b)) {
      for (const Step& step : path.steps) {
        BranchEdge edge{step.origin.stop, step.destination.stop};
        if (!seen_in_this_support.insert(edge).second) {
          continue;
        }
        auto [it, _] = by_edge.try_emplace(
            edge,
            Candidate{
                .label = "edge " + edge.Debug(state),
                .left_name = "require",
                .left = edge.Require(),
                .right_name = "forbid",
                .right = edge.Forbid(),
            }
        );
        it->second.support_mass += s.x;
        it->second.support_count += 1;
      }
    }
  }
  std::vector<Candidate> candidates;
  for (auto& [_, c] : by_edge) {
    candidates.push_back(std::move(c));
  }
  SortCandidates(candidates);
  return candidates;
}

std::string FormatTimes(const std::vector<TimeSinceServiceStart>& times) {
  constexpr int kMaxListed = 4;
  std::string result = "{";
  for (int i = 0; i < static_cast<int>(times.size()) && i < kMaxListed; ++i) {
    if (i > 0) {
      result += ", ";
    }
    result += times[i].ToString();
  }
  if (times.size() > kMaxListed) {
    result += ", +" + std::to_string(times.size() - kMaxListed) + " more";
  }
  return result + "}";
}

struct CriticalTimeCandidates {
  std::vector<Candidate> candidates;
  // Original tarel edges behind the support whose origin arrival is flex and
  // so have no critical times.
  int num_flex_origin_edges = 0;
  // Distinct critical time sets dropped because no scheduled minimal step
  // realizes any of their times (the constraint would be a no-op).
  int num_unrealizable_sets = 0;
};

// Critical time sets of every original tarel edge that determines the weight
// of a support edge, deduplicated by (stop, times).
CriticalTimeCandidates TimeCandidates(
    const ProblemState& state, const TarelRootLpResult& root
) {
  // Original tarel edges grouped by the mapped edge they merged into.
  std::map<std::pair<TarelState, TarelState>, std::vector<const TarelEdge*>>
      originals_by_mapped;
  for (const TarelEdge& e : root.tarel_edges) {
    originals_by_mapped[std::make_pair(
                            root.remap.original_to_mapped.at(e.origin),
                            root.remap.original_to_mapped.at(e.destination)
                        )]
        .push_back(&e);
  }

  // Scheduled arrival times of minimal steps, per stop: the times a
  // ConstraintArrivalTimes can actually act on.
  std::unordered_map<StopId, std::set<TimeSinceServiceStart>> step_arrivals;
  for (const Step& s : state.minimal.AllSteps()) {
    if (!s.is_flex) {
      step_arrivals[s.destination.stop].insert(s.destination.time);
    }
  }

  struct SetInfo {
    Candidate candidate;
    std::set<StepPartitionId> partitions;
    int num_critical;
    int num_arrival_times;
    int num_realizable;
  };
  std::map<std::pair<StopId, std::vector<TimeSinceServiceStart>>, SetInfo>
      by_set;

  CriticalTimeCandidates result;
  for (const TarelSupportEdge& s : root.support) {
    if (s.edge.origin.stop == state.boundary.end &&
        s.edge.destination.stop == state.boundary.start) {
      continue;
    }
    auto it = originals_by_mapped.find(
        std::make_pair(s.mapped.origin, s.mapped.destination)
    );
    if (it == originals_by_mapped.end()) {
      throw std::logic_error("support edge has no original tarel edges");
    }
    for (const TarelEdge* original : it->second) {
      if (original->weight != s.mapped.weight) {
        // Merged away: does not determine the LP edge's weight.
        continue;
      }
      std::optional<CriticalTimes> critical =
          ComputeCriticalTimes(root.intermediate, *original);
      if (!critical.has_value()) {
        result.num_flex_origin_edges += 1;
        continue;
      }
      StopId stop = original->origin.stop;
      auto key = std::make_pair(stop, critical->times);
      auto [set_it, inserted] = by_set.try_emplace(
          key,
          SetInfo{
              .candidate =
                  Candidate{
                      .label = "arrivals at " + state.StopName(stop) + " in " +
                               FormatTimes(critical->times),
                      .left_name = "only",
                      .left =
                          ConstraintArrivalTimes{stop, critical->times, true},
                      .right_name = "remove",
                      .right =
                          ConstraintArrivalTimes{stop, critical->times, false},
                  },
              .partitions = {},
              .num_critical = static_cast<int>(critical->times.size()),
              .num_arrival_times = critical->num_arrival_times,
              .num_realizable = 0,
          }
      );
      SetInfo& info = set_it->second;
      info.partitions.insert(original->origin.partition);
      info.candidate.support_mass += s.x;
      info.candidate.support_count += 1;
      if (inserted) {
        auto arrivals_it = step_arrivals.find(stop);
        for (const TimeSinceServiceStart& t : critical->times) {
          if (arrivals_it != step_arrivals.end() &&
              arrivals_it->second.contains(t)) {
            info.num_realizable += 1;
          }
        }
      }
    }
  }

  for (auto& [_, info] : by_set) {
    if (info.num_realizable == 0) {
      result.num_unrealizable_sets += 1;
      continue;
    }
    std::ostringstream detail;
    detail << " [" << info.num_critical << " of " << info.num_arrival_times
           << " arrivals";
    if (info.num_realizable < info.num_critical) {
      // Some critical times come from arrivals via a trailing flex step and
      // no scheduled minimal step arrives then, so the constraint cannot
      // act on them.
      detail << ", " << info.num_realizable << " realizable";
    }
    detail << ", partitions";
    for (StepPartitionId p : info.partitions) {
      detail << " " << state.PartitionName(p);
    }
    detail << "]";
    info.candidate.label += detail.str();
    result.candidates.push_back(std::move(info.candidate));
  }
  SortCandidates(result.candidates);
  return result;
}

}  // namespace

int main(int argc, char* argv[]) {
  CLI::App app{"Strong branching experiment at the branch-and-bound root"};

  std::string input_path;
  app.add_option("input_path", input_path, "Path to ProblemState JSON file")
      ->required();

  int top_n = 10;
  app.add_option("--top", top_n, "How many top branches to print")
      ->default_val(10);

  int max_edges = -1;
  app.add_option(
         "--max-edges",
         max_edges,
         "Only evaluate the first N primitive edges (-1 for all)"
  )
      ->default_val(-1);

  int max_times = -1;
  app.add_option(
         "--max-times",
         max_times,
         "Only evaluate the first N critical time sets (-1 for all)"
  )
      ->default_val(-1);

  std::string branches = "both";
  app.add_option("--branches", branches, "Which branch kinds to evaluate")
      ->check(CLI::IsMember({"edges", "times", "both"}))
      ->default_val("both");

  std::optional<std::string> ub_str;
  app.add_option(
      "--ub",
      ub_str,
      "Known upper bound on tour duration (hh:mm:ss). Narrows every problem "
      "to the arrival window a tour this short can use before its root LP"
  );

  std::string bound_str = "lp";
  app.add_option(
         "--bound",
         bound_str,
         "Bound used for the root reference and every child: Concorde's root "
         "LP only, or the exact tarel bound"
  )
      ->check(CLI::IsMember({"lp", "exact"}))
      ->default_val("lp");

  std::optional<std::string> root_tsp_log;
  app.add_option(
      "--root-tsp-log",
      root_tsp_log,
      "Write Concorde's output for the root LP solve to this file"
  );

  bool print_support = false;
  app.add_flag(
      "--print-support", print_support, "Print the root LP's support edges"
  );

  CLI11_PARSE(app, argc, argv);

  std::optional<int> ub;
  if (ub_str.has_value()) {
    ub = TimeSinceServiceStart::Parse(*ub_str).seconds;
  }
  BoundMethod method =
      bound_str == "exact" ? BoundMethod::kExact : BoundMethod::kLp;

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
  std::cout << "Required stops: " << state.required.size() << "\n\n";

  auto root_start = std::chrono::steady_clock::now();
  if (ub.has_value()) {
    std::optional<ArrivalWindowNarrowing> narrowed =
        NarrowArrivalTimes(state, *ub);
    if (!narrowed.has_value()) {
      std::cout << "No tour of duration <= " << TimeSinceServiceStart{*ub}
                << " exists: narrowing emptied the problem\n";
      return 1;
    }
    std::cout << "Narrowed to window [" << narrowed->earliest << ", "
              << narrowed->latest << "] for ub " << TimeSinceServiceStart{*ub}
              << ": removed " << narrowed->num_steps_removed << " of "
              << state.minimal.AllSteps().size() << " steps in "
              << narrowed->num_rounds << " rounds ("
              << FormatDuration(ElapsedMs(root_start)) << ")\n";
    state = std::move(narrowed->state);
  }

  // Root LP.
  std::optional<std::ofstream> root_log_file;
  if (root_tsp_log.has_value()) {
    root_log_file.emplace(*root_tsp_log);
  }
  std::optional<TarelRootLpResult> root = ComputeTarelRootLp(
      state, root_log_file.has_value() ? &*root_log_file : nullptr
  );
  int root_ms = ElapsedMs(root_start);
  if (!root.has_value()) {
    std::cout << "Root LP infeasible (" << FormatDuration(root_ms) << ")\n";
    return 1;
  }
  std::cout << "Root LP: bound " << std::fixed << std::setprecision(2)
            << root->lp_bound << " -> lb " << root->lower_bound << " ("
            << TimeSinceServiceStart{root->lower_bound} << "), "
            << root->support.size() << " support edges, "
            << FormatDuration(root_ms) << "\n";
  // The exact bound is always shown so the root LP's shortfall is visible; it
  // is also the reference for lifts under --bound exact.
  std::optional<TspTourResult> root_exact;
  {
    auto exact_start = std::chrono::steady_clock::now();
    root_exact = ComputeTarelLowerBound(state);
    std::cout << "Root exact tarel lb: ";
    if (root_exact.has_value()) {
      std::cout << root_exact->optimal_value << " ("
                << TimeSinceServiceStart{root_exact->optimal_value} << ")";
    } else {
      std::cout << "infeasible";
    }
    std::cout << ", " << FormatDuration(ElapsedMs(exact_start)) << "\n";
  }
  int root_lb;
  if (method == BoundMethod::kExact) {
    if (!root_exact.has_value()) {
      std::cout << "Root infeasible under the exact bound\n";
      return 1;
    }
    root_lb = root_exact->optimal_value;
    // Candidates come from the exact tour rather than the LP support. The
    // tour edges are in original states with the merged (mapped) weight, so
    // the mapped edge is recovered through the LP's remap, which was built
    // from the same state.
    std::vector<TarelSupportEdge> tour_support;
    for (const TarelEdge& e : root_exact->tour_edges) {
      TarelEdge mapped{
          .origin = root->remap.original_to_mapped.at(e.origin),
          .destination = root->remap.original_to_mapped.at(e.destination),
          .weight = e.weight,
      };
      tour_support.push_back(TarelSupportEdge{e, mapped, 1.0});
    }
    root->support = std::move(tour_support);
    std::cout << "Candidates come from the exact tour's "
              << root->support.size() << " edges\n";
  } else {
    root_lb = root->lower_bound;
  }
  std::cout << "Scoring lifts against the " << bound_str << " bound " << root_lb
            << "\n";
  if (root->num_forbidden_support_edges > 0) {
    std::cout << "WARNING: " << root->num_forbidden_support_edges
              << " forbidden edges in the LP support\n";
  }
  if (method == BoundMethod::kLp) {
    double inter_stop_mass = 0.0;
    for (const TarelSupportEdge& s : root->support) {
      inter_stop_mass += s.x;
    }
    std::cout << "Root LP mass: " << std::fixed << std::setprecision(3)
              << inter_stop_mass << " on inter-stop edges, "
              << root->cycle_edge_mass << " on cycle edges (a tour uses "
              << root->expected_num_cycle_edges << ")\n";
  }
  if (print_support) {
    std::cout
        << (method == BoundMethod::kExact ? "Root exact tour:\n"
                                          : "Root LP support:\n");
    for (const TarelSupportEdge& s : root->support) {
      std::cout << "  x=" << std::fixed << std::setprecision(3) << s.x << "  "
                << s.edge.Debug(state) << "\n";
    }
  }

  std::vector<Candidate> candidates;

  if (branches != "times") {
    StepPathsAdjacencyList completed = state.ComputeCompletedGraph();
    std::vector<Candidate> edges = EdgeCandidates(state, completed, *root);
    std::cout << "Unique primitive edges: " << edges.size() << "\n";
    for (const Candidate& c : edges) {
      std::cout << "  " << c.label << "  (mass " << std::fixed
                << std::setprecision(2) << c.support_mass << ", in "
                << c.support_count << " support edges)\n";
    }
    if (max_edges >= 0 && static_cast<int>(edges.size()) > max_edges) {
      edges.resize(max_edges);
    }
    candidates.insert(candidates.end(), edges.begin(), edges.end());
    std::cout << "\n";
  }

  if (branches != "edges") {
    CriticalTimeCandidates times = TimeCandidates(state, *root);
    std::cout << "Distinct critical time sets: " << times.candidates.size()
              << " (skipped " << times.num_flex_origin_edges
              << " flex-origin tarel edges, " << times.num_unrealizable_sets
              << " sets with no matching minimal step)\n";
    for (const Candidate& c : times.candidates) {
      std::cout << "  " << c.label << "  (mass " << std::fixed
                << std::setprecision(2) << c.support_mass << ")\n";
    }
    if (max_times >= 0 &&
        static_cast<int>(times.candidates.size()) > max_times) {
      times.candidates.resize(max_times);
    }
    candidates.insert(
        candidates.end(), times.candidates.begin(), times.candidates.end()
    );
    std::cout << "\n";
  }

  // Strong branching.
  std::vector<BranchEvaluation> evaluations;
  int n = candidates.size();
  for (int i = 0; i < n; ++i) {
    const Candidate& c = candidates[i];
    ChildResult left = EvaluateChild(state, c.left, ub, method);
    ChildResult right = EvaluateChild(state, c.right, ub, method);
    evaluations.push_back(BranchEvaluation{c, left, right});
    const BranchEvaluation& e = evaluations.back();

    std::cout << "[" << (i + 1) << "/" << n << "] " << c.label << "\n";
    std::cout << "  " << c.left_name << ": lift "
              << FormatLift(e.LeftLift(root_lb)) << " ("
              << FormatChild(left, ub) << ")"
              << "  " << c.right_name << ": lift "
              << FormatLift(e.RightLift(root_lb)) << " ("
              << FormatChild(right, ub) << ")"
              << "  score " << FormatScore(e.Score(root_lb)) << "\n";
    PrintTop(evaluations, root_lb, top_n);
    std::cout << std::flush;
  }

  std::cout << "\nTotal time: " << FormatDuration(ElapsedMs(root_start))
            << "\n";
  return 0;
}
