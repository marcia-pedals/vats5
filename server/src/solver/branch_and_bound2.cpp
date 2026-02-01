#include "solver/branch_and_bound2.h"
#include <algorithm>
#include <atomic>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include "solver/data.h"
#include "solver/step_merge.h"
#include "solver/steps_adjacency_list.h"
#include "solver/tarel_graph.h"

namespace vats5 {

std::vector<Path> FindSubpaths(const ProblemState& state, const std::vector<TarelEdge>& tour) {
  assert(tour.size() > 0);
  StopId tour_start = tour.begin()->origin.stop;
  StopId tour_end = (tour.end() - 1)->destination.stop;

  std::vector<Path> retired_subpaths;

  std::vector<Path> active_subpaths;
  // Correspond by index to active_subpaths, but unlike the usual thing where we
  // use the "merged_step" as a path's step, we use the step from the previous
  // tour stop. This is because different subpaths have different starting
  // points so we can't compare them by their whole "merged_step".
  std::vector<Step> active_steps;

  for (const TarelEdge& edge : tour) {
    std::span<const Path> next_paths = state.completed.PathsBetween(edge.origin.stop, edge.destination.stop);
    std::vector<Step> next_steps;
    next_steps.reserve(next_paths.size());
    for (const Path& path : next_paths) {
      next_steps.push_back(path.merged_step);
    }

    std::unordered_set<int> unused_active_step_indices;
    for (int i = 0; i < active_steps.size(); ++i) {
      unused_active_step_indices.insert(i);
    }
    std::unordered_set<int> unused_next_step_indices;
    for (int i = 0; i < next_steps.size(); ++i) {
      unused_next_step_indices.insert(i);
    }

    std::vector<StepProvenance> provenance;
    std::vector<Step> merged_active_steps = PairwiseMergedSteps(active_steps, next_steps, &provenance);

    std::vector<Path> new_active_subpaths;
    new_active_subpaths.reserve(next_paths.size() + provenance.size());
    std::vector<Step> new_active_steps;
    new_active_steps.reserve(next_paths.size() + provenance.size());

    // Add all next.
    for (int i = 0; i < next_steps.size(); ++i) {
      new_active_subpaths.push_back(next_paths[i]);
      new_active_steps.push_back(next_steps[i]);
    }

    // Extend.
    for (int i = 0; i < provenance.size(); ++i) {
      new_active_subpaths.push_back(Path{
        merged_active_steps[i],
        active_subpaths[provenance[i].ab_index].steps,
      });
      for (const Step& step : next_paths[provenance[i].bc_index].steps) {
        new_active_subpaths.back().steps.push_back(step);
      }

      const Step& active_step = active_steps[provenance[i].ab_index];
      Step next_step = next_steps[provenance[i].bc_index];
      if (next_step.is_flex && !active_step.is_flex) {
        int next_step_dur = next_step.FlexDurationSeconds();
        next_step.is_flex = false;
        next_step.origin.time = active_step.destination.time;
        next_step.destination.time = TimeSinceServiceStart{next_step.origin.time.seconds + next_step_dur};
      }

      new_active_steps.push_back(next_step);
      unused_active_step_indices.erase(provenance[i].ab_index);
      unused_next_step_indices.erase(provenance[i].bc_index);
    }

    // Retire retired paths.
    for (int index : unused_active_step_indices) {
      retired_subpaths.push_back(active_subpaths[index]);
    }

    // Make the active paths be sorted and minimal. This prefers existing paths
    // over the just-added paths because the sort is stable and the minimal
    // cover keeps the last whenever it deletes anything.
    SortSteps(new_active_steps, &new_active_subpaths);
    MakeMinimalCover(new_active_steps, &new_active_subpaths);
    active_steps = std::move(new_active_steps);
    active_subpaths = std::move(new_active_subpaths);
  }

  // Retire and normalize everything and return.
  for (Path& path : active_subpaths) {
    retired_subpaths.push_back(std::move(path));
  }
  for (Path& path : retired_subpaths) {
    NormalizeConsecutiveSteps(path.steps);
    path.merged_step = ConsecutiveMergedSteps(path.steps);
  }
  return retired_subpaths;
}

void DoTheThing(const ProblemState& state, const std::vector<TarelEdge>& tour) {
  if (tour.size() == 0) {
    return;
  }

  std::vector<StopId> tour_stops;
  tour_stops.push_back(tour[0].origin.stop);
  for (const TarelEdge& e : tour) {
    tour_stops.push_back(e.destination.stop);
  }

  for (int start_i = 0; start_i < tour_stops.size(); ++start_i) {
    int weight_before = 0;
    int weight_after = 0;
    for (int i = start_i; i < tour.size(); ++i) {
      weight_after += tour[i].weight;
    }

    StopId start_stop = tour_stops[start_i];
  }
}

Path FindWorstJump(const ProblemState& state, const std::vector<TarelEdge>& tour) {
  assert(tour.size() > 0);

  std::vector<StopId> tour_stops;
  tour_stops.push_back(tour[0].origin.stop);
  for (const TarelEdge& e : tour) {
    tour_stops.push_back(e.destination.stop);
  }

  std::vector<std::optional<int>> durs(tour_stops.size() * tour_stops.size(), std::nullopt);
  std::vector<std::optional<Path>> dur_paths(tour_stops.size() * tour_stops.size(), std::nullopt);

  for (int start_i = 0; start_i < tour_stops.size(); ++start_i) {
    int weight_before = 0;
    for (int i = 0; i < start_i; ++i) {
      weight_before += tour[i].weight;
    }

    StopId start_stop = tour_stops[start_i];
    std::vector<Step> cur_steps = {ZeroEdge(start_stop, start_stop)};
    std::vector<Path> cur_paths = {Path{ZeroEdge(start_stop, start_stop), {}}};
    for (int end_i = start_i; end_i < tour_stops.size(); ++end_i) {
      int weight_after = 0;
      for (int i = end_i; i < tour.size(); ++i) {
        weight_after += tour[i].weight;
      }

      auto min_it = std::min_element(cur_paths.begin(), cur_paths.end(), [](const Path& a, const Path& b) {
        return a.DurationSeconds() < b.DurationSeconds();
      });
      if (min_it == cur_paths.end()) {
        break;
      }
      durs[start_i * tour_stops.size() + end_i] = min_it->DurationSeconds() + weight_before + weight_after;
      dur_paths[start_i * tour_stops.size() + end_i] = *min_it;

      if (end_i + 1 < tour_stops.size()) {
        std::vector<Step> yo_steps;
        std::vector<Path> yo_paths;
        for (const Path& path: state.completed.PathsBetween(tour_stops[end_i], tour_stops[end_i + 1])) {
          // yo_steps.push_back(step);
          for (const TarelState &orig_dest : tour[end_i].original_destinations) {
            if (path.merged_step.destination.partition == orig_dest.partition) {
              yo_steps.push_back(path.merged_step);
              yo_paths.push_back(path);
              break;
            }
          }
        }
        std::vector<StepProvenance> provs;
        cur_steps = PairwiseMergedSteps(cur_steps, yo_steps, &provs);
        std::vector<Path> new_paths;
        for (int i = 0; i < cur_steps.size(); ++i) {
          new_paths.push_back(Path{
            cur_steps[i],
            cur_paths[provs[i].ab_index].steps
          });
          for (const Step& step : yo_paths[provs[i].bc_index].steps) {
            new_paths.back().steps.push_back(step);
          }
        }
        cur_paths = std::move(new_paths);
      }
    }
  }

  int max_stop_name_length = 0;
  for (int i = 0; i < tour_stops.size(); ++i) {
    max_stop_name_length = std::max(max_stop_name_length, static_cast<int>(state.StopName(tour_stops[i]).size()));
  }

  int max_partition_name_length = 0;
  for (int i = 0; i < tour.size(); ++i) {
    // TODO: ALL destinations.
    max_partition_name_length = std::max(max_partition_name_length, static_cast<int>(state.PartitionName(tour[i].original_destinations[0].partition).size()));
  }

  int col_width = 3;

  // Header row with numbers
  std::cout << std::string(col_width + 1 + max_stop_name_length + max_partition_name_length + 2, ' ');
  for (int i = 0; i < tour_stops.size(); ++i) {
    std::cout << " " << std::setw(col_width) << i;
  }
  std::cout << "\n";

  for (int i = 0; i < tour_stops.size(); ++i) {
    std::cout
      << std::setw(col_width) << i << " "
      << std::setw(max_stop_name_length) << state.StopName(tour_stops[i]) << " ";
    if (i < tour_stops.size() - 1) {
      // TODO: ALL destinations.
      std::cout << std::setw(max_partition_name_length) << state.PartitionName(tour[i].original_destinations[0].partition) << " ";
    } else {
      std::cout << std::setw(max_partition_name_length) << "" << " ";
    }
    for (int j = 0; j < tour_stops.size(); ++j) {
      std::optional<int> dur = durs[j * tour_stops.size() + i];
      if (dur.has_value()) {
        std::cout << " " << std::setw(col_width) << dur.value() / 60;
      } else {
        std::cout << " " << std::setw(col_width) << "";
      }
    }
    std::cout << "\n";
  }

  struct Jump {
    int jump;
    int start_i;
    int end_i;
    int prim_len;
  };
  std::vector<Jump> jumps;

  for (int start_i = 1; start_i < tour_stops.size(); ++start_i) {
    for (int end_i = start_i; end_i < tour_stops.size() - 2; ++end_i) {
      int at_end = durs[start_i * tour_stops.size() + end_i].value_or(std::numeric_limits<int>::max());
      int at_end_plus_1 = durs[start_i * tour_stops.size() + end_i + 1].value_or(std::numeric_limits<int>::max());
      int jump = at_end_plus_1 - at_end;
      jumps.push_back(Jump{
        .jump=jump,
        .start_i=start_i,
        .end_i=end_i,
        .prim_len=static_cast<int>(dur_paths[start_i * tour_stops.size() + end_i]->steps.size()),
      });
    }
  }

  std::ranges::sort(jumps, [](const Jump& a, const Jump& b) {
    if (a.jump == b.jump) {
      return a.start_i > b.start_i;
    }
    return a.jump > b.jump;
  });

  std::erase_if(jumps, [](const Jump& j) {
    return j.prim_len > 5;
  });

  if (jumps.size() > 0) {
    std::cout
      << "Worst jump " << jumps[0].jump / 60
      << ": " << state.StopName(tour_stops[jumps[0].start_i])
      << " -> " << state.StopName(tour_stops[jumps[0].end_i]) << "\n";
  } else {
    std::cout << "No jumps?!\n";
  }

  std::optional<Path> dur_path = dur_paths[jumps[0].start_i * tour_stops.size() + jumps[0].end_i];
  assert(dur_path.has_value());
  for (int i = 0; i < dur_path->steps.size(); ++i) {
    if (i == 0) {
      std::cout << state.StopName(dur_path->steps[i].origin.stop) << " -> ";
    } else {
      std::cout << " -> ";
    }
    std::cout << state.StopName(dur_path->steps[i].destination.stop);
  }
  std::cout << "\n";

  return dur_path.value();
}


ProblemState ApplyConstraint(
  const ProblemState& state,
  const ElaborateConstraint& constraint
) {
  assert(constraint.required_path.size() >= 1);

  // TODO: Allow START and END in constraints.
  for (StopId s : constraint.required_path) {
    assert(s != state.boundary.start);
    assert(s != state.boundary.end);
  }

  // Mutable copies of all the `ProblemState` fields we'll be mutating.
  std::vector<Step> steps = state.minimal.AllSteps();
  ProblemBoundary boundary = state.boundary;
  std::unordered_set<StopId> required_stops = state.required_stops;
  std::unordered_map<StopId, std::string> stop_names = state.stop_names;
  StopId next_stop_id{state.minimal.NumStops()};

  // Add a new stop "(required_path[0]->...->required_path[n])" representing the
  // act of going to "required_path[0]", following it all the way, and ending up
  // at "required_path[n]". Make this a new required stop, and remove all of
  // "required_path[i]" from the required stops.
  StopId required_path_stop;
  if (constraint.required_path.size() == 1) {
    // Special case: If the required path only has 1 stop, then we don't need to
    // do anything to make a new "required path stop".
    required_path_stop = constraint.required_path[0];
  } else {
    // General case: The required path has >1 stops.
    assert(constraint.required_path.size() > 1);
    required_path_stop = next_stop_id;
    next_stop_id.v += 1;
    {
      std::stringstream ss;
      ss << "(";
      for (int i = 0; i < constraint.required_path.size(); ++i) {
        if (i > 0) {
          ss << "->";
        }
        ss << state.StopName(constraint.required_path[i]);
      }
      ss << ")";
      stop_names[required_path_stop] = std::move(ss.str());
    }
    required_stops.insert(required_path_stop);
    for (StopId s : constraint.required_path) {
      required_stops.erase(s);
    }

    std::unordered_map<StopId, int> required_path_index;
    for (int i = 0; i < constraint.required_path.size(); ++i) {
      required_path_index[constraint.required_path[i]] = i;
    }

    // Collect some steps that we'll need for constructing the steps to and from
    // "required_path_stop".
    // `along_path[i]` are the steps from `required_path[i]` to `required_path[i+1]`.
    std::vector<std::vector<Step>> along_path(constraint.required_path.size() - 1);
    // Steps from * to `required_path[0]`, grouped by *.
    std::unordered_map<StopId, std::vector<Step>> star_to_start;
    // Steps from `required_path[n]` to *, not grouped by anything.
    std::vector<Step> end_to_star;
    for (const Step& step : steps) {
      // Collect into `along_path`.
      {
        auto origin_required_path_index_it = required_path_index.find(step.origin.stop);
        if (origin_required_path_index_it != required_path_index.end()) {
          int index = origin_required_path_index_it->second;
          if (
            index < constraint.required_path.size() - 1 &&
            step.destination.stop == constraint.required_path[index + 1]
          ) {
            along_path[index].push_back(step);
          }
        }
      }

      // Collect into `star_to_start`.
      if (step.destination.stop == *(constraint.required_path.begin())) {
        star_to_start[step.origin.stop].push_back(step);
      }

      // Collect into `end_to_star`.
      if (step.origin.stop == *(constraint.required_path.end() - 1)) {
        end_to_star.push_back(step);
      }
    }

    // Sorted and minimal should have been preserved in all the things we've
    // collected, but assert it to make sure.
    for (const std::vector<Step>& along_path_element : along_path) {
      assert(CheckSortedAndMinimal(along_path_element));
    }
    for (const auto& [star, star_to_start_steps] : star_to_start) {
      assert(CheckSortedAndMinimal(star_to_start_steps));
    }

    std::vector<Step> merged_along_path = along_path[0];
    for (int i = 1; i < along_path.size(); ++i) {
      merged_along_path = PairwiseMergedSteps(merged_along_path, along_path[i]);
    }

    // The steps to "(required_path[0]->...->required_path[n])" are the steps
    // "x->required_path[0]" merged with the required path.
    for (const auto& [x, x_to_start] : star_to_start) {
      for (Step step : PairwiseMergedSteps(x_to_start, merged_along_path)) {
        step.destination.stop = required_path_stop;
        steps.push_back(step);
      }
    }

    // The steps from "(required_path[0]->...->required_path[n])" are the steps
    // "required_path[n]->x".
    for (Step step : end_to_star) {
      step.origin.stop = required_path_stop;
      steps.push_back(step);
    }
  }

  // Ok now forbid steps from the required path to the "forbid next".
  if (constraint.forbid_next.has_value()) {
    StopId forbid_next = constraint.forbid_next.value();
    std::erase_if(steps, [&](const Step& step) {
      return step.origin.stop == required_path_stop && step.destination.stop == forbid_next;
    });
  }

  // if (constraint.forbid_next.has_value()) {
  //   StopId forbid_next = constraint.forbid_next.value();
  //   if (constraint.required_path.size() == 1) {
  //     // Special case: If required path only has 1 stop, just forbid steps from
  //     // there to "forbid next".
  //     // TODO: Hmm think, maybe I do also need required_path[0]' thingy for this.
  //     StopId required_stop = constraint.required_path[0];
  //     std::erase_if(steps, [required_stop, forbid_next](const Step& s) {
  //       return s.origin.stop == required_stop && s.destination.stop == forbid_next;
  //     });
  //   } else {
  //     // General case: Required path has >1 stops.
  //     assert(constraint.required_path.size() > 1);

  //     // For each "required_path[i]", add a new stop "required_path[i]'"
  //     // representing that we are not merely repeating part of the required path
  //     // after having visited it.
  //     std::vector<StopId> required_path_prime;
  //     std::unordered_map<StopId, StopId> to_prime;
  //     required_path_prime.reserve(constraint.required_path.size());
  //     for (StopId s : constraint.required_path) {
  //       StopId s_prime = next_stop_id;
  //       next_stop_id.v += 1;
  //       to_prime[s] = s_prime;
  //       stop_names[s_prime] = stop_names[s] + "'";
  //     }

  //     auto PrimeIfInRequiredPath = [&to_prime](StopId s) -> StopId {
  //       auto it = to_prime.find(s);
  //       return it == to_prime.end() ? s : it->second;
  //     };

  //     // Add all incoming and outgoing steps to the required path prime.
  //     for (const Step& step : steps) {
  //       if (step.origin.stop == required_path_stop) {
  //         continue;
  //       }

  //       Step step_prime = step;
  //       step_prime.origin.stop = PrimeIfInRequiredPath(step_prime.origin.stop);
  //       step_prime.destination.stop = PrimeIfInRequiredPath(step_prime.destination.stop);
  //       if (step_prime != step) {
  //         steps.push_back(step_prime);
  //       }
  //     }

  //     // Forbid steps from the required path to "forbid next".
  //     std::erase_if(steps, [&](const Step& step) {
  //       return (
  //         (step.origin.stop == required_path_stop || step.origin.stop == *(constraint.required_path.end() - 1)) &&
  //         step.destination.stop == forbid_next
  //       );
  //     });
  //   }
  // }

  // Build the new problem state from the stuff we've been mutating.
  return MakeProblemState(
    MakeAdjacencyList(steps),
    std::move(boundary),
    std::move(required_stops),
    std::move(stop_names),
    state.step_partition_names
  );
}

std::vector<std::pair<ProblemState, int>> MakeStates2(
  const ProblemState& base,
  const std::vector<StopId>& required_path,
  std::optional<int> ub_hint
) {
  std::vector<std::pair<ProblemState, int>> result;

  if (required_path.empty()) {
    return result;
  }

  // First pass: generate all branch states and their description strings.
  struct WorkItem {
    ProblemState state;
    std::string description;
  };
  std::vector<WorkItem> work_items;

  for (size_t i = 1; i <= required_path.size(); ++i) {
    ElaborateConstraint constraint;
    constraint.required_path = std::vector<StopId>(required_path.begin(), required_path.begin() + i);
    if (i < required_path.size()) {
      constraint.forbid_next = required_path[i];
    } else {
      constraint.forbid_next = std::nullopt;
    }

    ProblemState branch_state = ApplyConstraint(base, constraint);

    // Build description string.
    std::stringstream ss;
    ss << base.StopName(required_path[0]);
    for (size_t j = 1; j < i; ++j) {
      ss << "->" << base.StopName(required_path[j]);
    }
    if (i < required_path.size()) {
      ss << "-!" << base.StopName(required_path[i]);
    }

    work_items.push_back({std::move(branch_state), ss.str()});
  }

  // Prepare results vector.
  std::vector<int> lbs(work_items.size());

  // Parallel LB computation.
  std::atomic<size_t> next_work_index{0};
  std::mutex cout_mutex;

  auto worker = [&]() {
    while (true) {
      size_t idx = next_work_index.fetch_add(1);
      if (idx >= work_items.size()) {
        break;
      }

      std::optional<TspTourResult> lb_result = ComputeTarelLowerBound(work_items[idx].state, ub_hint);
      int lb = lb_result.has_value() ? lb_result->optimal_value : std::numeric_limits<int>::max();
      lbs[idx] = lb;

      {
        std::lock_guard<std::mutex> lock(cout_mutex);
        std::cout << work_items[idx].description << ": " << TimeSinceServiceStart{lb}.ToString() << "\n";
      }
    }
  };

  unsigned int num_threads = std::thread::hardware_concurrency();
  if (num_threads == 0) num_threads = 1;

  std::vector<std::thread> threads;
  for (unsigned int i = 0; i < num_threads; ++i) {
    threads.emplace_back(worker);
  }

  for (auto& t : threads) {
    t.join();
  }

  // Build final result.
  for (size_t i = 0; i < work_items.size(); ++i) {
    result.push_back({std::move(work_items[i].state), lbs[i]});
  }

  return result;
}

struct SearchNode2 {
  int lb;
  std::unique_ptr<ProblemState> state;

  bool operator<(const SearchNode2& other) const {
    // Min-heap: lower lb has higher priority
    return lb > other.lb;
  }
};

int BranchAndBoundSolve2(
  const ProblemState& initial_state,
  std::ostream* search_log,
  std::optional<std::string> run_dir,
  int max_iter
) {
  std::priority_queue<SearchNode2> q;
  q.push(SearchNode2{0, std::make_unique<ProblemState>(initial_state)});

  int iter_num = 0;
  int best_ub = std::numeric_limits<int>::max();

  while (!q.empty()) {
    if (max_iter > 0 && iter_num >= max_iter) {
      throw std::runtime_error("Exceeded max_iter");
    }
    iter_num += 1;

    SearchNode2 cur_node = std::move(const_cast<SearchNode2&>(q.top()));
    q.pop();
    ProblemState& state = *cur_node.state;

    if (search_log != nullptr) {
      *search_log << iter_num << " (" << q.size() + 1 << " active nodes) Take " << TimeSinceServiceStart{cur_node.lb}.ToString() << "\n";
    }

    if (cur_node.lb >= best_ub) {
      if (search_log != nullptr) {
        *search_log << "Search terminated: LB >= UB\n";
      }
      return best_ub;
    }

    // Compute lower bound.
    std::optional<TspTourResult> lb_result_opt = ComputeTarelLowerBound(
      state,
      best_ub < std::numeric_limits<int>::max() ? std::make_optional(best_ub) : std::nullopt
    );
    if (!lb_result_opt.has_value()) {
      if (search_log != nullptr) {
        *search_log << "  infeasible\n";
      }
      continue;
    }
    TspTourResult& lb_result = lb_result_opt.value();

    // Write iteration data to run_dir if specified.
    if (run_dir.has_value()) {
      std::string iter_dir = run_dir.value() + "/iter" + std::to_string(iter_num);
      mkdir(iter_dir.c_str(), 0755);
      nlohmann::json output = {
        {"state", state},
        {"lb_result", lb_result}
      };
      std::ofstream out(iter_dir + "/subpaths.json");
      out << output.dump();
    }

    std::vector<Path> subpaths = FindSubpaths(state, lb_result.tour_edges);

    if (lb_result.optimal_value >= best_ub) {
      if (search_log != nullptr) {
        *search_log << "  pruned: LB (" << TimeSinceServiceStart{lb_result.optimal_value}.ToString() << ") >= UB (" << TimeSinceServiceStart{best_ub}.ToString() << ")\n";
      }
      continue;
    }
    if (search_log != nullptr) {
      *search_log << "  lb: " << TimeSinceServiceStart{lb_result.optimal_value}.ToString() << "\n";
    }

    // Make an upper bound by actually following the LB path.
    std::vector<Step> feasible_steps;
    feasible_steps.push_back(Step::PrimitiveFlex(state.boundary.start, state.boundary.start, 0, TripId::NOOP));
    for (int i = 0; i < lb_result.tour_edges.size(); ++i) {
      TarelEdge& edge = lb_result.tour_edges[i];
      std::vector<Step> next_steps;
      for (const Path& p : state.completed.PathsBetween(edge.origin.stop, edge.destination.stop)) {
        next_steps.push_back(p.merged_step);
      }
      feasible_steps = PairwiseMergedSteps(feasible_steps, next_steps);
    }
    std::erase_if(feasible_steps, [](const Step& step) {
      return step.origin.time < TimeSinceServiceStart{0};
    });
    auto best_feasible_step_it = std::min_element(feasible_steps.begin(), feasible_steps.end(), [](const Step& a, const Step& b) {
      return a.DurationSeconds() < b.DurationSeconds();
    });
    if (best_feasible_step_it != feasible_steps.end() && best_feasible_step_it->DurationSeconds() < best_ub) {
      best_ub = best_feasible_step_it->DurationSeconds();
      if (search_log != nullptr) {
        *search_log << "  found new ub " << TimeSinceServiceStart{best_ub}.ToString() << "\n";
      }
    }

    // // Get primitive steps from the LB tour.
    // std::vector<Step> primitive_steps;
    // for (const TarelEdge& e : lb_result.tour_edges) {
    //   const auto& paths = state.completed.PathsBetween(e.origin.stop, e.destination.stop);
    //   assert(paths.size() > 0);
    //   Path best = paths[0];
    //   for (const Path& p : paths) {
    //     if (p.DurationSeconds() < best.DurationSeconds()) {
    //       best = p;
    //     }
    //   }
    //   for (const Step& s : best.steps) {
    //     primitive_steps.push_back(s);
    //   }
    // }

    // if (primitive_steps.size() <= 2) {
    //   if (search_log != nullptr) {
    //     *search_log << "  pruned: 2 or fewer primitive steps\n";
    //   }
    //   continue;
    // }

    // // Remove a random number of steps from start and end.
    // // At least 1 from each end (so START and END are never included).
    // // Must keep at least 1 step remaining.
    // {
    //   static thread_local std::mt19937 rng(std::random_device{}());
    //   int n = primitive_steps.size();
    //   // remove_start in [1, n-2], remove_end in [1, n-1-remove_start]
    //   std::uniform_int_distribution<int> start_dist(1, n - 2);
    //   int remove_start = start_dist(rng);
    //   std::uniform_int_distribution<int> end_dist(1, n - 1 - remove_start);
    //   int remove_end = end_dist(rng);
    //   primitive_steps.erase(primitive_steps.end() - remove_end, primitive_steps.end());
    //   primitive_steps.erase(primitive_steps.begin(), primitive_steps.begin() + remove_start);
    // }

    Path worst_jump = FindWorstJump(state, lb_result.tour_edges);
    std::vector<Step> primitive_steps = worst_jump.steps;
    assert(primitive_steps.size() > 0);
    assert(primitive_steps.front().origin.stop != state.boundary.start);
    assert(primitive_steps.back().destination.stop != state.boundary.end);

    // Build the required_path from the primitive steps (the sequence of stops).
    std::vector<StopId> required_path;
    required_path.push_back(primitive_steps[0].origin.stop);
    for (const Step& step : primitive_steps) {
      required_path.push_back(step.destination.stop);
    }

    if (search_log != nullptr) {
      *search_log << "  branching on path: ";
      for (size_t i = 0; i < required_path.size(); ++i) {
        if (i > 0) *search_log << " -> ";
        *search_log << state.StopName(required_path[i]);
      }
      *search_log << "\n";
    }

    // Create and evaluate branches in parallel.
    std::optional<int> ub_hint = best_ub < std::numeric_limits<int>::max() ? std::make_optional(best_ub) : std::nullopt;
    for (auto& [branch_state, branch_lb] : MakeStates2(state, required_path, ub_hint)) {
      if (branch_lb < best_ub) {
        q.push(SearchNode2{branch_lb, std::make_unique<ProblemState>(std::move(branch_state))});
      }
    }
  }

  return best_ub;
}

int DummyFunction() {
  return 42;
}

}  // namespace vats5
