#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>

namespace vats5 {

// Records a benchmark's bound events as JSON Lines, one object per line,
// flushed as they happen so that everything written so far survives the
// process being killed at a time limit. Lines are one of
//
//   {"type": "lb", "t": <seconds>, "lb": <seconds of tour duration>}
//   {"type": "ub", "t": <seconds>, "ub": <seconds of tour duration>}
//   {"type": "converged", "t": <seconds>}
//
// where "t" is seconds since Start(). experiments/measure_solver.py reads
// these. Without a path, every method is a no-op.
class BenchmarkEventLog {
 public:
  explicit BenchmarkEventLog(const std::optional<std::string>& path) {
    if (!path.has_value()) {
      return;
    }
    out_.emplace(*path);
    if (!out_->is_open()) {
      throw std::runtime_error("could not open events file " + *path);
    }
  }

  // Marks the origin of the events' timestamps. Call right before solving.
  void Start() { start_ = std::chrono::steady_clock::now(); }

  void LowerBound(int lb) { Write({{"type", "lb"}, {"lb", lb}}); }

  void UpperBound(int ub) { Write({{"type", "ub"}, {"ub", ub}}); }

  void Converged() { Write({{"type", "converged"}}); }

 private:
  void Write(nlohmann::json event) {
    if (!out_.has_value()) {
      return;
    }
    std::chrono::duration<double> elapsed =
        std::chrono::steady_clock::now() - start_;
    event["t"] = elapsed.count();
    *out_ << event.dump() << "\n";
    out_->flush();
  }

  std::optional<std::ofstream> out_;
  std::chrono::steady_clock::time_point start_ =
      std::chrono::steady_clock::now();
};

}  // namespace vats5
