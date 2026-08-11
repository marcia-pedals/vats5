#pragma once

#include <cassert>
#include <chrono>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace vats5 {

// A hierarchical wall-clock timing trace.
//
// A Trace has a root span covering its whole lifetime; TraceSpan opens nested
// spans under whatever span is currently open. Spans must be closed in LIFO
// order, which scoping them takes care of. A span closes even when the scope is
// left by an exception, so the trace of a run that timed out still describes
// everything that ran up to the timeout.
//
//   Trace trace("iterative_expansion");
//   for (...) {
//     TraceSpan span(trace, "iteration " + std::to_string(iteration));
//     span.SetMetadata("target_stops", subset.size());
//     ...
//   }
//   trace.ToJson();
//
// Serialized as a tree of {name, start_seconds, duration_seconds, metadata?,
// children?}, where start_seconds is relative to the start of the root span.
class Trace {
 public:
  explicit Trace(std::string root_name) {
    start_ = std::chrono::steady_clock::now();
    open_.push_back(Node{.name = std::move(root_name), .start_seconds = 0.0});
  }

  Trace(const Trace&) = delete;
  Trace& operator=(const Trace&) = delete;

  // Closes the root span (at the time of the first call) and returns the tree.
  nlohmann::json ToJson() {
    assert(open_.size() == 1 && "a span is still open");
    if (!root_.has_value()) {
      root_ = Close(std::move(open_.back()));
    }
    return ToJson(*root_);
  }

 private:
  friend class TraceSpan;

  struct Node {
    std::string name;
    double start_seconds;
    double duration_seconds = 0.0;
    nlohmann::json metadata = nlohmann::json::object();
    std::vector<Node> children;
  };

  double ElapsedSeconds() const {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - start_
    )
        .count();
  }

  void Begin(std::string name) {
    open_.push_back(
        Node{.name = std::move(name), .start_seconds = ElapsedSeconds()}
    );
  }

  void End() {
    assert(open_.size() > 1 && "no span is open");
    Node node = Close(std::move(open_.back()));
    open_.pop_back();
    open_.back().children.push_back(std::move(node));
  }

  Node Close(Node node) const {
    node.duration_seconds = ElapsedSeconds() - node.start_seconds;
    return node;
  }

  static nlohmann::json ToJson(const Node& node) {
    nlohmann::json j{
        {"name", node.name},
        {"start_seconds", node.start_seconds},
        {"duration_seconds", node.duration_seconds},
    };
    if (!node.metadata.empty()) {
      j["metadata"] = node.metadata;
    }
    if (!node.children.empty()) {
      nlohmann::json children = nlohmann::json::array();
      for (const Node& child : node.children) {
        children.push_back(ToJson(child));
      }
      j["children"] = std::move(children);
    }
    return j;
  }

  std::chrono::steady_clock::time_point start_;

  // Spans from the root down to the innermost currently-open span. A span's
  // children are attached when it closes, so only these are still mutable.
  std::vector<Node> open_;

  std::optional<Node> root_;
};

// Opens a span for the lifetime of the object. See Trace.
class TraceSpan {
 public:
  TraceSpan(Trace& trace, std::string name) : trace_(trace) {
    trace_.Begin(std::move(name));
    depth_ = trace_.open_.size();
  }

  ~TraceSpan() {
    assert(trace_.open_.size() == depth_ && "spans closed out of order");
    trace_.End();
  }

  TraceSpan(const TraceSpan&) = delete;
  TraceSpan& operator=(const TraceSpan&) = delete;

  // Attaches a value describing what this span did, e.g. its problem size.
  template <typename T>
  void SetMetadata(const std::string& key, T&& value) {
    assert(trace_.open_.size() == depth_ && "span is not the innermost one");
    trace_.open_.back().metadata[key] = std::forward<T>(value);
  }

 private:
  Trace& trace_;
  size_t depth_;
};

}  // namespace vats5
