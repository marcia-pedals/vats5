#pragma once

#include <functional>
#include <variant>

namespace vats5 {

struct TarelSolve {
  int vertex_count;
  int edge_count;
  int concorde_ms;
  bool feasible;
};

// The search proved that no tour is shorter than `lb` seconds. Emitted only
// when the bound improves on the previously emitted one.
struct NewLowerBound {
  int lb;
};

// The search found a tour of `ub` seconds, better than any found before.
struct NewUpperBound {
  int ub;
};

using SearchEvent = std::variant<TarelSolve, NewLowerBound, NewUpperBound>;

using SearchEventCallback = std::function<void(const SearchEvent&)>;

}  // namespace vats5
