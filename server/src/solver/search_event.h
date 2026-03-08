#pragma once

#include <functional>
#include <variant>

namespace vats5 {

struct TarelSolve {
  int vertex_count;
  int edge_count;
  int concorde_ms;
};

using SearchEvent = std::variant<TarelSolve>;

using SearchEventCallback = std::function<void(const SearchEvent&)>;

}  // namespace vats5
