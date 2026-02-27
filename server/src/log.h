#pragma once
#include <functional>
#include <format>
#include <iostream>
#include <string_view>

namespace vats5 {

using TextLogger = std::function<void(std::string_view)>;

inline TextLogger OstreamLogger(std::ostream& os) {
    return [&os](std::string_view msg) { os << msg << "\n"; };
}

inline TextLogger NullLogger() {
    return [](std::string_view) {};
}

}  // namespace vats5
