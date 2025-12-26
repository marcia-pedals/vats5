#pragma once

#include <string>

namespace vats5 {

// Offsets a date in YYYYMMDD format by the given number of days.
// Positive days moves forward, negative days moves backward.
std::string OffsetDate(const std::string& date, int days);

}  // namespace vats5
