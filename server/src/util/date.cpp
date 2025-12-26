#include "util/date.h"

#include <chrono>
#include <iomanip>
#include <sstream>

namespace vats5 {

std::string OffsetDate(const std::string& date, int days) {
  int y = std::stoi(date.substr(0, 4));
  unsigned m = std::stoi(date.substr(4, 2));
  unsigned d = std::stoi(date.substr(6, 2));

  std::chrono::year_month_day ymd{
      std::chrono::year{y}, std::chrono::month{m}, std::chrono::day{d}
  };
  auto sys_days = std::chrono::sys_days{ymd} + std::chrono::days{days};
  std::chrono::year_month_day result{sys_days};

  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(4) << int(result.year()) << std::setw(2)
      << unsigned(result.month()) << std::setw(2) << unsigned(result.day());
  return oss.str();
}

}  // namespace vats5
