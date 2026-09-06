#pragma once

#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace vats5 {

// Per-solve scratch directory for Concorde's checkpoint files and log.
// Removed on destruction.
class WorkDir {
 public:
  WorkDir() {
    // Create under concorde_work/ in cwd so it works in Claude sandbox (which
    // restricts /tmp) and doesn't clutter the cwd.
    if (mkdir("concorde_work", 0755) != 0 && errno != EEXIST) {
      throw std::runtime_error("Failed to create concorde_work directory");
    }
    std::string temp_dir = "concorde_work/vats5_tsp_XXXXXX";
    if (mkdtemp(temp_dir.data()) == nullptr) {
      throw std::runtime_error("Failed to create temp directory");
    }
    // Absolute, because the shim chdirs into it and must be able to find it
    // regardless of cwd.
    path_ = std::filesystem::absolute(temp_dir).string();
  }

  ~WorkDir() {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  WorkDir(const WorkDir&) = delete;
  WorkDir& operator=(const WorkDir&) = delete;

  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

inline std::string ReadFile(const std::string& path) {
  std::ifstream in(path);
  return std::string(
      std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()
  );
}

}  // namespace vats5
