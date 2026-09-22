#pragma once

#include <filesystem>
#include <fstream>

#include <fpvsim/sim/snapshot.hpp>

namespace fpvsim::log {

// Truth log writer; used from the I/O thread only
class TruthCsv {
 public:
  explicit TruthCsv(const std::filesystem::path& path);
  void write(const sim::Snapshot& s);
  void flush();

 private:
  std::ofstream file_;
};

}  // namespace fpvsim::log
