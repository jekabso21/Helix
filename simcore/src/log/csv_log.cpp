#include <fpvsim/log/csv_log.hpp>

#include <chrono>
#include <stdexcept>

namespace fpvsim::log {

TruthLog::TruthLog(const std::filesystem::path& path) : file_(path) {
  if (!file_) {
    throw std::runtime_error("cannot open log file " + path.string());
  }
  file_ << "t_s,north_m,east_m,down_m,vn_mps,ve_mps,vd_mps,qw,qx,qy,qz,p_radps,q_radps,r_radps,"
           "m1,m2,m3,m4,rc_a_us,rc_e_us,rc_t_us,rc_r_us,crashed\n";
  writer_ = std::jthread([this](const std::stop_token& stop) { run(stop); });
}

TruthLog::~TruthLog() {
  writer_.request_stop();
  writer_.join();
}

void TruthLog::push(const TruthRow& row) noexcept {
  if (!queue_.try_push(row)) {
    ++dropped_;
  }
}

void TruthLog::run(const std::stop_token& stop) {
  TruthRow row{};
  while (true) {
    while (queue_.try_pop(row)) {
      write(row);
    }
    if (stop.stop_requested()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  file_.flush();
}

void TruthLog::write(const TruthRow& row) {
  file_ << static_cast<double>(row.sim_time_ns) * 1e-9;
  const auto write_all = [this](const auto& values) {
    for (const auto v : values) {
      file_ << ',' << v;
    }
  };
  write_all(row.position_ned);
  write_all(row.velocity_ned);
  write_all(row.q_ned_from_frd);
  write_all(row.angular_rate_frd);
  write_all(row.motor_command);
  write_all(row.rc_aetr);
  file_ << ',' << (row.crashed ? 1 : 0) << '\n';
}

}  // namespace fpvsim::log
