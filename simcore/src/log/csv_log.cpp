#include <fpvsim/log/csv_log.hpp>

#include <stdexcept>

namespace fpvsim::log {

TruthCsv::TruthCsv(const std::filesystem::path& path) : file_(path) {
  if (!file_) {
    throw std::runtime_error("cannot open log file " + path.string());
  }
  file_ << "t_s,north_m,east_m,down_m,vn_mps,ve_mps,vd_mps,qw,qx,qy,qz,p_radps,q_radps,r_radps,"
           "m1,m2,m3,m4,rc_a_us,rc_e_us,rc_t_us,rc_r_us,crashed,"
           "rpm1,rpm2,rpm3,rpm4,i1,i2,i3,i4,vbat_v,ibat_a,soc\n";
}

void TruthCsv::write(const sim::Snapshot& s) {
  const auto write_all = [this](const auto& values, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
      file_ << ',' << values[i];
    }
  };
  file_ << static_cast<double>(s.sim_time_ns) * 1e-9;
  write_all(s.position_ned, 3);
  write_all(s.velocity_ned, 3);
  write_all(s.q_ned_from_frd, 4);
  write_all(s.angular_rate_frd, 3);
  write_all(s.motor_command, 4);
  write_all(s.rc_channels_us, 4);
  file_ << ',' << (s.crashed ? 1 : 0);
  write_all(s.motor_rpm, 4);
  write_all(s.motor_current_a, 4);
  file_ << ',' << s.battery_voltage_v << ',' << s.battery_current_a << ',' << s.battery_soc << '\n';
}

void TruthCsv::flush() { file_.flush(); }

}  // namespace fpvsim::log
