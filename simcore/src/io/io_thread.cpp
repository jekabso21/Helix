#include <fpvsim/io/io_thread.hpp>

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Geometry>

#include <fpvsim/api/protocol.hpp>
#include <fpvsim/config/session.hpp>
#include <fpvsim/frames.hpp>
#include <fpvsim/input/mapping_json.hpp>
#include <fpvsim/log/csv_log.hpp>
#include <fpvsim/net/udp_socket.hpp>
#include <fpvsim/proto/render_state.hpp>

namespace fpvsim::io {

namespace {

constexpr int kPollTimeoutMs = 1;
constexpr std::size_t kMaxLineBytes = std::size_t{64} * 1024;

struct Subscription {
  std::int64_t id;
  std::int64_t every_ticks;
  std::string topic;
};

struct Client {
  int fd;
  std::uint32_t id;
  std::string inbuf;
  std::vector<Subscription> subscriptions;
};

int listen_tcp(const std::string& host, std::uint16_t port) {
  const int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
  }
  const int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  const sockaddr_in address = net::make_address(host, port);
  if (bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0 ||
      listen(fd, 8) != 0) {
    const std::string error = std::strerror(errno);
    close(fd);
    throw std::runtime_error("control API bind " + host + ":" + std::to_string(port) + ": " +
                             error);
  }
  return fd;
}

nlohmann::json motors_json(const sim::Snapshot& s) {
  nlohmann::json motors = nlohmann::json::array();
  for (std::size_t i = 0; i < s.motor_count; ++i) {
    motors.push_back({{"bf_index", i + 1},
                      {"command", s.motor_command[i]},
                      {"rpm", s.motor_rpm[i]},
                      {"current_a", s.motor_current_a[i]},
                      {"thrust_n", s.motor_thrust_n[i]}});
  }
  return motors;
}

nlohmann::json state_json(const sim::Snapshot& s) {
  return {{"sim_time_ns", s.sim_time_ns},
          {"position_ned", s.position_ned},
          {"q_ned_from_frd", s.q_ned_from_frd},
          {"velocity_ned", s.velocity_ned},
          {"angular_rate_frd", s.angular_rate_frd},
          {"motors", motors_json(s)},
          {"armed", s.armed},
          {"crashed", s.crashed},
          {"touching", s.touching},
          {"paused", s.paused},
          {"air_density_kg_m3", s.air_density_kg_m3},
          {"loop",
           {{"step_time_mean_us", s.step_time_mean_us},
            {"step_time_p99_us", s.step_time_p99_us},
            {"step_time_max_us", s.step_time_max_us},
            {"overruns", s.overruns}}},
          {"error_counters", {{"malformed_motor_packets", s.malformed_packets}}},
          {"motor_packets", s.motor_packets}};
}

nlohmann::json telemetry_json(const sim::Snapshot& s, std::int64_t physics_rate_hz,
                              const std::string& input_source) {
  const Eigen::Quaterniond q(s.q_ned_from_frd[0], s.q_ned_from_frd[1], s.q_ned_from_frd[2],
                             s.q_ned_from_frd[3]);
  const frames::EulerZyx euler = frames::euler_zyx_from_q(q);
  const double ground_speed = std::hypot(s.velocity_ned[0], s.velocity_ned[1]);
  const double air_speed = std::hypot(s.velocity_ned[0], s.velocity_ned[1], s.velocity_ned[2]);
  return {{"flight",
           {{"altitude_agl_m", -s.position_ned[2]},
            {"ground_speed_mps", ground_speed},
            {"air_speed_mps", air_speed},
            {"climb_mps", -s.velocity_ned[2]},
            {"roll_rad", euler.roll_rad},
            {"pitch_rad", euler.pitch_rad},
            {"heading_rad", euler.yaw_rad},
            {"rates_frd_radps", s.angular_rate_frd}}},
          {"battery",
           {{"voltage_v", s.battery_voltage_v},
            {"current_a", s.battery_current_a},
            {"consumed_mah", s.battery_consumed_mah},
            {"soc", s.battery_soc},
            {"cutoff", s.battery_cutoff}}},
          {"motors", motors_json(s)},
          {"input",
           {{"source", input_source},
            {"channels_us", s.rc_channels_us},
            {"armed_switch", s.rc_channels_us[4] > 1500},
            {"device_connected", s.device_connected}}},
          {"sim",
           {{"mode", "realtime"},
            {"physics_rate_hz", physics_rate_hz},
            {"paused", s.paused},
            {"crashed", s.crashed},
            {"overruns", s.overruns},
            {"error_counters", {{"malformed_motor_packets", s.malformed_packets}}},
            {"active_failures", nlohmann::json::array()}}},
          {"link", nullptr}};
}

proto::RenderState render_state_from(const sim::Snapshot& s) {
  proto::RenderState r{};
  r.sim_time_ns = s.sim_time_ns;
  r.position_ned = s.position_ned;
  r.q_ned_from_frd = s.q_ned_from_frd;
  r.velocity_ned = s.velocity_ned;
  r.angular_rate_frd = s.angular_rate_frd;
  r.armed = s.armed ? 1 : 0;
  r.crashed = s.crashed ? 1 : 0;
  r.motor_count = s.motor_count;
  for (std::size_t i = 0; i < proto::kRenderStateMotors; ++i) {
    r.motor_rpm[i] = static_cast<float>(s.motor_rpm[i]);
  }
  r.sun_dir_ned = {0.0F, 0.0F, -1.0F};
  r.sun_intensity = 1.0F;
  return r;
}

}  // namespace

class IoThread::Impl {
 public:
  Impl(input::InputControl& input_control, sim::ModelControl& model_control, IoConfig config,
       SnapshotQueue& snapshots, CommandQueue& commands, ResultQueue& results)
      : input_control_(input_control),
        model_control_(model_control),
        config_(std::move(config)),
        snapshots_(snapshots),
        commands_(commands),
        results_(results),
        render_socket_(net::UdpSocket::unbound()),
        render_address_(net::make_address(config_.app_host, config_.app_port)),
        truth_(config_.truth_csv),
        listen_fd_(listen_tcp(config_.api_host, config_.api_port)),
        render_every_(config_.physics_rate_hz / config_.state_rate_hz),
        log_every_(config_.physics_rate_hz / config_.log_rate_hz) {}

  ~Impl() {
    for (const Client& client : clients_) {
      close(client.fd);
    }
    close(listen_fd_);
  }

  void run(const std::stop_token& stop) {
    while (!stop.stop_requested()) {
      drain_snapshots();
      drain_results();
      poll_clients();
    }
    drain_snapshots();
    truth_.flush();
  }

  IoStats stats() const { return stats_; }

 private:
  void drain_snapshots() {
    sim::Snapshot s{};
    while (snapshots_.try_pop(s)) {
      ++tick_;
      latest_ = s;
      if (tick_ % render_every_ == 0) {
        send_render_state(s);
      }
      if (!s.paused && s.step_index % log_every_ == 0) {
        truth_.write(s);
        ++stats_.log_rows;
      }
      for (Client& client : clients_) {
        for (const Subscription& sub : client.subscriptions) {
          if (tick_ % sub.every_ticks == 0) {
            const nlohmann::json data =
                sub.topic == "input_raw"
                    ? input_raw_json(s)
                    : telemetry_json(s, config_.physics_rate_hz, config_.input_source);
            write_to(client, api::event_line(sub.topic, s.sim_time_ns, data));
          }
        }
      }
      close_failed_clients();
    }
  }

  void drain_results() {
    sim::CommandResult result{};
    while (results_.try_pop(result)) {
      for (Client& client : clients_) {
        if (client.id == result.client) {
          write_to(client,
                   api::ok_response(result.request_id, {{"applied_at_ns", result.applied_at_ns}}));
        }
      }
    }
    close_failed_clients();
  }

  void send_render_state(const sim::Snapshot& s) {
    std::array<std::byte, proto::kRenderStateMessageSize> buffer{};
    const std::size_t size =
        proto::serialize_render_state(render_state_from(s), render_seq_++, buffer);
    if (render_socket_.send_to(std::span(buffer).first(size), render_address_)) {
      ++stats_.render_states_sent;
    }
  }

  void poll_clients() {
    std::vector<pollfd> fds;
    fds.push_back({.fd = listen_fd_, .events = POLLIN, .revents = 0});
    for (const Client& client : clients_) {
      fds.push_back({.fd = client.fd, .events = POLLIN, .revents = 0});
    }
    if (poll(fds.data(), fds.size(), kPollTimeoutMs) <= 0) {
      return;
    }
    if ((fds[0].revents & POLLIN) != 0) {
      accept_client();
    }
    for (std::size_t i = 1; i < fds.size(); ++i) {
      if (fds[i].revents != 0) {
        read_client(clients_[i - 1]);
      }
    }
    close_failed_clients();
  }

  void accept_client() {
    const int fd = accept4(listen_fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (fd < 0) {
      return;
    }
    const int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    clients_.push_back(Client{.fd = fd, .id = next_client_id_++, .inbuf = {}, .subscriptions = {}});
    ++stats_.clients_seen;
  }

  void read_client(Client& client) {
    std::array<char, 4096> buffer{};
    const ssize_t received = recv(client.fd, buffer.data(), buffer.size(), 0);
    if (received <= 0) {
      if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return;
      }
      failed_.push_back(client.id);
      return;
    }
    client.inbuf.append(buffer.data(), static_cast<std::size_t>(received));
    if (client.inbuf.size() > kMaxLineBytes) {
      failed_.push_back(client.id);
      return;
    }
    std::size_t newline = 0;
    while ((newline = client.inbuf.find('\n')) != std::string::npos) {
      const std::string line = client.inbuf.substr(0, newline);
      client.inbuf.erase(0, newline + 1);
      handle_line(client, line);
    }
  }

  void handle_line(Client& client, const std::string& line) {
    if (line.empty()) {
      return;
    }
    ++stats_.api_requests;
    const auto parsed = api::parse_request(line);
    if (const auto* error = std::get_if<api::RequestError>(&parsed)) {
      write_to(client, api::error_response(error->id, error->code, error->message));
      return;
    }
    const auto& request = std::get<api::Request>(parsed);
    switch (request.method) {
      case api::Method::kPing:
        write_to(client, api::ok_response(request.id, {{"pong", true}}));
        break;
      case api::Method::kGetInfo:
        write_to(client, api::ok_response(request.id, config_.info));
        break;
      case api::Method::kGetState:
        if (latest_) {
          write_to(client, api::ok_response(request.id, state_json(*latest_)));
        } else {
          write_to(client, api::error_response(request.id, "invalid_state", "no state yet"));
        }
        break;
      case api::Method::kReset:
        queue_command(client, request.id, sim::CommandType::kReset);
        break;
      case api::Method::kPause:
        queue_command(client, request.id, sim::CommandType::kPause);
        break;
      case api::Method::kResume:
        queue_command(client, request.id, sim::CommandType::kResume);
        break;
      case api::Method::kShutdown:
        queue_command(client, request.id, sim::CommandType::kShutdown);
        break;
      case api::Method::kSubscribe:
        subscribe(client, request);
        break;
      case api::Method::kUnsubscribe:
        unsubscribe(client, request);
        break;
      case api::Method::kListInputDevices:
        write_to(client,
                 api::ok_response(request.id, {{"devices", input_control_.status().device_names}}));
        break;
      case api::Method::kGetInput:
        write_to(client, api::ok_response(request.id, input_json()));
        break;
      case api::Method::kSetInputMapping:
        set_input_mapping(client, request);
        break;
      case api::Method::kSelectInputDevice:
        select_input_device(client, request);
        break;
      case api::Method::kReloadModel:
        reload_model(client, request);
        break;
    }
  }

  void queue_command(Client& client, std::int64_t request_id, sim::CommandType type) {
    const sim::Command command{.client = client.id, .request_id = request_id, .type = type};
    if (!commands_.try_push(command)) {
      write_to(client, api::error_response(request_id, "internal", "command queue full"));
    }
  }

  nlohmann::json input_json() const {
    const input::InputStatus status = input_control_.status();
    return {{"source", config_.input_source},
            {"device",
             {{"name", status.device.name},
              {"connected", status.device.connected},
              {"axis_count", status.device.axis_count},
              {"button_count", status.device.button_count}}},
            {"devices", status.device_names},
            {"mapping", input::mapping_to_json(status.mapping)}};
  }

  static nlohmann::json input_raw_json(const sim::Snapshot& s) {
    const std::vector<float> axes(s.raw_axes.begin(), s.raw_axes.begin() + s.raw_axis_count);
    const std::vector<int> buttons(s.raw_buttons.begin(),
                                   s.raw_buttons.begin() + s.raw_button_count);
    return {{"connected", s.device_connected},
            {"axes", axes},
            {"buttons", buttons},
            {"channels_us", s.rc_channels_us}};
  }

  void set_input_mapping(Client& client, const api::Request& request) {
    if (!request.params.contains("mapping")) {
      write_to(client, api::error_response(request.id, "invalid_params", "missing mapping"));
      return;
    }
    try {
      input_control_.request_mapping(input::mapping_from_json(request.params["mapping"]));
    } catch (const std::exception& error) {
      write_to(client, api::error_response(request.id, "invalid_params", error.what()));
      return;
    }
    queue_command(client, request.id, sim::CommandType::kSetInputMapping);
  }

  void reload_model(Client& client, const api::Request& request) {
    const auto& params = request.params;
    if (!params.contains("path") || !params["path"].is_string()) {
      write_to(client, api::error_response(request.id, "invalid_params", "missing path"));
      return;
    }
    try {
      model_control_.request(config::load_drone(params["path"].get<std::string>()));
    } catch (const std::exception& error) {
      write_to(client, api::error_response(request.id, "invalid_params", error.what()));
      return;
    }
    queue_command(client, request.id, sim::CommandType::kReloadModel);
  }

  void select_input_device(Client& client, const api::Request& request) {
    const auto& params = request.params;
    if (!params.contains("name_contains") || !params["name_contains"].is_string()) {
      write_to(client, api::error_response(request.id, "invalid_params", "missing name_contains"));
      return;
    }
    input_control_.request_device(params["name_contains"].get<std::string>());
    queue_command(client, request.id, sim::CommandType::kSelectInputDevice);
  }

  void subscribe(Client& client, const api::Request& request) {
    const auto& params = request.params;
    const std::string topic = params.value("topic", "");
    if (topic != "telemetry" && topic != "input_raw") {
      write_to(client, api::error_response(request.id, "invalid_params",
                                           "topic must be 'telemetry' or 'input_raw'"));
      return;
    }
    const double rate_hz = params.value("rate_hz", 10.0);
    if (rate_hz <= 0.0 || rate_hz > static_cast<double>(config_.physics_rate_hz)) {
      write_to(client, api::error_response(request.id, "invalid_params", "rate_hz out of range"));
      return;
    }
    const auto every =
        static_cast<std::int64_t>(static_cast<double>(config_.physics_rate_hz) / rate_hz);
    const std::int64_t id = next_subscription_id_++;
    client.subscriptions.push_back(
        {.id = id, .every_ticks = std::max<std::int64_t>(1, every), .topic = topic});
    write_to(client, api::ok_response(request.id, {{"subscription_id", id}}));
  }

  void unsubscribe(Client& client, const api::Request& request) {
    const auto& params = request.params;
    if (!params.contains("subscription_id") || !params["subscription_id"].is_number_integer()) {
      write_to(client,
               api::error_response(request.id, "invalid_params", "missing subscription_id"));
      return;
    }
    const auto id = params["subscription_id"].get<std::int64_t>();
    auto& subs = client.subscriptions;
    const auto it = std::ranges::find_if(subs, [id](const Subscription& s) { return s.id == id; });
    if (it == subs.end()) {
      write_to(client, api::error_response(request.id, "invalid_params", "unknown subscription"));
      return;
    }
    subs.erase(it);
    write_to(client, api::ok_response(request.id, {{"ok", true}}));
  }

  void write_to(Client& client, const std::string& text) {
    std::size_t sent = 0;
    while (sent < text.size()) {
      const ssize_t n = send(client.fd, text.data() + sent, text.size() - sent, MSG_NOSIGNAL);
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd waiter{.fd = client.fd, .events = POLLOUT, .revents = 0};
        poll(&waiter, 1, 100);
        continue;
      }
      if (n <= 0) {
        failed_.push_back(client.id);
        return;
      }
      sent += static_cast<std::size_t>(n);
    }
  }

  void close_failed_clients() {
    for (const std::uint32_t id : failed_) {
      const auto it = std::ranges::find_if(clients_, [id](const Client& c) { return c.id == id; });
      if (it != clients_.end()) {
        close(it->fd);
        clients_.erase(it);
      }
    }
    failed_.clear();
  }

  input::InputControl& input_control_;
  sim::ModelControl& model_control_;
  IoConfig config_;
  SnapshotQueue& snapshots_;
  CommandQueue& commands_;
  ResultQueue& results_;
  net::UdpSocket render_socket_;
  sockaddr_in render_address_;
  log::TruthCsv truth_;
  int listen_fd_;
  std::int64_t render_every_;
  std::int64_t log_every_;
  std::vector<Client> clients_;
  std::vector<std::uint32_t> failed_;
  std::optional<sim::Snapshot> latest_;
  std::uint32_t next_client_id_ = 1;
  std::int64_t next_subscription_id_ = 1;
  std::uint32_t render_seq_ = 0;
  std::int64_t tick_ = 0;
  IoStats stats_;
};

IoThread::IoThread(input::InputControl& input_control, sim::ModelControl& model_control,
                   IoConfig config, SnapshotQueue& snapshots, CommandQueue& commands,
                   ResultQueue& results)
    : impl_(std::make_unique<Impl>(input_control, model_control, std::move(config), snapshots,
                                   commands, results)),
      thread_([this](const std::stop_token& stop) { impl_->run(stop); }) {}

IoThread::~IoThread() {
  if (thread_.joinable()) {
    thread_.request_stop();
    thread_.join();
  }
}

IoStats IoThread::stop() {
  thread_.request_stop();
  thread_.join();
  return impl_->stats();
}

}  // namespace fpvsim::io
