#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <map>
#include <mutex>

#include <nlohmann/json.hpp>

#include <fpvsim/proto/frame_ring.hpp>
#include <fpvsim/video/config.hpp>
#include <fpvsim/video/control.hpp>
#include <fpvsim/video/runner.hpp>
#include <fpvsim/video/schedule.hpp>
#include <fpvsim/video/status.hpp>

namespace {

std::atomic<bool> g_stop{false};

void request_stop(int /*signal*/) { g_stop.store(true, std::memory_order_relaxed); }

std::int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Minimal UDP sender for the status datagrams
class StatusSocket {
 public:
  StatusSocket(const std::string& host, std::uint16_t port)
      : fd_(::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0)) {
    address_.sin_family = AF_INET;
    address_.sin_port = htons(port);
    ::inet_pton(AF_INET, host.c_str(), &address_.sin_addr);
  }
  ~StatusSocket() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }
  StatusSocket(const StatusSocket&) = delete;
  StatusSocket& operator=(const StatusSocket&) = delete;

  void send(const std::string& payload) const {
    if (fd_ >= 0) {
      ::sendto(fd_, payload.data(), payload.size(), MSG_DONTWAIT,
               reinterpret_cast<const sockaddr*>(&address_), sizeof(address_));
    }
  }

 private:
  int fd_;
  sockaddr_in address_{};
};

// Commands from the control port, picked up by the camera thread that owns the pipelines
class ControlMailbox {
 public:
  void post(std::size_t index, bool enabled) {
    const std::lock_guard<std::mutex> guard(mutex_);
    pending_.emplace_back(index, enabled);
  }
  std::vector<std::pair<std::size_t, bool>> take() {
    const std::lock_guard<std::mutex> guard(mutex_);
    return std::exchange(pending_, {});
  }

 private:
  std::mutex mutex_;
  std::vector<std::pair<std::size_t, bool>> pending_;
};

using Mailboxes = std::map<std::string, ControlMailbox>;

void handle_control_datagram(const char* data, std::size_t size, Mailboxes& mailboxes) {
  const auto command = fpvsim::video::parse_control(std::string_view(data, size));
  if (!command) {
    return;
  }
  const auto found = mailboxes.find(command->camera);
  if (found != mailboxes.end()) {
    found->second.post(command->index, command->enabled);
  }
}

// Listens for output commands until the process stops
void run_control(const std::string& host, std::uint16_t port, Mailboxes& mailboxes) {
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    return;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  ::inet_pton(AF_INET, host.c_str(), &address.sin_addr);
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
    std::cerr << "simvideo: control port " << port << " is taken; outputs cannot be switched\n";
    ::close(fd);
    return;
  }
  timeval timeout{.tv_sec = 0, .tv_usec = 200000};
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  std::array<char, 4096> buffer{};
  while (!g_stop.load(std::memory_order_relaxed)) {
    const ssize_t received = ::recv(fd, buffer.data(), buffer.size(), 0);
    if (received > 0) {
      handle_control_datagram(buffer.data(), static_cast<std::size_t>(received), mailboxes);
    }
  }
  ::close(fd);
}

// Feeds one camera: ring -> sensor latency hold -> the camera's output pipelines
void run_camera(const fpvsim::video::CameraSpec& spec, const StatusSocket& status,
                ControlMailbox& mailbox) {
  using namespace std::chrono_literals;
  fpvsim::video::CameraRunner runner(spec);
  runner.start();

  std::optional<fpvsim::proto::FrameRingReader> reader;
  std::vector<std::byte> pixels;
  std::vector<std::byte> pending_pixels;
  std::uint64_t last_seq = 0;
  fpvsim::video::Anchor anchor{};
  bool anchored = false;
  bool reanchor = false;
  std::int64_t last_pts = 0;
  bool have_pending = false;
  std::int64_t pending_pts = 0;
  std::int64_t pending_release = 0;
  auto next_status = std::chrono::steady_clock::now();
  auto next_open_attempt = std::chrono::steady_clock::now();
  auto next_replaced_check = std::chrono::steady_clock::now();

  while (!g_stop.load(std::memory_order_relaxed)) {
    const auto now = std::chrono::steady_clock::now();
    bool did_work = false;

    // A publisher that closed cleanly unlinked its ring and the next one creates a new object,
    // so the mapped one would never change again
    if (reader && now >= next_replaced_check) {
      next_replaced_check = now + 500ms;
      if (reader->replaced()) {
        reader.reset();
        last_seq = 0;
        reanchor = true;
        next_open_attempt = now;
      }
    }

    if (!reader) {
      if (now >= next_open_attempt) {
        next_open_attempt = now + 500ms;
        try {
          reader.emplace(spec.name);  // the publisher may not have created the ring yet
          pixels.assign(reader->frame_bytes(), std::byte{});
          pending_pixels.assign(reader->frame_bytes(), std::byte{});
        } catch (const std::exception&) {
          reader.reset();
        }
      }
    } else if (!have_pending) {
      // Hold at most one frame: never sleep while holding, or the ring would run ahead and the
      // next read would skip the frame in between.
      fpvsim::proto::FrameMeta meta{};
      const std::uint64_t seq = reader->read_latest(last_seq, meta, pixels);
      if (seq != 0) {
        // a lower sequence is a publisher that restarted in the same ring
        if (!anchored || reanchor || seq < last_seq) {
          const std::int64_t next_pts =
              anchored ? last_pts + fpvsim::video::frame_period_ns(spec.fps) : 0;
          anchor = fpvsim::video::anchor_at(meta.sim_time_ns, steady_ns(), next_pts);
          anchored = true;
          reanchor = false;
        }
        last_seq = seq;
        pending_pts = fpvsim::video::buffer_pts_ns(meta.sim_time_ns, anchor.first_sim_ns);
        last_pts = pending_pts;
        pending_release = fpvsim::video::release_wall_ns(
            meta.sim_time_ns, anchor.first_sim_ns, anchor.first_wall_ns, spec.sensor_latency_s);
        pending_pixels.swap(pixels);
        have_pending = true;
        did_work = true;
      }
    }

    if (have_pending && steady_ns() >= pending_release) {
      if (fpvsim::video::is_late(pending_release, steady_ns(), spec.fps)) {
        runner.note_late_frame();
      }
      runner.push_frame(pending_pixels, pending_pts);
      have_pending = false;
      did_work = true;
    }

    for (const auto& [index, enabled] : mailbox.take()) {
      runner.set_output_enabled(index, enabled);
      did_work = true;
    }
    runner.poll();
    if (now >= next_status) {
      next_status = now + 500ms;
      status.send(fpvsim::video::status_json(runner.status()));
    }
    if (!did_work) {
      std::this_thread::sleep_for(500us);
    }
  }
  runner.stop();
}

}  // namespace

int main(int argc, char** argv) {
  std::string cameras_path;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--cameras" && i + 1 < argc) {
      cameras_path = argv[++i];
    }
  }
  if (cameras_path.empty()) {
    std::cerr << "usage: simvideo --cameras <resolved/cameras.json>\n";
    return 2;
  }

  std::signal(SIGTERM, request_stop);
  std::signal(SIGINT, request_stop);

  try {
    const fpvsim::video::VideoConfig config = fpvsim::video::load_cameras(cameras_path);
    fpvsim::video::init_gstreamer();
    const StatusSocket status(config.host, config.status_port);
    std::cout << "simvideo: " << config.cameras.size() << " camera(s), status to " << config.host
              << ":" << config.status_port << std::endl;

    Mailboxes mailboxes;
    for (const fpvsim::video::CameraSpec& camera : config.cameras) {
      mailboxes[camera.name];
    }
    std::thread control([&config, &mailboxes] {
      run_control(config.host, config.control_port, mailboxes);
    });

    std::vector<std::thread> workers;
    workers.reserve(config.cameras.size());
    for (const fpvsim::video::CameraSpec& camera : config.cameras) {
      workers.emplace_back([&camera, &status, &mailboxes] {
        run_camera(camera, status, mailboxes.at(camera.name));
      });
    }
    for (std::thread& worker : workers) {
      worker.join();
    }
    control.join();
  } catch (const std::exception& error) {
    std::cerr << "simvideo: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
