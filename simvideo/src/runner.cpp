#include <fpvsim/video/runner.hpp>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <mutex>
#include <utility>

namespace fpvsim::video {

namespace {

constexpr auto kFpsWindow = std::chrono::milliseconds(1000);
constexpr auto kMaxBackoff = std::chrono::milliseconds(8000);

GstElement* as_element(void* pointer) { return static_cast<GstElement*>(pointer); }

}  // namespace

void init_gstreamer() {
  static std::once_flag once;
  std::call_once(once, [] { gst_init(nullptr, nullptr); });
}

OutputBranch::OutputBranch(const CameraSpec& camera, OutputSpec output, int index)
    : output_(std::move(output)),
      index_(index),
      description_(output_.pipeline),
      caps_(caps_string(camera)) {
  if (!output_.enabled) {
    state_ = "disabled";
  }
}

OutputBranch::OutputBranch(OutputBranch&& other) noexcept
    : output_(std::move(other.output_)),
      index_(other.index_),
      description_(std::move(other.description_)),
      caps_(std::move(other.caps_)),
      pipeline_(std::exchange(other.pipeline_, nullptr)),
      appsrc_(std::exchange(other.appsrc_, nullptr)),
      state_(std::move(other.state_)),
      last_error_(std::move(other.last_error_)),
      pushed_(other.pushed_),
      backoff_(other.backoff_) {}

OutputBranch::~OutputBranch() { stop(); }

void OutputBranch::start() {
  if (!output_.enabled || pipeline_ != nullptr) {
    return;
  }
  const std::string launch =
      "appsrc name=src is-live=true format=time do-timestamp=false caps=" + caps_ +
      " ! queue leaky=downstream max-size-buffers=2 ! videoconvert" + " ! " + output_.pipeline;
  GError* error = nullptr;
  GstElement* pipeline = gst_parse_launch(launch.c_str(), &error);
  if (pipeline == nullptr || error != nullptr) {
    const std::string message = error != nullptr ? error->message : "could not build the pipeline";
    if (error != nullptr) {
      g_error_free(error);
    }
    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
    }
    fail(message, std::chrono::steady_clock::now());
    return;
  }
  GstElement* appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
  if (appsrc == nullptr) {
    gst_object_unref(pipeline);
    fail("the pipeline has no appsrc named src", std::chrono::steady_clock::now());
    return;
  }
  if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    gst_object_unref(appsrc);
    gst_object_unref(pipeline);
    fail("the pipeline refused to start", std::chrono::steady_clock::now());
    return;
  }
  pipeline_ = pipeline;
  appsrc_ = appsrc;
  state_ = "running";
  last_error_.clear();
  window_start_ = std::chrono::steady_clock::now();
  pushed_at_window_ = pushed_;
}

void OutputBranch::stop() {
  if (appsrc_ != nullptr) {
    gst_object_unref(as_element(appsrc_));
    appsrc_ = nullptr;
  }
  if (pipeline_ != nullptr) {
    gst_element_set_state(as_element(pipeline_), GST_STATE_NULL);
    gst_object_unref(as_element(pipeline_));
    pipeline_ = nullptr;
  }
  if (state_ == "running") {
    state_ = "disabled";
  }
}

void OutputBranch::fail(const std::string& message, std::chrono::steady_clock::time_point now) {
  last_error_ = message;
  state_ = "error";
  retry_at_ = now + backoff_;
  backoff_ = std::min(backoff_ * 2, kMaxBackoff);
}

void OutputBranch::push(void* buffer) {
  if (appsrc_ == nullptr || state_ != "running") {
    return;
  }
  auto* gst_buffer = static_cast<GstBuffer*>(buffer);
  const GstFlowReturn flow =
      gst_app_src_push_buffer(GST_APP_SRC(as_element(appsrc_)), gst_buffer_ref(gst_buffer));
  if (flow != GST_FLOW_OK) {
    fail("appsrc rejected a buffer", std::chrono::steady_clock::now());
    return;
  }
  ++pushed_;
}

void OutputBranch::poll(std::chrono::steady_clock::time_point now) {
  if (pipeline_ != nullptr) {
    GstBus* bus = gst_element_get_bus(as_element(pipeline_));
    while (GstMessage* message = gst_bus_pop_filtered(
               bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS))) {
      if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
        GError* error = nullptr;
        gst_message_parse_error(message, &error, nullptr);
        const std::string text = error != nullptr ? error->message : "pipeline error";
        if (error != nullptr) {
          g_error_free(error);
        }
        gst_message_unref(message);
        gst_object_unref(bus);
        stop();
        fail(text, now);
        return;
      }
      gst_message_unref(message);
      gst_object_unref(bus);
      stop();
      fail("the pipeline ended", now);
      return;
    }
    gst_object_unref(bus);
    return;
  }
  if (output_.enabled && state_ == "error" && now >= retry_at_) {
    state_ = "restarting";
    start();
  }
}

OutputStatus OutputBranch::status(std::chrono::steady_clock::time_point now) {
  if (now - window_start_ >= kFpsWindow) {
    const auto seconds = std::chrono::duration<double>(now - window_start_).count();
    fps_ = seconds > 0.0 ? static_cast<double>(pushed_ - pushed_at_window_) / seconds : 0.0;
    window_start_ = now;
    pushed_at_window_ = pushed_;
  }
  return OutputStatus{
      .index = index_,
      .pipeline = description_,
      .state = state_,
      .fps = state_ == "running" ? fps_ : 0.0,
      .bitrate_bps = std::nullopt,
      .last_error = last_error_.empty() ? std::nullopt : std::optional<std::string>(last_error_)};
}

CameraRunner::CameraRunner(CameraSpec camera) : camera_(std::move(camera)) {
  init_gstreamer();
  int index = 0;
  for (const OutputSpec& output : camera_.outputs) {
    branches_.push_back(std::make_unique<OutputBranch>(camera_, output, index));
    ++index;
  }
  window_start_ = std::chrono::steady_clock::now();
}

CameraRunner::~CameraRunner() { stop(); }

void CameraRunner::start() {
  for (auto& branch : branches_) {
    branch->start();
  }
  window_start_ = std::chrono::steady_clock::now();
  frames_at_window_ = frames_;
}

void CameraRunner::stop() {
  for (auto& branch : branches_) {
    branch->stop();
  }
}

void CameraRunner::push_frame(std::span<const std::byte> pixels, std::int64_t pts_ns) {
  if (pixels.empty() || branches_.empty()) {
    return;
  }
  GstBuffer* buffer = gst_buffer_new_allocate(nullptr, pixels.size(), nullptr);
  if (buffer == nullptr) {
    return;
  }
  gst_buffer_fill(buffer, 0, pixels.data(), pixels.size());
  GST_BUFFER_PTS(buffer) = static_cast<GstClockTime>(pts_ns);
  GST_BUFFER_DURATION(buffer) =
      camera_.fps > 0.0 ? static_cast<GstClockTime>(GST_SECOND / camera_.fps) : GST_CLOCK_TIME_NONE;
  for (auto& branch : branches_) {
    branch->push(buffer);
  }
  gst_buffer_unref(buffer);
  ++frames_;
}

void CameraRunner::poll() {
  const auto now = std::chrono::steady_clock::now();
  for (auto& branch : branches_) {
    branch->poll(now);
  }
}

CameraStatus CameraRunner::status() {
  const auto now = std::chrono::steady_clock::now();
  if (now - window_start_ >= kFpsWindow) {
    const auto seconds = std::chrono::duration<double>(now - window_start_).count();
    input_fps_ = seconds > 0.0 ? static_cast<double>(frames_ - frames_at_window_) / seconds : 0.0;
    window_start_ = now;
    frames_at_window_ = frames_;
  }
  CameraStatus status{
      .camera = camera_.name, .input_fps = input_fps_, .late_frames = late_frames_, .outputs = {}};
  for (auto& branch : branches_) {
    status.outputs.push_back(branch->status(now));
  }
  return status;
}

}  // namespace fpvsim::video
