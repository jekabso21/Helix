#include <fpvsim/api/protocol.hpp>

#include <map>

namespace fpvsim::api {

namespace {

const std::map<std::string, Method>& method_names() {
  static const std::map<std::string, Method> names = {
      {"ping", Method::kPing},
      {"get_info", Method::kGetInfo},
      {"get_state", Method::kGetState},
      {"reset", Method::kReset},
      {"pause", Method::kPause},
      {"resume", Method::kResume},
      {"shutdown", Method::kShutdown},
      {"subscribe", Method::kSubscribe},
      {"unsubscribe", Method::kUnsubscribe},
      {"list_input_devices", Method::kListInputDevices},
      {"get_input", Method::kGetInput},
      {"set_input_mapping", Method::kSetInputMapping},
      {"select_input_device", Method::kSelectInputDevice},
      {"reload_model", Method::kReloadModel},
  };
  return names;
}

}  // namespace

std::variant<Request, RequestError> parse_request(const std::string& line) {
  const nlohmann::json doc = nlohmann::json::parse(line, nullptr, false);
  if (doc.is_discarded() || !doc.is_object()) {
    return RequestError{.id = std::nullopt, .code = "invalid_json", .message = "not a JSON object"};
  }
  std::optional<std::int64_t> id;
  if (doc.contains("id") && doc["id"].is_number_integer()) {
    id = doc["id"].get<std::int64_t>();
  }
  if (!id) {
    return RequestError{
        .id = std::nullopt, .code = "invalid_params", .message = "missing integer id"};
  }
  if (!doc.contains("method") || !doc["method"].is_string()) {
    return RequestError{.id = id, .code = "invalid_params", .message = "missing method"};
  }
  const auto it = method_names().find(doc["method"].get<std::string>());
  if (it == method_names().end()) {
    return RequestError{
        .id = id, .code = "unknown_method", .message = doc["method"].get<std::string>()};
  }
  nlohmann::json params = nlohmann::json::object();
  if (doc.contains("params")) {
    if (!doc["params"].is_object()) {
      return RequestError{
          .id = id, .code = "invalid_params", .message = "params must be an object"};
    }
    params = doc["params"];
  }
  return Request{.id = *id, .method = it->second, .params = params};
}

std::string ok_response(std::int64_t id, const nlohmann::json& result) {
  return nlohmann::json{{"id", id}, {"ok", true}, {"result", result}}.dump() + "\n";
}

std::string error_response(std::optional<std::int64_t> id, const std::string& code,
                           const std::string& message) {
  nlohmann::json doc = {{"ok", false}, {"error", {{"code", code}, {"message", message}}}};
  doc["id"] = id ? nlohmann::json(*id) : nlohmann::json(nullptr);
  return doc.dump() + "\n";
}

std::string event_line(const std::string& event, std::int64_t sim_time_ns,
                       const nlohmann::json& data) {
  return nlohmann::json{{"event", event}, {"t_ns", sim_time_ns}, {"data", data}}.dump() + "\n";
}

}  // namespace fpvsim::api
