#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include <nlohmann/json.hpp>

namespace fpvsim::api {

enum class Method : std::uint8_t {
  kPing,
  kGetInfo,
  kGetState,
  kReset,
  kPause,
  kResume,
  kShutdown,
  kSubscribe,
  kUnsubscribe,
};

struct Request {
  std::int64_t id;
  Method method;
  nlohmann::json params;
};

struct RequestError {
  std::optional<std::int64_t> id;
  std::string code;  // invalid_json, unknown_method, invalid_params
  std::string message;
};

std::variant<Request, RequestError> parse_request(const std::string& line);

std::string ok_response(std::int64_t id, const nlohmann::json& result);
std::string error_response(std::optional<std::int64_t> id, const std::string& code,
                           const std::string& message);
std::string event_line(const std::string& event, std::int64_t sim_time_ns,
                       const nlohmann::json& data);

}  // namespace fpvsim::api
