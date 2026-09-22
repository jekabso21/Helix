#pragma once

#include <nlohmann/json.hpp>

#include <fpvsim/input/mapping.hpp>

namespace fpvsim::input {

// {"device_name_contains", "arm_channel", "channels": {name: {"axis"|"button", "inverted",
// "deadband"}}} Throws std::runtime_error naming the field on invalid input
InputMapping mapping_from_json(const nlohmann::json& node);
nlohmann::json mapping_to_json(const InputMapping& mapping);

}  // namespace fpvsim::input
