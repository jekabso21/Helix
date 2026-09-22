#include <gtest/gtest.h>

#include <string>
#include <variant>

#include <fpvsim/api/protocol.hpp>

namespace api = fpvsim::api;

TEST(ApiProtocolTest, ParsesARequestWithParams) {
  const auto parsed = api::parse_request(
      R"({"id": 3, "method": "subscribe", "params": {"topic": "telemetry", "rate_hz": 30}})");
  ASSERT_TRUE(std::holds_alternative<api::Request>(parsed));
  const auto& request = std::get<api::Request>(parsed);
  EXPECT_EQ(request.id, 3);
  EXPECT_EQ(request.method, api::Method::kSubscribe);
  EXPECT_EQ(request.params.at("rate_hz").get<int>(), 30);
}

TEST(ApiProtocolTest, KnowsReloadModel) {
  const auto parsed =
      api::parse_request(R"({"id": 7, "method": "reload_model", "params": {"path": "x"}})");
  ASSERT_TRUE(std::holds_alternative<api::Request>(parsed));
  EXPECT_EQ(std::get<api::Request>(parsed).method, api::Method::kReloadModel);
}

TEST(ApiProtocolTest, MissingParamsDefaultsToEmptyObject) {
  const auto parsed = api::parse_request(R"({"id": 1, "method": "ping"})");
  ASSERT_TRUE(std::holds_alternative<api::Request>(parsed));
  EXPECT_TRUE(std::get<api::Request>(parsed).params.is_object());
}

TEST(ApiProtocolTest, ReportsErrorsWithCodes) {
  const auto bad_json = api::parse_request("{not json");
  ASSERT_TRUE(std::holds_alternative<api::RequestError>(bad_json));
  EXPECT_EQ(std::get<api::RequestError>(bad_json).code, "invalid_json");

  const auto unknown = api::parse_request(R"({"id": 2, "method": "fly"})");
  ASSERT_TRUE(std::holds_alternative<api::RequestError>(unknown));
  EXPECT_EQ(std::get<api::RequestError>(unknown).code, "unknown_method");
  EXPECT_EQ(std::get<api::RequestError>(unknown).id, 2);

  const auto no_id = api::parse_request(R"({"method": "ping"})");
  ASSERT_TRUE(std::holds_alternative<api::RequestError>(no_id));
  EXPECT_FALSE(std::get<api::RequestError>(no_id).id.has_value());
}

TEST(ApiProtocolTest, ResponsesAndEventsAreSingleJsonLines) {
  const std::string ok = api::ok_response(5, {{"pong", true}});
  EXPECT_EQ(ok, R"({"id":5,"ok":true,"result":{"pong":true}})"
                "\n");
  const std::string err = api::error_response(std::nullopt, "invalid_json", "x");
  EXPECT_EQ(err, R"({"error":{"code":"invalid_json","message":"x"},"id":null,"ok":false})"
                 "\n");
  const std::string event = api::event_line("telemetry", 42, {{"a", 1}});
  EXPECT_EQ(event, R"({"data":{"a":1},"event":"telemetry","t_ns":42})"
                   "\n");
}
