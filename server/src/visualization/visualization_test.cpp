#include "visualization/visualization.h"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace vats5::viz {

TEST(VisualizationTest, StopSerializesToJson) {
  Stop stop{
      .stop_id = "12345",
      .stop_name = "Main Street Station",
      .lat = 37.7749,
      .lon = -122.4194,
  };

  nlohmann::json j = stop;

  EXPECT_EQ(j["stop_id"], "12345");
  EXPECT_EQ(j["stop_name"], "Main Street Station");
  EXPECT_DOUBLE_EQ(j["lat"], 37.7749);
  EXPECT_DOUBLE_EQ(j["lon"], -122.4194);
}

TEST(VisualizationTest, StopDeserializesFromJson) {
  nlohmann::json j = {
      {"stop_id", "67890"},
      {"stop_name", "Airport Terminal"},
      {"lat", 40.7128},
      {"lon", -74.0060},
  };

  Stop stop = j.get<Stop>();

  EXPECT_EQ(stop.stop_id, "67890");
  EXPECT_EQ(stop.stop_name, "Airport Terminal");
  EXPECT_DOUBLE_EQ(stop.lat, 40.7128);
  EXPECT_DOUBLE_EQ(stop.lon, -74.0060);
}

TEST(VisualizationTest, StopRoundTrip) {
  Stop original{
      .stop_id = "abc123",
      .stop_name = "University Avenue",
      .lat = 51.5074,
      .lon = -0.1278,
  };

  nlohmann::json j = original;
  Stop deserialized = j.get<Stop>();

  EXPECT_EQ(deserialized.stop_id, original.stop_id);
  EXPECT_EQ(deserialized.stop_name, original.stop_name);
  EXPECT_DOUBLE_EQ(deserialized.lat, original.lat);
  EXPECT_DOUBLE_EQ(deserialized.lon, original.lon);
}

}  // namespace vats5::viz
