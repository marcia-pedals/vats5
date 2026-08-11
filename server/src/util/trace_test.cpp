#include "util/trace.h"

#include <gtest/gtest.h>

namespace vats5 {
namespace {

TEST(TraceTest, RootOnly) {
  Trace trace("root");
  nlohmann::json j = trace.ToJson();
  EXPECT_EQ(j["name"], "root");
  EXPECT_EQ(j["start_seconds"], 0.0);
  EXPECT_FALSE(j.contains("children"));
  EXPECT_FALSE(j.contains("metadata"));
}

TEST(TraceTest, NestedSpansWithMetadata) {
  Trace trace("root");
  {
    TraceSpan outer(trace, "outer");
    outer.SetMetadata("size", 3);
    {
      TraceSpan inner(trace, "inner");
    }
    {
      TraceSpan sibling(trace, "sibling");
    }
  }
  {
    TraceSpan after(trace, "after");
  }

  nlohmann::json j = trace.ToJson();
  ASSERT_EQ(j["children"].size(), 2);

  const nlohmann::json& outer = j["children"][0];
  EXPECT_EQ(outer["name"], "outer");
  EXPECT_EQ(outer["metadata"]["size"], 3);
  ASSERT_EQ(outer["children"].size(), 2);
  EXPECT_EQ(outer["children"][0]["name"], "inner");
  EXPECT_EQ(outer["children"][1]["name"], "sibling");

  EXPECT_EQ(j["children"][1]["name"], "after");
  EXPECT_FALSE(j["children"][1].contains("children"));
}

TEST(TraceTest, StartsAreOrderedAndWithinTheirParent) {
  Trace trace("root");
  {
    TraceSpan outer(trace, "outer");
    {
      TraceSpan inner(trace, "inner");
    }
  }

  nlohmann::json j = trace.ToJson();
  const nlohmann::json& outer = j["children"][0];
  const nlohmann::json& inner = outer["children"][0];

  // Starts are all relative to the root, so a child starts at or after its
  // parent and ends at or before it.
  EXPECT_GE(outer["start_seconds"].get<double>(), 0.0);
  EXPECT_GE(inner["start_seconds"].get<double>(), outer["start_seconds"]);
  EXPECT_LE(
      inner["start_seconds"].get<double>() +
          inner["duration_seconds"].get<double>(),
      outer["start_seconds"].get<double>() +
          outer["duration_seconds"].get<double>()
  );
  EXPECT_LE(
      outer["start_seconds"].get<double>() +
          outer["duration_seconds"].get<double>(),
      j["duration_seconds"].get<double>()
  );
}

// A solve that gives up part way through still reports what it did get through.
TEST(TraceTest, SpanIsRecordedWhenLeftByException) {
  Trace trace("root");
  try {
    TraceSpan span(trace, "interrupted");
    throw std::runtime_error("timeout");
  } catch (const std::runtime_error&) {
  }

  nlohmann::json j = trace.ToJson();
  ASSERT_EQ(j["children"].size(), 1);
  EXPECT_EQ(j["children"][0]["name"], "interrupted");
}

}  // namespace
}  // namespace vats5
