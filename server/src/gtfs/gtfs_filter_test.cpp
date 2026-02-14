#include "gtfs/gtfs_filter.h"

#include <GUnit.h>

#include <algorithm>
#include <unordered_set>

using namespace vats5;
using ::testing::Eq;

// Helper to build a minimal Gtfs for synthetic tests.
static Gtfs MakeSyntheticGtfs() {
  Gtfs gtfs;

  // Two routes: R1 and R2
  gtfs.routes = {
      GtfsRoute{GtfsRouteId{"R1"}, "Route1", "Route One"},
      GtfsRoute{GtfsRouteId{"R2"}, "Route2", "Route Two"},
  };

  // Two directions per route
  gtfs.directions = {
      GtfsDirection{GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0}, "North"},
      GtfsDirection{GtfsRouteDirectionId{GtfsRouteId{"R1"}, 1}, "South"},
      GtfsDirection{GtfsRouteDirectionId{GtfsRouteId{"R2"}, 0}, "East"},
  };

  // Three stops
  gtfs.stops = {
      GtfsStop{GtfsStopId{"S1"}, "Stop 1", 37.0, -122.0, std::nullopt},
      GtfsStop{GtfsStopId{"S2"}, "Stop 2", 37.1, -122.1, std::nullopt},
      GtfsStop{GtfsStopId{"S3"}, "Stop 3", 37.2, -122.2, std::nullopt},
  };

  // Calendar: service "weekday" runs Mon-Fri, "weekend" runs Sat-Sun,
  // "latenight" runs Mon-Fri (has trips past midnight).
  // Date range covers July 2025.
  gtfs.calendar = {
      GtfsCalendar{
          GtfsServiceId{"weekday"},
          true, true, true, true, true, false, false,
          "20250701", "20250731"
      },
      GtfsCalendar{
          GtfsServiceId{"weekend"},
          false, false, false, false, false, true, true,
          "20250701", "20250731"
      },
      GtfsCalendar{
          GtfsServiceId{"latenight"},
          true, true, true, true, true, false, false,
          "20250701", "20250731"
      },
  };

  // Trips:
  // T1 (weekday, R1 dir 0) - normal daytime trip
  // T2 (weekday, R1 dir 1) - normal daytime trip
  // T3 (weekend, R2 dir 0) - weekend trip
  // T4 (latenight, R1 dir 0) - runs past midnight (times >= 24:00)
  // T5 (weekday, R2 dir 0) - early morning trip (times < 24:00)
  // PREFIX:T6 (weekday, R1 dir 0) - prefixed trip for prefix filtering test
  gtfs.trips = {
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0},
          GtfsTripId{"T1"}, GtfsServiceId{"weekday"}
      },
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R1"}, 1},
          GtfsTripId{"T2"}, GtfsServiceId{"weekday"}
      },
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R2"}, 0},
          GtfsTripId{"T3"}, GtfsServiceId{"weekend"}
      },
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0},
          GtfsTripId{"T4"}, GtfsServiceId{"latenight"}
      },
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R2"}, 0},
          GtfsTripId{"T5"}, GtfsServiceId{"weekday"}
      },
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0},
          GtfsTripId{"PREFIX:T6"}, GtfsServiceId{"weekday"}
      },
  };

  // Stop times:
  // T1: S1 08:00 -> S2 08:30
  // T2: S2 09:00 -> S1 09:30
  // T3: S1 10:00 -> S3 10:45 (weekend only)
  // T4: S1 25:00 -> S2 25:30 (past midnight, latenight service)
  // T5: S2 06:00 -> S3 06:30 (early morning)
  // PREFIX:T6: S1 12:00 -> S2 12:30
  gtfs.stop_times = {
      GtfsStopTime{
          GtfsTripId{"T1"}, GtfsStopId{"S1"}, 1,
          GtfsTimeSinceServiceStart{8 * 3600},
          GtfsTimeSinceServiceStart{8 * 3600}
      },
      GtfsStopTime{
          GtfsTripId{"T1"}, GtfsStopId{"S2"}, 2,
          GtfsTimeSinceServiceStart{8 * 3600 + 1800},
          GtfsTimeSinceServiceStart{8 * 3600 + 1800}
      },
      GtfsStopTime{
          GtfsTripId{"T2"}, GtfsStopId{"S2"}, 1,
          GtfsTimeSinceServiceStart{9 * 3600},
          GtfsTimeSinceServiceStart{9 * 3600}
      },
      GtfsStopTime{
          GtfsTripId{"T2"}, GtfsStopId{"S1"}, 2,
          GtfsTimeSinceServiceStart{9 * 3600 + 1800},
          GtfsTimeSinceServiceStart{9 * 3600 + 1800}
      },
      GtfsStopTime{
          GtfsTripId{"T3"}, GtfsStopId{"S1"}, 1,
          GtfsTimeSinceServiceStart{10 * 3600},
          GtfsTimeSinceServiceStart{10 * 3600}
      },
      GtfsStopTime{
          GtfsTripId{"T3"}, GtfsStopId{"S3"}, 2,
          GtfsTimeSinceServiceStart{10 * 3600 + 2700},
          GtfsTimeSinceServiceStart{10 * 3600 + 2700}
      },
      GtfsStopTime{
          GtfsTripId{"T4"}, GtfsStopId{"S1"}, 1,
          GtfsTimeSinceServiceStart{25 * 3600},
          GtfsTimeSinceServiceStart{25 * 3600}
      },
      GtfsStopTime{
          GtfsTripId{"T4"}, GtfsStopId{"S2"}, 2,
          GtfsTimeSinceServiceStart{25 * 3600 + 1800},
          GtfsTimeSinceServiceStart{25 * 3600 + 1800}
      },
      GtfsStopTime{
          GtfsTripId{"T5"}, GtfsStopId{"S2"}, 1,
          GtfsTimeSinceServiceStart{6 * 3600},
          GtfsTimeSinceServiceStart{6 * 3600}
      },
      GtfsStopTime{
          GtfsTripId{"T5"}, GtfsStopId{"S3"}, 2,
          GtfsTimeSinceServiceStart{6 * 3600 + 1800},
          GtfsTimeSinceServiceStart{6 * 3600 + 1800}
      },
      GtfsStopTime{
          GtfsTripId{"PREFIX:T6"}, GtfsStopId{"S1"}, 1,
          GtfsTimeSinceServiceStart{12 * 3600},
          GtfsTimeSinceServiceStart{12 * 3600}
      },
      GtfsStopTime{
          GtfsTripId{"PREFIX:T6"}, GtfsStopId{"S2"}, 2,
          GtfsTimeSinceServiceStart{12 * 3600 + 1800},
          GtfsTimeSinceServiceStart{12 * 3600 + 1800}
      },
  };

  return gtfs;
}

// ============================================================================
// RemoveUnreferencedTripsRoutesAndDirections tests
// ============================================================================

GTEST("RemoveUnreferencedTripsRoutesAndDirections removes orphaned trips") {
  GtfsDay day;
  day.stops = {
      GtfsStop{GtfsStopId{"S1"}, "Stop 1", 0.0, 0.0, std::nullopt},
  };
  day.routes = {
      GtfsRoute{GtfsRouteId{"R1"}, "R1", "Route 1"},
      GtfsRoute{GtfsRouteId{"R2"}, "R2", "Route 2"},
  };
  day.directions = {
      GtfsDirection{GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0}, "North"},
      GtfsDirection{GtfsRouteDirectionId{GtfsRouteId{"R2"}, 0}, "East"},
  };
  day.trips = {
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0},
          GtfsTripId{"T1"}, GtfsServiceId{"svc"}
      },
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R2"}, 0},
          GtfsTripId{"T2"}, GtfsServiceId{"svc"}
      },
  };
  // Only T1 has stop_times; T2 is orphaned.
  day.stop_times = {
      GtfsStopTime{
          GtfsTripId{"T1"}, GtfsStopId{"S1"}, 1,
          GtfsTimeSinceServiceStart{3600},
          GtfsTimeSinceServiceStart{3600}
      },
  };

  RemoveUnreferencedTripsRoutesAndDirections(day);

  EXPECT_EQ(day.trips.size(), 1);
  EXPECT_EQ(day.trips[0].trip_id.v, "T1");
  EXPECT_EQ(day.routes.size(), 1);
  EXPECT_EQ(day.routes[0].route_id.v, "R1");
  EXPECT_EQ(day.directions.size(), 1);
  EXPECT_EQ(day.directions[0].route_direction_id.route_id.v, "R1");
}

GTEST("RemoveUnreferencedTripsRoutesAndDirections keeps all when all referenced"
) {
  GtfsDay day;
  day.routes = {GtfsRoute{GtfsRouteId{"R1"}, "R1", ""}};
  day.directions = {
      GtfsDirection{GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0}, "N"}
  };
  day.trips = {
      GtfsTrip{
          GtfsRouteDirectionId{GtfsRouteId{"R1"}, 0},
          GtfsTripId{"T1"}, GtfsServiceId{"s"}
      },
  };
  day.stop_times = {
      GtfsStopTime{
          GtfsTripId{"T1"}, GtfsStopId{"S1"}, 1,
          GtfsTimeSinceServiceStart{0},
          GtfsTimeSinceServiceStart{0}
      },
  };

  RemoveUnreferencedTripsRoutesAndDirections(day);

  EXPECT_EQ(day.trips.size(), 1);
  EXPECT_EQ(day.routes.size(), 1);
  EXPECT_EQ(day.directions.size(), 1);
}

// ============================================================================
// GtfsFilterByPrefixes tests
// ============================================================================

GTEST("GtfsFilterByPrefixes filters by single prefix") {
  Gtfs gtfs = MakeSyntheticGtfs();

  Gtfs filtered = GtfsFilterByPrefixes(gtfs, {"PREFIX:"});

  // Only PREFIX:T6 should remain
  EXPECT_EQ(filtered.trips.size(), 1);
  EXPECT_EQ(filtered.trips[0].trip_id.v, "PREFIX:T6");
  EXPECT_EQ(filtered.stop_times.size(), 2);
  // Route R1 should be present (PREFIX:T6 uses R1)
  EXPECT_EQ(filtered.routes.size(), 1);
  EXPECT_EQ(filtered.routes[0].route_id.v, "R1");
}

GTEST("GtfsFilterByPrefixes filters by multiple prefixes") {
  Gtfs gtfs = MakeSyntheticGtfs();

  // "T" matches T1-T5, "PREFIX:" matches PREFIX:T6
  Gtfs filtered = GtfsFilterByPrefixes(gtfs, {"PREFIX:", "T"});

  // All 6 trips should match
  EXPECT_EQ(filtered.trips.size(), 6);
}

GTEST("GtfsFilterByPrefixes with no matching prefix returns empty") {
  Gtfs gtfs = MakeSyntheticGtfs();

  Gtfs filtered = GtfsFilterByPrefixes(gtfs, {"NONEXISTENT:"});

  EXPECT_EQ(filtered.trips.size(), 0);
  EXPECT_EQ(filtered.stop_times.size(), 0);
  EXPECT_EQ(filtered.routes.size(), 0);
  EXPECT_EQ(filtered.directions.size(), 0);
}

// ============================================================================
// GtfsFilterDateWithServiceDays tests (synthetic data)
// ============================================================================

GTEST("GtfsFilterDateWithServiceDays includes target day trips") {
  Gtfs gtfs = MakeSyntheticGtfs();

  // 20250708 is a Tuesday (weekday)
  GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, "20250708");

  // Should include weekday trips from target day
  std::unordered_set<std::string> trip_ids;
  for (const auto& trip : result.trips) {
    trip_ids.insert(trip.trip_id.v);
  }

  EXPECT_TRUE(trip_ids.count("T1")) << "Target day trip T1 should be present";
  EXPECT_TRUE(trip_ids.count("T2")) << "Target day trip T2 should be present";
  EXPECT_TRUE(trip_ids.count("T5")) << "Target day trip T5 should be present";
  EXPECT_TRUE(trip_ids.count("PREFIX:T6"))
      << "Target day trip PREFIX:T6 should be present";
  // Weekend trip T3 should not be present
  EXPECT_FALSE(trip_ids.count("T3"))
      << "Weekend trip T3 should not be on a weekday";
}

GTEST("GtfsFilterDateWithServiceDays includes previous day late-night trips") {
  Gtfs gtfs = MakeSyntheticGtfs();

  // 20250708 is a Tuesday. Previous day (Monday) also has weekday service.
  // T4 runs at 25:00 on the previous day, so it should appear with :prev-sd
  // suffix and shifted times.
  GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, "20250708");

  std::unordered_set<std::string> trip_ids;
  for (const auto& trip : result.trips) {
    trip_ids.insert(trip.trip_id.v);
  }

  EXPECT_TRUE(trip_ids.count("T4:prev-sd"))
      << "Previous day late-night trip T4:prev-sd should be present";

  // Check that times are shifted by -24h: 25:00 - 24:00 = 01:00 = 3600s
  for (const auto& st : result.stop_times) {
    if (st.trip_id.v == "T4:prev-sd" && st.stop_sequence == 1) {
      EXPECT_EQ(st.arrival_time.seconds, 1 * 3600)
          << "Previous day trip time should be shifted by -24h";
    }
    if (st.trip_id.v == "T4:prev-sd" && st.stop_sequence == 2) {
      EXPECT_EQ(st.arrival_time.seconds, 1 * 3600 + 1800)
          << "Previous day trip time should be shifted by -24h";
    }
  }
}

GTEST("GtfsFilterDateWithServiceDays includes next day early-morning trips") {
  Gtfs gtfs = MakeSyntheticGtfs();

  // 20250708 is Tuesday. Next day (Wednesday) also has weekday service.
  // T5 runs at 06:00 (< 24:00) on the next day, so it should appear with
  // :next-sd suffix and times shifted by +24h.
  GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, "20250708");

  std::unordered_set<std::string> trip_ids;
  for (const auto& trip : result.trips) {
    trip_ids.insert(trip.trip_id.v);
  }

  EXPECT_TRUE(trip_ids.count("T5:next-sd"))
      << "Next day early-morning trip T5:next-sd should be present";

  // Check that times are shifted by +24h: 06:00 + 24:00 = 30:00 = 108000s
  for (const auto& st : result.stop_times) {
    if (st.trip_id.v == "T5:next-sd" && st.stop_sequence == 1) {
      EXPECT_EQ(st.arrival_time.seconds, 30 * 3600)
          << "Next day trip time should be shifted by +24h";
    }
    if (st.trip_id.v == "T5:next-sd" && st.stop_sequence == 2) {
      EXPECT_EQ(st.arrival_time.seconds, 30 * 3600 + 1800)
          << "Next day trip time should be shifted by +24h";
    }
  }
}

GTEST("GtfsFilterDateWithServiceDays on weekend excludes weekday trips") {
  Gtfs gtfs = MakeSyntheticGtfs();

  // 20250712 is a Saturday
  GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, "20250712");

  std::unordered_set<std::string> trip_ids;
  for (const auto& trip : result.trips) {
    trip_ids.insert(trip.trip_id.v);
  }

  // Weekend trip T3 should be present as a target day trip
  EXPECT_TRUE(trip_ids.count("T3")) << "Weekend trip T3 should be present";

  // Weekday trips should NOT be on target day (only possibly as prev/next sd)
  EXPECT_FALSE(trip_ids.count("T1"))
      << "Weekday trip T1 should not be on Saturday";
  EXPECT_FALSE(trip_ids.count("T2"))
      << "Weekday trip T2 should not be on Saturday";
}

GTEST(
    "GtfsFilterDateWithServiceDays weekday-weekend boundary handles prev-sd "
    "correctly"
) {
  Gtfs gtfs = MakeSyntheticGtfs();

  // 20250712 is Saturday. Previous day (Friday) has weekday service.
  // T4 (latenight, runs at 25:00) should appear as prev-sd.
  GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, "20250712");

  std::unordered_set<std::string> trip_ids;
  for (const auto& trip : result.trips) {
    trip_ids.insert(trip.trip_id.v);
  }

  EXPECT_TRUE(trip_ids.count("T4:prev-sd"))
      << "Friday late-night trip should carry over to Saturday";
}

GTEST("GtfsFilterDateWithServiceDays all stop_times reference valid trips") {
  Gtfs gtfs = MakeSyntheticGtfs();
  GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, "20250708");

  std::unordered_set<std::string> trip_ids;
  for (const auto& trip : result.trips) {
    trip_ids.insert(trip.trip_id.v);
  }

  for (const auto& st : result.stop_times) {
    EXPECT_TRUE(trip_ids.count(st.trip_id.v))
        << "Stop time references unknown trip: " << st.trip_id.v;
  }
}

// ============================================================================
// Tests on real data (raw_RG_202506)
// ============================================================================

static Gtfs* getGlobalGtfs() {
  static Gtfs* gtfs = nullptr;
  if (!gtfs) {
    std::string gtfs_directory_path = "../data/raw_RG_202506";
    gtfs = new Gtfs(GtfsLoad(gtfs_directory_path));
  }
  return gtfs;
}

GTEST("GtfsFilterByPrefixes on real data filters CT trips") {
  const Gtfs& gtfs = *getGlobalGtfs();

  Gtfs filtered = GtfsFilterByPrefixes(gtfs, {"CT:"});

  // Exact counts for the CT: prefix in raw_RG_202506
  EXPECT_EQ(filtered.trips.size(), 178);
  EXPECT_EQ(filtered.routes.size(), 5);

  // CT:507 (Express northbound) should be present
  bool found_507 = false;
  for (const auto& trip : filtered.trips) {
    if (trip.trip_id.v == "CT:507") {
      found_507 = true;
      EXPECT_EQ(trip.route_direction_id.route_id.v, "CT:Express");
      EXPECT_EQ(trip.route_direction_id.direction_id, 0);
      EXPECT_EQ(trip.service_id.v, "CT:72982");
    }
  }
  EXPECT_TRUE(found_507) << "CT:507 should be in the filtered data";

  // Non-CT trip SR:198 should be absent
  for (const auto& trip : filtered.trips) {
    EXPECT_NE(trip.trip_id.v, "SR:198")
        << "Non-CT trip SR:198 should not be in CT-filtered data";
  }
}

GTEST("GtfsFilterByPrefixes on real data filters multiple prefixes") {
  const Gtfs& gtfs = *getGlobalGtfs();

  Gtfs ct_only = GtfsFilterByPrefixes(gtfs, {"CT:"});
  Gtfs sr_only = GtfsFilterByPrefixes(gtfs, {"SR:"});
  Gtfs both = GtfsFilterByPrefixes(gtfs, {"CT:", "SR:"});

  EXPECT_EQ(ct_only.trips.size(), 178);
  EXPECT_EQ(both.trips.size(), ct_only.trips.size() + sr_only.trips.size());
}

GTEST(
    "GtfsFilterDateWithServiceDays on real data includes target day CT:507"
) {
  const Gtfs& gtfs = *getGlobalGtfs();

  // Tuesday 2025-07-08. CT:507 is an Express northbound trip on weekday
  // service CT:72982.
  GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, "20250708");

  // Find CT:507 and check its stop times (San Jose Diridon -> SF 4th & King)
  std::vector<GtfsStopTime> trip_507_times;
  for (const auto& st : result.stop_times) {
    if (st.trip_id.v == "CT:507") {
      trip_507_times.push_back(st);
    }
  }
  std::sort(trip_507_times.begin(), trip_507_times.end(),
      [](const GtfsStopTime& a, const GtfsStopTime& b) {
        return a.stop_sequence < b.stop_sequence;
      });

  ASSERT_EQ(trip_507_times.size(), 11);

  // First stop: San Jose Diridon Northbound (70261) at 07:22 = 26520s
  EXPECT_EQ(trip_507_times[0].stop_id.v, "70261");
  EXPECT_EQ(trip_507_times[0].stop_sequence, 1);
  EXPECT_EQ(trip_507_times[0].arrival_time.seconds, 26520);

  // Middle stop: Redwood City Northbound (70141) at 07:49 = 28140s
  EXPECT_EQ(trip_507_times[4].stop_id.v, "70141");
  EXPECT_EQ(trip_507_times[4].stop_sequence, 5);
  EXPECT_EQ(trip_507_times[4].arrival_time.seconds, 28140);

  // Last stop: SF 4th & King Northbound (70011) at 08:22 = 30120s
  EXPECT_EQ(trip_507_times[10].stop_id.v, "70011");
  EXPECT_EQ(trip_507_times[10].stop_sequence, 11);
  EXPECT_EQ(trip_507_times[10].arrival_time.seconds, 30120);
}

GTEST(
    "GtfsFilterDateWithServiceDays on real data includes prev-sd CT:176"
) {
  const Gtfs& gtfs = *getGlobalGtfs();

  // Tuesday 2025-07-08. CT:176 is a Local Weekday southbound trip that runs
  // past midnight on Monday (service CT:72982). Its first stop departs at
  // 24:05:00 (86700s). After -24h shift it becomes 00:05:00 (300s).
  GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, "20250708");

  std::vector<GtfsStopTime> trip_176_times;
  for (const auto& st : result.stop_times) {
    if (st.trip_id.v == "CT:176:prev-sd") {
      trip_176_times.push_back(st);
    }
  }
  std::sort(trip_176_times.begin(), trip_176_times.end(),
      [](const GtfsStopTime& a, const GtfsStopTime& b) {
        return a.stop_sequence < b.stop_sequence;
      });

  ASSERT_EQ(trip_176_times.size(), 22);

  // First stop: SF 4th & King Southbound (70012) at 24:05 - 24:00 = 00:05 =
  // 300s
  EXPECT_EQ(trip_176_times[0].stop_id.v, "70012");
  EXPECT_EQ(trip_176_times[0].stop_sequence, 1);
  EXPECT_EQ(trip_176_times[0].arrival_time.seconds, 300);

  // Last stop: San Jose Diridon Southbound (70262) at 25:23 - 24:00 = 01:23 =
  // 4980s
  EXPECT_EQ(trip_176_times[21].stop_id.v, "70262");
  EXPECT_EQ(trip_176_times[21].stop_sequence, 22);
  EXPECT_EQ(trip_176_times[21].arrival_time.seconds, 4980);

  // Verify the trip itself has :prev-sd suffix and correct route
  bool found = false;
  for (const auto& trip : result.trips) {
    if (trip.trip_id.v == "CT:176:prev-sd") {
      found = true;
      EXPECT_EQ(trip.route_direction_id.route_id.v, "CT:Local Weekday");
      EXPECT_EQ(trip.route_direction_id.direction_id, 1);
    }
  }
  EXPECT_TRUE(found) << "CT:176:prev-sd should be present";
}

GTEST(
    "GtfsFilterDateWithServiceDays on real data includes next-sd CT:101"
) {
  const Gtfs& gtfs = *getGlobalGtfs();

  // Tuesday 2025-07-08. CT:101 is a Local Weekday northbound trip on
  // Wednesday's service (CT:72982). Its first stop departs at 04:43:00
  // (16980s). After +24h shift it becomes 28:43:00 (103380s).
  GtfsDay result = GtfsFilterDateWithServiceDays(gtfs, "20250708");

  std::vector<GtfsStopTime> trip_101_times;
  for (const auto& st : result.stop_times) {
    if (st.trip_id.v == "CT:101:next-sd") {
      trip_101_times.push_back(st);
    }
  }
  std::sort(trip_101_times.begin(), trip_101_times.end(),
      [](const GtfsStopTime& a, const GtfsStopTime& b) {
        return a.stop_sequence < b.stop_sequence;
      });

  ASSERT_EQ(trip_101_times.size(), 22);

  // First stop: San Jose Diridon Northbound (70261) at 04:43 + 24:00 = 28:43
  // = 103380s
  EXPECT_EQ(trip_101_times[0].stop_id.v, "70261");
  EXPECT_EQ(trip_101_times[0].stop_sequence, 1);
  EXPECT_EQ(trip_101_times[0].arrival_time.seconds, 103380);

  // Last stop: SF 4th & King Northbound (70011) at 06:01 + 24:00 = 30:01 =
  // 108060s
  EXPECT_EQ(trip_101_times[21].stop_id.v, "70011");
  EXPECT_EQ(trip_101_times[21].stop_sequence, 22);
  EXPECT_EQ(trip_101_times[21].arrival_time.seconds, 108060);

  // Verify the trip itself has :next-sd suffix and correct route
  bool found = false;
  for (const auto& trip : result.trips) {
    if (trip.trip_id.v == "CT:101:next-sd") {
      found = true;
      EXPECT_EQ(trip.route_direction_id.route_id.v, "CT:Local Weekday");
      EXPECT_EQ(trip.route_direction_id.direction_id, 0);
    }
  }
  EXPECT_TRUE(found) << "CT:101:next-sd should be present";
}

GTEST("GtfsFilterDateWithServiceDays excludes weekend trip on weekday") {
  const Gtfs& gtfs = *getGlobalGtfs();

  // Tuesday 2025-07-08. CT:Local Weekend trips should NOT appear.
  Gtfs ct_only = GtfsFilterByPrefixes(gtfs, {"CT:"});
  GtfsDay result = GtfsFilterDateWithServiceDays(ct_only, "20250708");

  for (const auto& trip : result.trips) {
    EXPECT_NE(trip.route_direction_id.route_id.v, "CT:Local Weekend")
        << "Weekend route should not appear on Tuesday, trip: "
        << trip.trip_id.v;
  }
}
