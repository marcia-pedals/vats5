#include <gtest/gtest.h>
#include "solver/data.h"
#include "gtfs/gtfs.h"
#include <unordered_set>

namespace vats5 {

static Gtfs* getGlobalGtfs() {
  static Gtfs* gtfs = nullptr;
  if (!gtfs) {
    std::string gtfs_directory_path = "../data/RG";
    gtfs = new Gtfs(GtfsLoad(gtfs_directory_path));
  }
  return gtfs;
}

TEST(DataTest, GetStepsFromGtfsPlaceholder) {
    const Gtfs& gtfs = *getGlobalGtfs();
    GtfsDay gtfs_day = GtfsFilterByDate(gtfs, "20250718");
    const GtfsTripId trip_id{"CT:507"};
    
    std::unordered_set<GtfsTripId> trips_set = {trip_id};
    GtfsDay filtered = GtfsFilterByTrips(gtfs_day, trips_set);
    
    StepsFromGtfs result = GetStepsFromGtfs(filtered);
    
    EXPECT_TRUE(result.steps.empty());
}

}  // namespace vats5