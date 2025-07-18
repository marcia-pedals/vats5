#include "solver/data.h"

namespace vats5 {

StepsFromGtfs GetStepsFromGtfs(GtfsDay gtfs) {
    StepsFromGtfs result;
    result.mapping = DataGtfsMapping{};
    result.steps = {};
    return result;
}

}  // namespace vats5