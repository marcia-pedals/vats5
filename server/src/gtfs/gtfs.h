#pragma once

#include <string>
#include <vector>
#include <optional>

namespace vats5 {

struct StopId {
    std::string v;
};

struct RouteId {
    std::string v;
};

struct TripId {
    std::string v;
};

struct ServiceId {
    std::string v;
};

struct Stop {
    StopId stop_id;
    std::string stop_name;
    double stop_lat;
    double stop_lon;
    std::optional<StopId> parent_station;
};

struct Trip {
    RouteId route_id;
    TripId trip_id;
    ServiceId service_id;
};

struct Calendar {
    ServiceId service_id;
    bool monday;
    bool tuesday;
    bool wednesday;
    bool thursday;
    bool friday;
    bool saturday;
    bool sunday;
    std::string start_date;
    std::string end_date;
};

// Load all stops from the GTFS stops.txt file
std::vector<Stop> GtfsLoadStops(const std::string& stops_file_path);

// Load all trips from the GTFS trips.txt file
std::vector<Trip> GtfsLoadTrips(const std::string& trips_file_path);

// Load all calendar entries from the GTFS calendar.txt file
std::vector<Calendar> GtfsLoadCalendar(const std::string& calendar_file_path);

} // namespace vats5
