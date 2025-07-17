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

struct RouteDirectionId {
    RouteId route_id;
    int direction_id;
};

struct Stop {
    StopId stop_id;
    std::string stop_name;
    double stop_lat;
    double stop_lon;
    std::optional<StopId> parent_station;
};

struct Trip {
    RouteDirectionId route_direction_id;
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

struct StopTime {
    TripId trip_id;
    StopId stop_id;
    int stop_sequence;
    std::string arrival_time;
    std::string departure_time;
};

struct Route {
    RouteId route_id;
     std::string route_short_name;
    std::string route_long_name;
};

struct Direction {
    RouteDirectionId route_direction_id;
    std::string direction;
};

// Load all stops from the GTFS stops.txt file
std::vector<Stop> GtfsLoadStops(const std::string& stops_file_path);

// Load all trips from the GTFS trips.txt file
std::vector<Trip> GtfsLoadTrips(const std::string& trips_file_path);

// Load all calendar entries from the GTFS calendar.txt file
std::vector<Calendar> GtfsLoadCalendar(const std::string& calendar_file_path);

// Load all stop times from the GTFS stop_times.txt file
std::vector<StopTime> GtfsLoadStopTimes(const std::string& stop_times_file_path);

// Load all routes from the GTFS routes.txt file
std::vector<Route> GtfsLoadRoutes(const std::string& routes_file_path);

// Load all directions from the GTFS directions.txt file
std::vector<Direction> GtfsLoadDirections(const std::string& directions_file_path);

} // namespace vats5
