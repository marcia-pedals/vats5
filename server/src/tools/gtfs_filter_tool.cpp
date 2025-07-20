#include "gtfs/gtfs.h"

#include <iostream>
#include <string>
#include <vector>
#include <unordered_set>
#include <sstream>
#include <filesystem>

using namespace vats5;

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <input_dir> <date> <trip_ids> <output_dir>\n"
              << "\n"
              << "GTFS Filter Tool - Filters GTFS data by date and trip IDs\n"
              << "\n"
              << "Arguments:\n"
              << "  input_dir   Directory containing GTFS files (stops.txt, trips.txt, etc.)\n"
              << "  date        Date in YYYYMMDD format (e.g., 20250708)\n"
              << "  trip_ids    Comma-separated list of trip IDs to include (e.g., \"CT:507,SR:198\")\n"
              << "  output_dir  Directory where filtered GTFS files will be saved\n"
              << "\n"
              << "Description:\n"
              << "  This tool loads GTFS data from the input directory, filters it for the\n"
              << "  specified date and trip IDs, then saves the filtered data to the output\n"
              << "  directory. The output will contain the same GTFS file structure but only\n"
              << "  with data relevant to the specified trips on the given date.\n"
              << "\n"
              << "Example:\n"
              << "  " << program_name << " ../data/RG 20250708 \"CT:507,SR:198\" ./filtered_gtfs\n"
              << "\n"
              << "  This filters GTFS data for July 8, 2025, including only trips CT:507 and\n"
              << "  SR:198, and saves the result to ./filtered_gtfs/\n"
              << std::endl;
}

std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        // Trim whitespace
        size_t start = token.find_first_not_of(" \t\r\n");
        size_t end = token.find_last_not_of(" \t\r\n");
        
        if (start != std::string::npos && end != std::string::npos) {
            tokens.push_back(token.substr(start, end - start + 1));
        } else if (start != std::string::npos) {
            tokens.push_back(token.substr(start));
        }
    }
    
    return tokens;
}

bool isValidDate(const std::string& date) {
    if (date.length() != 8) return false;
    
    for (char c : date) {
        if (c < '0' || c > '9') return false;
    }
    
    // Basic validation - could be more thorough
    int year = std::stoi(date.substr(0, 4));
    int month = std::stoi(date.substr(4, 2));
    int day = std::stoi(date.substr(6, 2));
    
    return year >= 1900 && year <= 2100 && 
           month >= 1 && month <= 12 && 
           day >= 1 && day <= 31;
}

int main(int argc, char* argv[]) {
    // Check argument count
    if (argc != 5) {
        if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
            printUsage(argv[0]);
            return 0;
        }
        
        std::cerr << "Error: Expected 4 arguments, got " << (argc - 1) << std::endl;
        std::cerr << "Use --help for usage information." << std::endl;
        return 1;
    }
    
    std::string input_dir = argv[1];
    std::string date = argv[2];
    std::string trip_ids_str = argv[3];
    std::string output_dir = argv[4];
    
    // Validate arguments
    if (!std::filesystem::exists(input_dir)) {
        std::cerr << "Error: Input directory '" << input_dir << "' does not exist." << std::endl;
        return 1;
    }
    
    if (!std::filesystem::is_directory(input_dir)) {
        std::cerr << "Error: Input path '" << input_dir << "' is not a directory." << std::endl;
        return 1;
    }
    
    if (!isValidDate(date)) {
        std::cerr << "Error: Invalid date format '" << date << "'. Expected YYYYMMDD format." << std::endl;
        return 1;
    }
    
    if (trip_ids_str.empty()) {
        std::cerr << "Error: Trip IDs cannot be empty." << std::endl;
        return 1;
    }
    
    // Parse trip IDs
    std::vector<std::string> trip_id_list = split(trip_ids_str, ',');
    if (trip_id_list.empty()) {
        std::cerr << "Error: No valid trip IDs found in '" << trip_ids_str << "'." << std::endl;
        return 1;
    }
    
    std::unordered_set<GtfsTripId> trip_ids_set;
    for (const auto& trip_id_str : trip_id_list) {
        trip_ids_set.insert(GtfsTripId{trip_id_str});
    }
    
    std::cout << "Loading GTFS data from: " << input_dir << std::endl;
    std::cout << "Filtering for date: " << date << std::endl;
    std::cout << "Including " << trip_ids_set.size() << " trip(s): ";
    for (size_t i = 0; i < trip_id_list.size(); ++i) {
        std::cout << trip_id_list[i];
        if (i < trip_id_list.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
    std::cout << "Output directory: " << output_dir << std::endl;
    std::cout << std::endl;
    
    try {
        // Load GTFS data
        std::cout << "Loading GTFS data..." << std::endl;
        Gtfs gtfs = GtfsLoad(input_dir);
        
        std::cout << "Loaded: " << gtfs.stops.size() << " stops, "
                  << gtfs.trips.size() << " trips, "
                  << gtfs.stop_times.size() << " stop times, "
                  << gtfs.routes.size() << " routes, "
                  << gtfs.directions.size() << " directions" << std::endl;
        
        // Filter by date
        std::cout << "Filtering by date: " << date << "..." << std::endl;
        GtfsDay gtfs_day = GtfsFilterByDate(gtfs, date);
        
        std::cout << "After date filter: " << gtfs_day.stops.size() << " stops, "
                  << gtfs_day.trips.size() << " trips, "
                  << gtfs_day.stop_times.size() << " stop times, "
                  << gtfs_day.routes.size() << " routes, "
                  << gtfs_day.directions.size() << " directions" << std::endl;
        
        if (gtfs_day.trips.empty()) {
            std::cerr << "Warning: No trips found for date " << date << ". Check if the date is within the service period." << std::endl;
        }
        
        // Filter by trip IDs
        std::cout << "Filtering by trip IDs..." << std::endl;
        GtfsDay filtered_gtfs = GtfsFilterByTrips(gtfs_day, trip_ids_set);
        
        std::cout << "After trip filter: " << filtered_gtfs.stops.size() << " stops, "
                  << filtered_gtfs.trips.size() << " trips, "
                  << filtered_gtfs.stop_times.size() << " stop times, "
                  << filtered_gtfs.routes.size() << " routes, "
                  << filtered_gtfs.directions.size() << " directions" << std::endl;
        
        if (filtered_gtfs.trips.empty()) {
            std::cerr << "Warning: No trips found matching the specified trip IDs for date " << date << "." << std::endl;
            std::cerr << "Make sure the trip IDs exist and are active on the given date." << std::endl;
        }
        
        // Save to output directory
        std::cout << "Saving filtered data to: " << output_dir << "..." << std::endl;
        GtfsSave(filtered_gtfs, output_dir);
        
        std::cout << "Successfully saved filtered GTFS data!" << std::endl;
        std::cout << "Output files created in: " << output_dir << std::endl;
        
        // List created files
        std::cout << "\nCreated files:" << std::endl;
        for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
            if (entry.is_regular_file()) {
                auto file_size = std::filesystem::file_size(entry.path());
                std::cout << "  " << entry.path().filename().string() 
                          << " (" << file_size << " bytes)" << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}