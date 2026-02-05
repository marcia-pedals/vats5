#include <fstream>
#include <iostream>
#include <string>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include "solver/tarel_graph.h"

using namespace vats5;

int main(int argc, char* argv[]) {
    CLI::App app{"Load and display problem state from JSON"};

    std::string input_path;
    app.add_option("input_path", input_path, "Input JSON file path")
        ->required();

    CLI11_PARSE(app, argc, argv);
    std::cout << "Loading problem state from: " << input_path << "\n";

    std::ifstream in(input_path);
    if (!in.is_open()) {
        std::cerr << "Error: could not open " << input_path << "\n";
        return 1;
    }

    nlohmann::json j = nlohmann::json::parse(in);
    ProblemState state = j.get<ProblemState>();

    std::cout << "Loaded successfully.\n\n";
    std::cout << "Number of stops: " << state.minimal.NumStops() << "\n";
    std::cout << "Number of required stops: " << state.required_stops.size() << "\n";
    std::cout << "Number of minimal steps: " << state.minimal.AllSteps().size() << "\n";
    std::cout << "Boundary: START=" << state.boundary.start.v
              << " END=" << state.boundary.end.v << "\n";
    std::cout << "Step partitions: " << state.step_partition_names.size() << "\n";
    std::cout << "\n";

    // Okay so the next thing I want to do: Figure out what some paths are that are preventing some of the stops from being "inner".

    return 0;
}
