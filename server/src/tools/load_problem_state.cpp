#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

#include "solver/tarel_graph.h"

using namespace vats5;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: load_problem_state <input_path.json>\n";
        return 1;
    }

    std::string input_path = argv[1];
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
