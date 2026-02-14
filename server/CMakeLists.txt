cmake_minimum_required(VERSION 3.20)
include(FetchContent)

project(vats5
    VERSION 1.0.0
    DESCRIPTION "Visit All The Stops!!!"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Crow)
find_package(nlohmann_json REQUIRED)

FetchContent_Declare(
  csv
  GIT_REPOSITORY https://github.com/StaflSystems/csv-parser.git
  GIT_TAG f30f50d3b4f6c4c0973dacb8e319faaca79d3960  # PR #277: fix --coverage linker error
)
FetchContent_MakeAvailable(csv)

FetchContent_Declare(
  gunit
  GIT_REPOSITORY https://github.com/cpp-testing/GUnit.git
  GIT_TAG        master
)
FETCHCONTENT_MAKEAVAILABLE(gunit)

FetchContent_Declare(
  rapidcheck
  GIT_REPOSITORY https://github.com/emil-e/rapidcheck.git
  GIT_TAG        master
)
set(RC_ENABLE_GTEST ON CACHE BOOL "Enable RapidCheck GTest integration")
FetchContent_MakeAvailable(rapidcheck)

FetchContent_Declare(
  cli11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG        v2.4.2
)
FetchContent_MakeAvailable(cli11)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

# Create date utility library target
add_library(date STATIC src/util/date.cpp)
target_include_directories(date PUBLIC src)

# Create GTFS library target
add_library(gtfs STATIC src/gtfs/gtfs.cpp)
target_link_libraries(gtfs PUBLIC csv nlohmann_json::nlohmann_json)
target_include_directories(gtfs PUBLIC src)

# Create data library target
add_library(data STATIC src/solver/data.cpp)
target_link_libraries(data PUBLIC gtfs)
target_include_directories(data PUBLIC src)

# Create graph_util library target
add_library(graph_util STATIC src/solver/graph_util.cpp)
target_link_libraries(graph_util PUBLIC data)
target_include_directories(graph_util PUBLIC src)

# Create step_merge library target
add_library(step_merge STATIC src/solver/step_merge.cpp)
target_link_libraries(step_merge PUBLIC data)
target_include_directories(step_merge PUBLIC src)

# Create steps_adjacency_list library target
add_library(steps_adjacency_list STATIC src/solver/steps_adjacency_list.cpp)
target_link_libraries(steps_adjacency_list PUBLIC step_merge)
target_include_directories(steps_adjacency_list PUBLIC src)

# Create relaxed_adjacency_list library target
add_library(relaxed_adjacency_list STATIC src/solver/relaxed_adjacency_list.cpp)
target_link_libraries(relaxed_adjacency_list PUBLIC steps_adjacency_list)
target_include_directories(relaxed_adjacency_list PUBLIC src)

# Create concorde library target
add_library(concorde STATIC src/solver/concorde.cpp)
target_link_libraries(concorde PUBLIC relaxed_adjacency_list)
target_include_directories(concorde PUBLIC src)

# Create relaxed_shortest_path library target
add_library(relaxed_shortest_path STATIC src/solver/relaxed_shortest_path.cpp)
target_link_libraries(relaxed_shortest_path PUBLIC relaxed_adjacency_list)
target_include_directories(relaxed_shortest_path PUBLIC src)

# Create steps_shortest_path library target
add_library(steps_shortest_path STATIC src/solver/steps_shortest_path.cpp)
target_link_libraries(steps_shortest_path PUBLIC relaxed_shortest_path)
target_include_directories(steps_shortest_path PUBLIC src)

# Create debug_printing library target
add_library(debug_printing STATIC src/solver/debug_printing.cpp)
target_link_libraries(debug_printing PUBLIC tarel_graph)
target_include_directories(debug_printing PUBLIC src)

# Create branch_and_bound library target
add_library(branch_and_bound STATIC src/solver/branch_and_bound.cpp)
target_link_libraries(branch_and_bound PUBLIC tarel_graph)
target_include_directories(branch_and_bound PUBLIC src)

# Create tarel_graph library target
add_library(tarel_graph STATIC src/solver/tarel_graph.cpp)
target_link_libraries(tarel_graph PUBLIC steps_shortest_path concorde graph_util)
target_include_directories(tarel_graph PUBLIC src)

# Create visualization library target
add_library(visualization STATIC src/visualization/visualization.cpp)
target_link_libraries(visualization PUBLIC steps_shortest_path)
target_include_directories(visualization PUBLIC src)

add_executable(main_server main.cpp)
target_link_libraries(main_server PUBLIC Crow::Crow gtfs)

# Add GTFS filter tool
add_executable(gtfs_filter_tool src/tools/gtfs_filter_tool.cpp)
target_link_libraries(gtfs_filter_tool PUBLIC gtfs date CLI11::CLI11)

# Add benchmark reduction tool
add_executable(benchmark_reduction src/tools/benchmark_reduction.cpp)
target_link_libraries(benchmark_reduction PUBLIC steps_shortest_path CLI11::CLI11)

# Add reduction diff tool
add_executable(reduction_diff src/tools/reduction_diff.cpp)
target_link_libraries(reduction_diff PUBLIC steps_shortest_path CLI11::CLI11)

# Add plain_viz tool (minimal paths only, no iteration)
add_executable(plain_viz src/tools/plain_viz.cpp)
target_link_libraries(plain_viz PUBLIC visualization)

# Add branch_and_bound_dev tool
add_executable(branch_and_bound_dev src/tools/branch_and_bound_dev.cpp)
target_link_libraries(branch_and_bound_dev PUBLIC tarel_graph branch_and_bound)

# Add initialize_problem_state tool
add_executable(initialize_problem_state src/tools/initialize_problem_state.cpp)
target_link_libraries(initialize_problem_state PUBLIC tarel_graph CLI11::CLI11)

# Add load_problem_state tool
add_executable(load_problem_state src/tools/load_problem_state.cpp)
target_link_libraries(load_problem_state PUBLIC tarel_graph CLI11::CLI11)

# Add iterative_expansion tool
add_executable(iterative_expansion src/tools/iterative_expansion.cpp)
target_link_libraries(iterative_expansion PUBLIC tarel_graph CLI11::CLI11)

# Test utility libraries (in src/solver/test_util/)
add_library(cached_test_data INTERFACE)
target_link_libraries(cached_test_data INTERFACE steps_adjacency_list)
target_include_directories(cached_test_data INTERFACE src)

add_library(problem_state_gen src/solver/test_util/problem_state_gen.cpp)
target_link_libraries(problem_state_gen PUBLIC tarel_graph rapidcheck)
target_include_directories(problem_state_gen PUBLIC src)

add_library(naive_solve src/solver/test_util/naive_solve.cpp)
target_link_libraries(naive_solve PUBLIC tarel_graph step_merge)
target_include_directories(naive_solve PUBLIC src)

# Enable CMake testing
enable_testing()
set(CMAKE_CTEST_ARGUMENTS "--label-exclude;slow")

add_custom_target(test-all
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Add test executable
add_executable(gtfs_test src/gtfs/gtfs_test.cpp)
target_link_libraries(gtfs_test PUBLIC gunit gtfs)
add_test(NAME gtfs_test COMMAND gtfs_test)

# Add data test executable
add_executable(data_test src/solver/data_test.cpp)
target_link_libraries(data_test PUBLIC gunit data)
add_test(NAME data_test COMMAND data_test)

# Add step_merge test executable
add_executable(step_merge_test src/solver/step_merge_test.cpp)
target_link_libraries(step_merge_test PUBLIC gunit step_merge rapidcheck rapidcheck_gtest)
add_test(NAME step_merge_test COMMAND step_merge_test)

# Add steps_adjacency_list test executable
add_executable(steps_adjacency_list_test src/solver/steps_adjacency_list_test.cpp)
target_link_libraries(steps_adjacency_list_test PUBLIC gunit steps_adjacency_list rapidcheck rapidcheck_gtest)
add_test(NAME steps_adjacency_list_test COMMAND steps_adjacency_list_test)

# Add relaxed_adjacency_list test executable
add_executable(relaxed_adjacency_list_test src/solver/relaxed_adjacency_list_test.cpp)
target_link_libraries(relaxed_adjacency_list_test PUBLIC gunit relaxed_adjacency_list cached_test_data)
add_test(NAME relaxed_adjacency_list_test COMMAND relaxed_adjacency_list_test)

# Add concorde test executable
add_executable(concorde_test src/solver/concorde_test.cpp)
target_link_libraries(concorde_test PUBLIC gunit concorde rapidcheck rapidcheck_gtest)
add_test(NAME concorde_test COMMAND concorde_test)
set_tests_properties(concorde_test PROPERTIES LABELS "slow")

# Add relaxed_shortest_path test executable
add_executable(relaxed_shortest_path_test src/solver/relaxed_shortest_path_test.cpp)
target_link_libraries(relaxed_shortest_path_test PUBLIC gunit relaxed_shortest_path)
add_test(NAME relaxed_shortest_path_test COMMAND relaxed_shortest_path_test)

# Add steps_shortest_path test executable
add_executable(steps_shortest_path_test src/solver/steps_shortest_path_test.cpp)
target_link_libraries(steps_shortest_path_test PUBLIC gunit steps_shortest_path cached_test_data rapidcheck rapidcheck_gtest)
add_test(NAME steps_shortest_path_test COMMAND steps_shortest_path_test)

# Add date test executable
add_executable(date_test src/util/date_test.cpp)
target_link_libraries(date_test PUBLIC gunit date)
add_test(NAME date_test COMMAND date_test)

# Add branch_and_bound test executable
add_executable(branch_and_bound_test src/solver/branch_and_bound_test.cpp)
target_link_libraries(branch_and_bound_test PUBLIC gunit branch_and_bound problem_state_gen naive_solve rapidcheck_gtest)
add_test(NAME branch_and_bound_test COMMAND branch_and_bound_test)
set_tests_properties(branch_and_bound_test PROPERTIES LABELS "slow")

# Add tarel_graph test executable
add_executable(tarel_graph_test src/solver/tarel_graph_test.cpp)
target_link_libraries(tarel_graph_test PUBLIC gunit tarel_graph problem_state_gen naive_solve rapidcheck_gtest)
add_test(NAME tarel_graph_test COMMAND tarel_graph_test)
set_tests_properties(tarel_graph_test PROPERTIES LABELS "slow")

# Add graph_util test executable
add_executable(graph_util_test src/solver/graph_util_test.cpp)
target_link_libraries(graph_util_test PUBLIC gunit graph_util steps_shortest_path)
add_test(NAME graph_util_test COMMAND graph_util_test)
