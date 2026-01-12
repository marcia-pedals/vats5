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

# Create step_merge library target
add_library(step_merge STATIC src/solver/step_merge.cpp)
target_link_libraries(step_merge PUBLIC data)
target_include_directories(step_merge PUBLIC src)

# Create shortest_path library target
add_library(shortest_path STATIC src/solver/shortest_path.cpp)
target_link_libraries(shortest_path PUBLIC step_merge)
target_include_directories(shortest_path PUBLIC src)

# Create branch_and_bound_solver library target
add_library(branch_and_bound_solver STATIC src/solver/branch_and_bound_solver.cpp)
target_link_libraries(branch_and_bound_solver PUBLIC shortest_path)
target_include_directories(branch_and_bound_solver PUBLIC src)

# Create visualization library target
add_library(visualization STATIC src/visualization/visualization.cpp)
target_link_libraries(visualization PUBLIC shortest_path)
target_include_directories(visualization PUBLIC src)

add_executable(main_server main.cpp)
target_link_libraries(main_server PUBLIC Crow::Crow gtfs)

# Add GTFS filter tool
add_executable(gtfs_filter_tool src/tools/gtfs_filter_tool.cpp)
target_link_libraries(gtfs_filter_tool PUBLIC gtfs date)

# Add visualization tool
add_executable(visualization_tool src/tools/visualization_tool.cpp)
target_link_libraries(visualization_tool PUBLIC visualization)

# Add benchmark reduction tool
add_executable(benchmark_reduction src/tools/benchmark_reduction.cpp)
target_link_libraries(benchmark_reduction PUBLIC shortest_path)

# Add reduction diff tool
add_executable(reduction_diff src/tools/reduction_diff.cpp)
target_link_libraries(reduction_diff PUBLIC shortest_path)

# Add plain_viz tool (minimal paths only, no iteration)
add_executable(plain_viz src/tools/plain_viz.cpp)
target_link_libraries(plain_viz PUBLIC visualization)

# Add branch_and_bound_tool
add_executable(branch_and_bound_tool src/tools/branch_and_bound_tool.cpp)
target_link_libraries(branch_and_bound_tool PUBLIC branch_and_bound_solver)

# Enable CMake testing
enable_testing()

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

# Add shortest_path test executable
add_executable(shortest_path_test src/solver/shortest_path_test.cpp)
target_link_libraries(shortest_path_test PUBLIC gunit shortest_path rapidcheck rapidcheck_gtest)
add_test(NAME shortest_path_test COMMAND shortest_path_test)

# Add date test executable
add_executable(date_test src/util/date_test.cpp)
target_link_libraries(date_test PUBLIC gunit date)
add_test(NAME date_test COMMAND date_test)

# Add branch_and_bound_solver test executable
add_executable(branch_and_bound_solver_test src/solver/branch_and_bound_solver_test.cpp)
target_link_libraries(branch_and_bound_solver_test PUBLIC gunit branch_and_bound_solver)
add_test(NAME branch_and_bound_solver_test COMMAND branch_and_bound_solver_test)
