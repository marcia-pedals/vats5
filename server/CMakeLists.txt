cmake_minimum_required(VERSION 3.20)
include(FetchContent)

project(vats5
    VERSION 1.0.0
    DESCRIPTION "Visit All The Stops!!!"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Crow)

FetchContent_Declare(
  csv  
  GIT_REPOSITORY https://github.com/vincentlaucsb/csv-parser.git
  GIT_SHALLOW TRUE 
  GIT_TAG 2.2.1
) 
FetchContent_MakeAvailable(csv)

if(TARGET csv)
    target_compile_options(csv PRIVATE -fno-profile-arcs -fno-test-coverage)
endif()

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

# Create GTFS library target
add_library(gtfs STATIC src/gtfs/gtfs.cpp)
target_link_libraries(gtfs PUBLIC csv)
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

add_executable(main_server main.cpp)
target_link_libraries(main_server PUBLIC Crow::Crow gtfs)

# Add GTFS filter tool
add_executable(gtfs_filter_tool src/tools/gtfs_filter_tool.cpp)
target_link_libraries(gtfs_filter_tool PUBLIC gtfs)

# Add test executable
add_executable(gtfs_test src/gtfs/gtfs_test.cpp)
target_link_libraries(gtfs_test PUBLIC gunit gtfs)

# Add data test executable
add_executable(data_test src/solver/data_test.cpp)
target_link_libraries(data_test PUBLIC gunit data)

# Add step_merge test executable
add_executable(step_merge_test src/solver/step_merge_test.cpp)
target_link_libraries(step_merge_test PUBLIC gunit step_merge rapidcheck rapidcheck_gtest)

# Add shortest_path test executable
add_executable(shortest_path_test src/solver/shortest_path_test.cpp)
target_link_libraries(shortest_path_test PUBLIC gunit shortest_path)
