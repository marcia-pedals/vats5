cmake_minimum_required(VERSION 3.20)
include(FetchContent)

project(hello_world 
    VERSION 1.0.0
    DESCRIPTION "A simple hello world application"
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

# Disable coverage for csv library to avoid linking issues
if(TARGET csv)
    target_compile_options(csv PRIVATE -fno-profile-arcs -fno-test-coverage)
endif()

FetchContent_Declare(
  gunit
  GIT_REPOSITORY https://github.com/cpp-testing/GUnit.git
  GIT_TAG        master
)
FETCHCONTENT_MAKEAVAILABLE(gunit)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

add_executable(hello_world main.cpp src/gtfs/gtfs.cpp)
target_link_libraries(hello_world PUBLIC Crow::Crow csv)

# Add test executable
add_executable(gtfs_test src/gtfs/gtfs_test.cpp src/gtfs/gtfs.cpp)
target_link_libraries(gtfs_test PUBLIC gunit csv)
