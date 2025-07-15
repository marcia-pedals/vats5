cmake_minimum_required(VERSION 3.20)

project(hello_world 
    VERSION 1.0.0
    DESCRIPTION "A simple hello world application"
    LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable debug info for debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

# Create the executable
add_executable(hello_world main.cpp)

# Set output directory
set_target_properties(hello_world PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Enable warnings
if(MSVC)
    target_compile_options(hello_world PRIVATE /W4)
else()
    target_compile_options(hello_world PRIVATE -Wall -Wextra -Wpedantic)
endif() 