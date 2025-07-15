cmake_minimum_required(VERSION 3.20)

project(hello_world 
    VERSION 1.0.0
    DESCRIPTION "A simple hello world application"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

add_executable(hello_world main.cpp)

target_compile_options(hello_world PRIVATE -Wall -Wextra -Wpedantic)
