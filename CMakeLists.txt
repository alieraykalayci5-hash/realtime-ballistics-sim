cmake_minimum_required(VERSION 3.20)
project(realtime_ballistics_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ballistics
    src/main.cpp
    src/sim/ballistics.cpp
)

target_include_directories(ballistics PRIVATE src)

if (MSVC)
    target_compile_options(ballistics PRIVATE /W4 /permissive-)
else()
    target_compile_options(ballistics PRIVATE -Wall -Wextra -Wpedantic)
endif()