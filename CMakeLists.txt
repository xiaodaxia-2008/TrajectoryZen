cmake_minimum_required(VERSION 3.20)

project(TrajectoryZen LANGUAGES CXX
    VERSION 0.0.1
    DESCRIPTION "A variety of trajectory planning algorithms")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS FALSE)

find_package(spdlog CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

add_subdirectory(Src)