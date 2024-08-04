cmake_minimum_required(VERSION 3.20)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(TrajectoryZen_BUILD_EXAMPLE "Build example" OFF)
option(TrajectoryZen_BUILD_TEST "Build test" OFF)

project(TrajectoryZen LANGUAGES CXX
    VERSION 0.0.1
    DESCRIPTION "A variety of trajectory planning algorithms")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS FALSE)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

find_package(spdlog CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# Set the source and execution character sets to UTF-8 for MSVC
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

include(GenerateExportHeader)
include_directories(${PROJECT_SOURCE_DIR}/Src)

add_subdirectory(Src/TrajectoryZen)

if(TrajectoryZen_BUILD_EXAMPLE)
    add_subdirectory(Example)
endif()

if(TrajectoryZen_BUILD_TEST)
    add_subdirectory(Test)
endif()