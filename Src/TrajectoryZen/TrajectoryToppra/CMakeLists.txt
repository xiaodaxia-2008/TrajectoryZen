file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS "*.cpp" "*.h")
add_library(TrajectoryZen ${SRC_FILES})

generate_export_header(TrajectoryZen)

target_link_libraries(TrajectoryZen PUBLIC spdlog::spdlog Eigen3::Eigen)
target_include_directories(TrajectoryZen PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
