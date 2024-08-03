file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS "*.cpp" "*.h")
add_library(TrajectoryZen ${SRC_FILES})

target_link_libraries(TrajectoryZen PUBLIC spdlog::spdlog Eigen3::Eigen)