add_library(TrajectoryZen)
generate_export_header(TrajectoryZen)
target_link_libraries(TrajectoryZen PUBLIC spdlog::spdlog Eigen3::Eigen)
target_include_directories(TrajectoryZen PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>)

add_subdirectory(Base)
add_subdirectory(SplinePath)
add_subdirectory(TrajectoryToppra)