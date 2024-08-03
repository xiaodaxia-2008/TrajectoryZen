find_package(Catch2 CONFIG REQUIRED)
add_executable(Benchmarks BenchmarkLP.cpp)
target_link_libraries(Benchmarks PRIVATE TrajectoryZen Catch2::Catch2WithMain)