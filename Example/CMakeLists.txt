find_package(Matplot++ CONFIG REQUIRED)

add_executable(ExampleLP ExampleLP.cpp)
target_link_libraries(ExampleLP PUBLIC TrajectoryZen Matplot++::cimg Matplot++::matplot)