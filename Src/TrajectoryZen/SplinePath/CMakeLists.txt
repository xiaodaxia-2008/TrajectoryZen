file(GLOB_RECURSE src_files CONFIGURE_DEPENDS "*.cpp" "*.h")
target_sources(TrajectoryZen PRIVATE ${src_files})