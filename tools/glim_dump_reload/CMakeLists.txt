cmake_minimum_required(VERSION 3.16)
project(glim_dump_reload_validation LANGUAGES CXX)

# Consume installed GLIM; this directory is not a ROS/colcon package.
find_package(glim 1.2.2 EXACT CONFIG REQUIRED)
add_executable(glim_dump_reload validate_dump.cpp)
target_compile_features(glim_dump_reload PRIVATE cxx_std_17)
target_link_libraries(glim_dump_reload PRIVATE glim::glim)
target_compile_options(glim_dump_reload PRIVATE -Wall -Wextra)
