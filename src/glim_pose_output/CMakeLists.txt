cmake_minimum_required(VERSION 3.16)
project(glim_pose_output LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(glim 1.2.2 EXACT CONFIG REQUIRED)

# Do not link glim_ros: its exported libraries include rviz_viewer.
add_library(${PROJECT_NAME} SHARED src/pose_output.cpp src/extension.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC glim::glim)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp geometry_msgs)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_output test/test_pose_output.cpp TIMEOUT 30
    ENV "GLIM_POSE_OUTPUT_TEST_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/../../configs/glim/mode_a_cpu_mapping_pose_output")
  target_link_libraries(test_pose_output ${PROJECT_NAME})
endif()

ament_package()
