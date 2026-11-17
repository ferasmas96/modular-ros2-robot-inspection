cmake_minimum_required(VERSION 3.8)
project(husky_a300_sensor_extensions)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/pointcloud2_timing_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_pointcloud2_timing_adapter
    test/test_pointcloud2_timing_adapter.py
  )
endif()

ament_package()
