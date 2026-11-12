cmake_minimum_required(VERSION 3.22)
project(staubli_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf test
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Test SRDF files
  ament_add_pytest_test(test_default_srdf_xacro_parsing test/test_default_srdf.py)
  add_launch_test(test/test_moveit_mock_planning.launch.py TIMEOUT 180)
endif()

ament_package()
