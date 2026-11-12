cmake_minimum_required(VERSION 3.8)
project(staubli_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz scripts test
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wait_for_robot_ready.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_wait_for_robot_ready test/test_wait_for_robot_ready.py)
  add_launch_test(test/test_mock_bringup.launch.py TIMEOUT 120)
endif()

ament_package()
