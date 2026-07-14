cmake_minimum_required(VERSION 3.16)
project(ros2_control_diff_drive LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

# INSTALL
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_diff_drive
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_diff_drive
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(diff_drive_urdf_xacro test/test_urdf_xacro.py)

  # Integration (launch) tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_view_robot_launch.py)
  add_ros_isolated_launch_test(test/test_diffbot_launch.py)
endif()

ament_package()
