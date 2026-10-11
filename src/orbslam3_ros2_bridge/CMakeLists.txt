cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros2_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(ORB_SLAM3_DIR "/home/user/ORB_SLAM3")

add_executable(
  orbslam3_smoke
  src/orbslam3_smoke.cpp
)

target_include_directories(
  orbslam3_smoke
  PRIVATE

  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus

  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

ament_target_dependencies(
  orbslam3_smoke
  rclcpp
)

target_link_libraries(
  orbslam3_smoke

  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

set_target_properties(
  orbslam3_smoke
  PROPERTIES

  BUILD_RPATH
  "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"

  INSTALL_RPATH
  "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"
)

install(
  TARGETS orbslam3_smoke
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  image_test
  src/image_test.cpp
)

ament_target_dependencies(
  image_test
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  image_test
  ${OpenCV_LIBS}
)

install(
  TARGETS image_test
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  imu_test
  src/imu_test.cpp
)

ament_target_dependencies(
  imu_test
  rclcpp
  sensor_msgs
)

install(
  TARGETS imu_test
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  sync_test
  src/sync_test.cpp
)

ament_target_dependencies(
  sync_test
  rclcpp
  sensor_msgs
)

install(
  TARGETS sync_test
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  mono_inertial_node
  src/mono_inertial_node.cpp
)

target_include_directories(
  mono_inertial_node
  PRIVATE

  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus

  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

ament_target_dependencies(
  mono_inertial_node

  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  mono_inertial_node

  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

set_target_properties(
  mono_inertial_node
  PROPERTIES

  BUILD_RPATH
  "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"

  INSTALL_RPATH
  "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"
)

install(
  TARGETS mono_inertial_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  mono_node
  src/mono_node.cpp
)

target_include_directories(
  mono_node
  PRIVATE

  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus

  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

ament_target_dependencies(
  mono_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  mono_node

  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

set_target_properties(
  mono_node
  PROPERTIES

  BUILD_RPATH
  "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"

  INSTALL_RPATH
  "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"
)

install(
  TARGETS mono_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()