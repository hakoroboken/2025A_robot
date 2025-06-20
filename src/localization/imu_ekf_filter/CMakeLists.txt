cmake_minimum_required(VERSION 3.5)
project(imu_ekf_filter)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(target_imu_ekf_filter SHARED
  DIRECTORY src
)
target_include_directories(target_imu_ekf_filter
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}
)

rclcpp_components_register_node(target_imu_ekf_filter
  PLUGIN "imu_ekf_filter::ImuEkfFilter"
  EXECUTABLE imu_ekf_filter
)

ament_auto_package(
  INSTALL_TO_SHARE
)