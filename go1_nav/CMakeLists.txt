cmake_minimum_required(VERSION 3.5)
project(go1_nav)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(unitree_legged_real REQUIRED)


include_directories(
  include
)
include_directories(
  include
  ../unitree_ros2_to_real/unitree_legged_sdk/include
)

add_executable(imu_test src/imu_test.cpp)
target_link_libraries(imu_test ${EXTRA_LIBS})
ament_target_dependencies(imu_test
  rclcpp
  ros2_unitree_legged_msgs
  unitree_legged_real
  sensor_msgs
  tf2
  tf2_ros
)


add_executable(unitree_odom_node src/unitree_odom_node.cpp)
target_link_libraries(unitree_odom_node ${EXTRA_LIBS})
ament_target_dependencies(unitree_odom_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  unitree_legged_real
  ros2_unitree_legged_msgs
)

add_executable(unitree_highcmd_keepalive src/unitree_highcmd_keepalive.cpp)

ament_target_dependencies(unitree_highcmd_keepalive
  rclcpp
  ros2_unitree_legged_msgs
)

add_executable(cmd_vel_to_highcmd src/cmd_vel_to_highcmd.cpp)

ament_target_dependencies(cmd_vel_to_highcmd
  rclcpp
  geometry_msgs
  ros2_unitree_legged_msgs
)

install(TARGETS
  cmd_vel_to_highcmd
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS
  unitree_highcmd_keepalive
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS

  imu_test
  unitree_odom_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
