cmake_minimum_required(VERSION 3.0.2)
project(jetbot_slam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  motor_driver_node.py
  person_follower_node.py
  spatial_perception_node.py
  semantic_mapping_node.py
  slam_web_dashboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
