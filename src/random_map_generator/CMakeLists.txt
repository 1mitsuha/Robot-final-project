cmake_minimum_required(VERSION 3.5)

# ------------------------------------------------------------------
# Dual-build: ROS1 (catkin) or ROS2 (ament_cmake)
# Detected via ROS_VERSION env var (set by setup.bash)
# ------------------------------------------------------------------

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
  # ===================== ROS2 build =====================
  project(random_map_generator)

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(gazebo_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rclcpp REQUIRED)

  # Install Python scripts
  install(PROGRAMS
    src/spawn_obstacles.py
    src/ros_compat.py
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install launch files
  install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  # Install RViz config
  install(FILES
    final.rviz
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()

else()
  # ===================== ROS1 build =====================
  find_package(catkin REQUIRED COMPONENTS
    gazebo_msgs
    rospy
  )

  project(random_map_generator)

  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES random_map_generator
  #  CATKIN_DEPENDS gazebo_msgs rospy
  #  DEPENDS system_lib
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  catkin_install_python(PROGRAMS
    src/spawn_obstacles.py src/ros_compat.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

endif()
