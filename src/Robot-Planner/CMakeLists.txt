cmake_minimum_required(VERSION 3.5)

# ------------------------------------------------------------------
# Dual-build: ROS1 (catkin) or ROS2 (ament_cmake)
# Detected via ROS_VERSION env var (set by setup.bash)
# ------------------------------------------------------------------

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
  # ===================== ROS2 build =====================
  project(Robot-Planner)

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(gazebo_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(rclcpp REQUIRED)

  # Install Python scripts
  install(PROGRAMS
    scripts/planner.py
    scripts/controller.py
    scripts/gazebo_to_tf.py
    scripts/map_publisher.py
    scripts/utils.py
    scripts/ros_compat.py
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install launch files
  install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  # Install maps
  install(DIRECTORY
    maps/
    DESTINATION share/${PROJECT_NAME}/maps
  )

  ament_package()

else()
  # ===================== ROS1 build (pure catkin, no ament_cmake) =====================
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
  )

  project(Robot-Planner)

  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES turtle
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  #  DEPENDS system_lib
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  catkin_install_python(PROGRAMS
    scripts/planner.py scripts/controller.py scripts/gazebo_to_tf.py scripts/map_publisher.py scripts/utils.py scripts/ros_compat.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  # Install launch and maps for colcon discovery
  install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
  install(DIRECTORY
    maps/
    DESTINATION share/${PROJECT_NAME}/maps
  )

endif()
