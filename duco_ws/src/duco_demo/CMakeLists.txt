cmake_minimum_required(VERSION 2.8.3)
project(duco_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

if("$ENV{ROS_DISTRO}" STREQUAL "melodic")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
      duco_driver
      # duco_msg
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for melodic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
      duco_driver
      # duco_msg
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for kinetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
      duco_driver
      # duco_msg
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for  for indigo")
endif()




###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES duco_demo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${catkin_LIBRARY_DIRS})

if("$ENV{ROS_DISTRO}" STREQUAL "melodic")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(gcr5_demo src/gcr5_demo.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(gcr5_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES} robot_interface)
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: build duco_demo node for Melodic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(gcr5_demo src/gcr5_demo.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(gcr5_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES} robot_interface)

    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note:build duco_demo node for kinetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(gcr5_demo src/gcr5_demo.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(gcr5_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES} robot_interface)

    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note:build duco_demo node for indigo")
endif()




