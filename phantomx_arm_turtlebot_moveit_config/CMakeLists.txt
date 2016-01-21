cmake_minimum_required(VERSION 2.8.3)
project(phantomx_arm_turtlebot_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  roscpp
  rospy
  rosbag
  std_msgs
  control_msgs
  message_generation
##  ros_control         // Check to see why we dont need to add these here and in package.xml!! -- Error finding package. Check if you need to add to catkin_package (__)(see below).
##  ros_controllers     // Check to see why we dont need to add these here and in package.xml!! -- Error finding package. Check if you need to add to catkin_package (__)(see below).
  gazebo_ros
)

## find_package(Curses REQUIRED COMPONENTS system)

find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)

find_package(Boost REQUIRED system filesystem date_time thread)

generate_messages(
  DEPENDENCIES
  std_msgs
  moveit_msgs
  control_msgs
  trajectory_msgs

)

catkin_package(
  ##INCLUDE_DIRS include
  LIBRARIES phantomx_arm/phantomx_arm_turtlebot_moveit_config
  CATKIN_DEPENDS roscpp rospy std_msgs moveit_core moveit_msgs control_msgs moveit_ros_planning moveit_ros_planning_interface gazebo_ros
  DEPENDS system_lib
)

## include_directories(
##  ${catkin_INCLUDE_DIRS}
## )

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS}) 
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
## Check ^^^^^^
## include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(motion_planner src/motion_planner.cpp)
target_link_libraries(motion_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(motion_planner phantomx_arm_turtlebot_moveit_config_generate_messages_cpp)

add_executable(kinematicSolver src/kinematicSolver.cpp)
target_link_libraries(kinematicSolver ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(kinematicSolver phantomx_arm_turtlebot_moveit_config_generate_messages_cpp)

add_executable(pickANDplace_trial src/PicknPlace/tasks/pickANDplace_trial.cpp src/PicknPlace/nodes/pickANDplace.cpp)
target_link_libraries(pickANDplace_trial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pickANDplace_trial phantomx_arm_turtlebot_moveit_config_generate_messages_cpp)

add_library(hello_world SHARED src/Gazebo_Plugins/hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_phantomx_grasp_hack SHARED src/Gazebo_Plugins/gazebo_ros_phantomx_grasp_hack.cc)
target_link_libraries(gazebo_ros_phantomx_grasp_hack ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
