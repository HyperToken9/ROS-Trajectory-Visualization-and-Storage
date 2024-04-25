cmake_minimum_required(VERSION 3.0.2)
project(amr_trajectory_viz)

add_definitions(-std=c++11)  # For CMake versions < 3.1
# or for CMake 3.1 and later:
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  nav_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS filesystem system)


add_service_files(
  FILES
  SaveTrajectory.srv
)



generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amr_trajectory_viz
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime roslib
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_library(trajectory 
    src/trajectory.cpp 
    src/trajectory_logger.cpp 
    src/trajectory_replayer.cpp
)

target_link_libraries(trajectory ${catkin_LIBRARIES})

# Ensure logger_node.cpp is the correct filename for your logger node's main file.
add_executable(logger_node src/logger.cpp)
target_link_libraries(logger_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} trajectory)

# Ensure replayer_node.cpp is the correct filename for your replayer node's main file.
add_executable(replayer_node src/replayer.cpp)
target_link_libraries(replayer_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} trajectory)