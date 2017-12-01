cmake_minimum_required(VERSION 2.8.3)
project(translator_vision)

find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    rospy
    gazebo_ros
    baxter_core_msgs
)

# Depend on system install of Gazebo
# find_package(GAZEBO REQUIRED)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
)

catkin_python_setup()


# link_directories(
#   ${GAZEBO_LIBRARY_DIRS}
#   ${catkin_LIBRARY_DIRS}
# )

# include_directories(include
#   ${Boost_INCLUDE_DIR}
#   ${catkin_INCLUDE_DIRS}
#   ${GAZEBO_INCLUDE_DIRS}
# )

# add_executable(myProgram src/main.cpp src/some_file.cpp src/another_file.cpp)
# This will build a target executable called myProgram which is built from 3 source files:
# src/main.cpp, src/some_file.cpp and src/another_file.cpp.

# add_library(baxter_gazebo_ros_control
#   src/baxter_gazebo_ros_control_plugin.cpp
# )

# target_link_libraries(baxter_gazebo_ros_control
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   gazebo_ros_control
# )

# install
