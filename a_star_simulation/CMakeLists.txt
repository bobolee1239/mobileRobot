cmake_minimum_required(VERSION 2.8.3)
project(a_star_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES a_star_simulation
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/a_star_simulation.cpp
# )

add_executable(map_cal_sim src/map_calTA.cpp)
target_link_libraries(map_cal_sim ${catkin_LIBRARIES})

add_executable(tf_pub src/tf_pubTA.cpp)
target_link_libraries(tf_pub ${catkin_LIBRARIES})

add_executable(tf_pos_sim src/tf_pos_sim.cpp)
target_link_libraries(tf_pos_sim ${catkin_LIBRARIES})

add_executable(map_astar src/map_astar.cpp)
target_link_libraries(map_astar ${catkin_LIBRARIES})

add_executable(map_astar_lab5_sim src/map_astar_lab5_sim.cpp)
target_link_libraries(map_astar_lab5_sim ${catkin_LIBRARIES})

add_executable(nav_sim src/nav.cpp)
target_link_libraries(nav_sim ${catkin_LIBRARIES})

add_executable(a_star_planning src/a_star_planning.cpp)
target_link_libraries(a_star_planning ${catkin_LIBRARIES})
