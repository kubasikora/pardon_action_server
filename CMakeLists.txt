cmake_minimum_required(VERSION 3.0.2)
project(pardon_action_server)

find_package(catkin 
  REQUIRED COMPONENTS
    actionlib_msgs
    actionlib
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    pal_common_msgs
    tf
)

find_package(Boost 
  REQUIRED COMPONENTS 
    system
)

add_action_files(
  DIRECTORY actions 
  FILES TurnToHuman.action
)

generate_messages(
  DEPENDENCIES 
    actionlib_msgs 
    std_msgs 
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs roscpp std_msgs nav_msgs geometry_msgs sensor_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/main.cxx src/TurnToHumanActionServer.cxx src/util.cxx)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
