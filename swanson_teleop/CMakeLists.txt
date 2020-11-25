cmake_minimum_required(VERSION 2.8.3)
project(swanson_teleop)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set( ROS_CXX_DEPENDENCIES
     roscpp
     rospy
     std_msgs
     geometry_msgs
     message_generation
)
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(robocommander REQUIRED COMPONENTS android_app_interface)

message(STATUS "robocommander: Found = ${robocommander_FOUND}")
message(STATUS "     Root Directory --- ${robocommander_ROOT}")
message(STATUS "     Includes Directory --- ${robocommander_INCLUDE_DIR}")
message(STATUS "     CMake Directory --- ${robocommander_INSTALLED_CMAKE_DIR}")
message(STATUS "     Library Objects:")
foreach(obj ${robocommander_LIBRARIES})
     message(STATUS "          ${obj}")
endforeach()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#      DEPENDENCIES
#      std_msgs
#      geometry_msgs
#      swanson_teleop
# )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${robocommander_INCLUDE_DIR}
     ${robocommander_INCLUDES}
     ${catkin_INCLUDE_DIRS}
)
link_directories(
     ${Boost_LIBRARIES}
     ${robocommander_LIBRARIES}
)

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES swanson_teleop
     CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
     DEPENDS Boost
)
add_dependencies(${catkin_EXPORTED_TARGETS})

###########
## Build ##
###########

add_library(androidRosInterface src/android_ros_interface_listener.cpp)
target_link_libraries(androidRosInterface ${catkin_LIBRARIES} android_app_interface)
add_dependencies(androidRosInterface ${catkin_EXPORTED_TARGETS} )#${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

add_executable(android_interface_listener_node src/android_interface_listener_node.cpp)
target_link_libraries(android_interface_listener_node androidRosInterface)

#############
## Install ##
#############
install(TARGETS
     androidRosInterface
     android_interface_listener_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
