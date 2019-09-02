cmake_minimum_required(VERSION 2.8.3)
project(swanson_sensors)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set( ROS_CXX_DEPENDENCIES
     roscpp
     rospy
     std_msgs
     geometry_msgs
     sensor_msgs
     message_generation
)
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(Boost 1.55.0 REQUIRED COMPONENTS system thread)
find_package(RoboCommander REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#      DEPENDENCIES
#      std_msgs
#      geometry_msgs
#      swanson_sensors
# )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${RoboCommander_INCLUDE_DIR}
     ${catkin_INCLUDE_DIRS}
)
link_directories(
     ${Boost_LIBRARIES}
     ${RoboCommander_LIBRARIES}
)

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES swanson_sensors
     CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
     DEPENDS
          Boost
          RoboCommander
)
add_dependencies(${catkin_EXPORTED_TARGETS})

###########
## Build ##
###########

add_library(rtimuRos src/rtimu_ros.cpp)
target_link_libraries(rtimuRos ${catkin_LIBRARIES} ${RoboCommander_LIBRARIES})
add_dependencies(rtimuRos ${catkin_EXPORTED_TARGETS}) #${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

add_executable(rtimu_ros_node src/rtimu_ros_node.cpp)
target_link_libraries(rtimu_ros_node rtimuRos)

#############
## Install ##
#############
install(TARGETS
     rtimuRos
     rtimu_ros_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
