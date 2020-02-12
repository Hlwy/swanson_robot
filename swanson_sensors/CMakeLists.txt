cmake_minimum_required(VERSION 2.8.3)
project(swanson_sensors)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpmachine OUTPUT_VARIABLE MACHINE)
message(STATUS "MACHINE = ${MACHINE}")
if(${MACHINE} MATCHES "arm-linux-gnueabihf")
     set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -march=armv8-a -mfpu=neon-fp-armv8 -mtune=cortex-a72 -mfloat-abi=hard -ftree-vectorize -latomic -marm")
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -mfpu=neon-fp-armv8 -mtune=cortex-a72 -mfloat-abi=hard -ftree-vectorize -latomic -marm")
endif(${MACHINE} MATCHES "arm-linux-gnueabihf")

set( ROS_CXX_DEPENDENCIES
     roscpp
     rospy
     tf
     cv_bridge
     image_transport
     std_msgs
     geometry_msgs
     sensor_msgs
     message_generation
)
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(Boost 1.55.0 REQUIRED COMPONENTS system thread)
find_package(RoboCommander REQUIRED COMPONENTS tca9548a bno055_i2c rtimu img_utils cv_utils camera_d415)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${RoboCommander_INCLUDE_DIR}
     ${catkin_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
     ${realsense2_INCLUDE_DIR}
)
link_directories(
     ${Boost_LIBRARIES}
     ${RoboCommander_LIBRARIES}
     ${OpenCV_LIBS}
     ${realsense2_LIBRARY}
)

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES swanson_sensors
     CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
     DEPENDS Boost RoboCommander OpenCV
)
add_dependencies(${catkin_EXPORTED_TARGETS})

###########
## Build ##
###########

add_library(rtimuRos src/rtimu_ros.cpp)
target_link_libraries(rtimuRos ${catkin_LIBRARIES} rtimu)
add_dependencies(rtimuRos ${catkin_EXPORTED_TARGETS})

add_executable(rtimu_ros_node src/rtimu_ros_node.cpp)
target_link_libraries(rtimu_ros_node rtimuRos)

add_library(bno055_i2c_ros src/bno055_i2c_ros.cpp)
target_link_libraries(bno055_i2c_ros ${catkin_LIBRARIES} bno055_i2c tca9548a)
add_dependencies(bno055_i2c_ros ${catkin_EXPORTED_TARGETS})

add_executable(bno055_i2c_node src/bno055_i2c_ros_node.cpp)
target_link_libraries(bno055_i2c_node bno055_i2c_ros)

add_executable(bno055_multiplexed_node src/bno055_multiplexed_node.cpp)
target_link_libraries(bno055_multiplexed_node bno055_i2c_ros)

add_library(cameraD415Ros src/camera_d415_ros.cpp)
target_link_libraries(cameraD415Ros cv_utils img_utils camera_d415 ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(cameraD415Ros ${catkin_EXPORTED_TARGETS})

add_executable(camera_d415_node src/camera_d415_ros_node.cpp)
target_link_libraries(camera_d415_node cameraD415Ros cv_utils camera_d415 ${OpenCV_LIBS} ${catkin_LIBRARIES})

#############
## Install ##
#############
install(TARGETS
          rtimuRos
          rtimu_ros_node
          bno055_i2c_ros
          bno055_i2c_node
          bno055_multiplexed_node
          cameraD415Ros
          camera_d415_node
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
