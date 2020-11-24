cmake_minimum_required(VERSION 2.8.3)
project(swanson_sensors)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpmachine OUTPUT_VARIABLE MACHINE)
message(STATUS "MACHINE = ${MACHINE}")
if(${MACHINE} MATCHES "arm-linux-gnueabihf")
     set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -march=armv8-a -mfpu=neon-fp-armv8 -mtune=cortex-a72 -mfloat-abi=hard -ftree-vectorize -latomic -marm")
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -mfpu=neon-fp-armv8 -mtune=cortex-a72 -mfloat-abi=hard -ftree-vectorize -latomic -marm")
endif(${MACHINE} MATCHES "arm-linux-gnueabihf")

set( ROS_CXX_DEPENDENCIES
     rospy
     roscpp
     std_msgs
     sensor_msgs
     geometry_msgs
     tf
     cv_bridge
     image_transport
     message_generation
)
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
# find_package(OpenCV 4 REQUIRED)
find_package(realsense2 REQUIRED)
# find_package(robocommander REQUIRED COMPONENTS tca9548a bno055_i2c rtimu image_utils camera_d4xx)
find_package(robocommander REQUIRED )
message(STATUS "RoboCommander: Found = ${robocommander_FOUND}")
message(STATUS "     Root Directory --- ${robocommander_ROOT}")
message(STATUS "     Includes Directory --- ${robocommander_INCLUDE_DIR}")
message(STATUS "     CMake Directory --- ${robocommander_INSTALLED_CMAKE_DIR}")
message(STATUS "     Library Objects:")
foreach(obj ${robocommander_LIBRARIES})
     message(STATUS "          ${obj}")
endforeach()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${robocommander_INCLUDE_DIR}
     ${robocommander_INCLUDES}
     ${catkin_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
     ${realsense2_INCLUDE_DIR}
)
link_directories(
     ${Boost_LIBRARIES}
     ${robocommander_LIBRARIES}
     ${OpenCV_LIBS}
     ${realsense2_LIBRARY}
)

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES swanson_sensors
     CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
     DEPENDS Boost
)
add_dependencies(${catkin_EXPORTED_TARGETS})

###########
## Build ##
###########

add_library(rtimuRos src/rtimu_ros.cpp)
target_link_libraries(rtimuRos ${catkin_LIBRARIES} rtimu)
add_dependencies(rtimuRos ${catkin_EXPORTED_TARGETS}) #${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

add_executable(rtimu_ros_node src/rtimu_ros_node.cpp)
target_link_libraries(rtimu_ros_node rtimuRos)

add_library(bno055_i2c_ros src/bno055_i2c_ros.cpp)
target_link_libraries(bno055_i2c_ros ${catkin_LIBRARIES} bno055_i2c tca9548a)
add_dependencies(bno055_i2c_ros ${catkin_EXPORTED_TARGETS}) #${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

add_executable(bno055_i2c_node src/bno055_i2c_ros_node.cpp)
target_link_libraries(bno055_i2c_node bno055_i2c_ros)

add_executable(bno055_multiplexed_node src/bno055_multiplexed_node.cpp)
target_link_libraries(bno055_multiplexed_node bno055_i2c_ros)

add_library(camera_d4xx_ros src/camera_d4xx_ros.cpp)
target_link_libraries(camera_d4xx_ros ${catkin_LIBRARIES} image_utils camera_d4xx) # ${robocommander_LIBRARIES})
add_dependencies(camera_d4xx_ros ${catkin_EXPORTED_TARGETS}) #${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

add_executable(camera_d4xx_node src/camera_d4xx_ros_node.cpp)
target_link_libraries(camera_d4xx_node camera_d4xx_ros)

#############
## Install ##
#############
install(TARGETS
     rtimuRos
     rtimu_ros_node
     bno055_i2c_ros
     bno055_i2c_node
     bno055_multiplexed_node
     # ${robocommander_LIBRARIES}
     camera_d4xx_ros
     camera_d4xx_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
