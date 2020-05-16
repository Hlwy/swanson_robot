cmake_minimum_required(VERSION 2.8.3)
project(swanson_algorithms)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpmachine OUTPUT_VARIABLE MACHINE)
message(STATUS "MACHINE = ${MACHINE}")
if(${MACHINE} MATCHES "arm-linux-gnueabihf")
     set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -march=armv8-a -mfpu=neon-fp-armv8 -mtune=cortex-a72 -mfloat-abi=hard -ftree-vectorize -latomic -marm")
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -mfpu=neon-fp-armv8 -mtune=cortex-a72 -mfloat-abi=hard -ftree-vectorize -latomic -marm")
endif(${MACHINE} MATCHES "arm-linux-gnueabihf")

set( ROS_CXX_DEPENDENCIES
     rospy
     roscpp
     tf
     std_msgs
     cv_bridge
     sensor_msgs
     stereo_msgs
     image_transport
     message_generation
     swanson_msgs
     pcl_ros
     dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(Boost 1.55.0 REQUIRED COMPONENTS system thread)
find_package(OpenCV 4.2 REQUIRED)
# find_package(OpenCV 4.1.1 REQUIRED PATHS /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake)
find_package(realsense2 REQUIRED)

# For proper linkage use:
#    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/RoboCommander
find_package(RoboCommander REQUIRED)

message(STATUS "RoboCommander: Found = ${RoboCommander_FOUND}")
message(STATUS "     Root Directory --- ${RoboCommander_ROOT}")
message(STATUS "     Includes Directory --- ${RoboCommander_INCLUDE_DIR}")
message(STATUS "     CMake Directory --- ${RoboCommander_INSTALLED_CMAKE_DIR}")
message(STATUS "     Library Objects:")
foreach(obj ${RoboCommander_LIBRARIES})
     message(STATUS "          ${obj}")
endforeach()

find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${RoboCommander_INCLUDE_DIR}
     ${catkin_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
     ${realsense2_INCLUDE_DIR}
     ${PCL_INCLUDE_DIRS}
)
link_directories(
     ${Boost_LIBRARIES}
     ${RoboCommander_LIBRARIES}
     ${OpenCV_LIBS}
     ${realsense2_LIBRARY}
     ${PCL_LIBRARIES}
)

generate_dynamic_reconfigure_options(
     cfg/Vboats.cfg
)

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES swanson_algorithms
     CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
     DEPENDS
          Boost
          RoboCommander
)
add_dependencies(${catkin_EXPORTED_TARGETS})

###########
## Build ##
###########

add_library(vboats_ros src/vboats_ros.cpp)
target_link_libraries(vboats_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} ${RoboCommander_LIBRARIES})
add_dependencies(vboats_ros ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(vboats_node src/vboats_ros_node.cpp)
target_link_libraries(vboats_node vboats_ros)

add_executable(test_image_proc src/test_image_proc.cpp)
target_link_libraries(test_image_proc ${catkin_LIBRARIES} ${OpenCV_LIBS} ${RoboCommander_LIBRARIES})
add_dependencies(test_image_proc ${catkin_EXPORTED_TARGETS})
#############
## Install ##
#############
install(TARGETS
     vboats_ros
     vboats_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
