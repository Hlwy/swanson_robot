cmake_minimum_required(VERSION 2.8.3)
project(swanson_nodelets)
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
     tf
     pcl_ros
     nodelet
     std_msgs
     cv_bridge
     sensor_msgs
     image_transport
     message_generation
     swanson_msgs
     dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV 4 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL 1.7 REQUIRED)
# add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

# For proper linkage use:
#    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/robocommander
find_package(robocommander REQUIRED)
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
catkin_package(
     INCLUDE_DIRS include
     LIBRARIES swanson_nodelets
     CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES} swanson_algorithms
     DEPENDS Boost
)
include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${robocommander_INCLUDE_DIR}
     ${robocommander_INCLUDES}
     ${catkin_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
     ${realsense2_INCLUDE_DIR}
     ${PCL_INCLUDE_DIRS}
     ${swanson_algorithms_INCLUDE_DIRS}
)
link_directories(
     ${Boost_LIBRARIES}
     ${robocommander_LIBRARIES}
     ${OpenCV_LIBS}
     ${realsense2_LIBRARY}
     ${PCL_LIBRARY_DIRS}
)
add_dependencies(${catkin_EXPORTED_TARGETS})

###########
## Build ##
###########

add_library(SwansonNodelets src/vboats_nodelet.cpp)
target_link_libraries(SwansonNodelets ${catkin_LIBRARIES} ${OpenCV_LIBS} ${robocommander_LIBRARIES})
add_dependencies(SwansonNodelets ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
install(TARGETS
     SwansonNodelets
     # angle_test
     # angle_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
