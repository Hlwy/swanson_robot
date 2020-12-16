cmake_minimum_required(VERSION 2.8.3)
project(swanson_algorithms)
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
     std_msgs
     cv_bridge
     sensor_msgs
     swanson_msgs
     image_transport
     message_generation
     dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV 4 REQUIRED)
find_package(PCL 1.7 REQUIRED)
# add_definitions(${PCL_DEFINITIONS})

# For proper linkage use:
#    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/robocommander
find_package(robocommander REQUIRED vboats)
message(STATUS "RoboCommander: Found = ${robocommander_FOUND}")
message(STATUS "     Root Directory --- ${robocommander_ROOT}")
message(STATUS "     Includes Directory --- ${robocommander_INCLUDE_DIR}")
message(STATUS "     CMake Directory --- ${robocommander_INSTALLED_CMAKE_DIR}")
message(STATUS "     Library Objects:")
foreach(obj ${robocommander_LIBRARIES})
     message(STATUS "          ${obj}")
endforeach()

if(OpenCV_CUDA_VERSION)
     message(" ----- OpenCV is CUDA-enabled")
     option(WITH_CUDA  "Build CUDA-accelerated Image Processing Functions" ON)
else()
     message(" ----- OpenCV is not CUDA-enabled")
     option(WITH_CUDA  "Build CUDA-accelerated Image Processing Functions" OFF)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${robocommander_INCLUDE_DIR}
     ${robocommander_INCLUDES}
     ${catkin_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
     ${PCL_INCLUDE_DIRS}
)
link_directories(
     ${Boost_LIBRARIES}
     ${robocommander_LIBRARIES}
     ${OpenCV_LIBS}
     ${OpenCV_LIBRARIES}
     ${PCL_LIBRARY_DIRS}
)

generate_dynamic_reconfigure_options(
     cfg/Vboats.cfg
)

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES swanson_algorithms
     CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
     DEPENDS Boost OpenCV
)
add_dependencies(${catkin_EXPORTED_TARGETS})

###########
## Build ##
###########

add_library(vboats_ros SHARED src/vboats_ros.cpp)
target_link_libraries(vboats_ros PUBLIC vboats ${catkin_LIBRARIES})
add_dependencies(vboats_ros ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg) #${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

add_executable(vboats_node src/vboats_ros_node.cpp)
target_link_libraries(vboats_node PUBLIC vboats_ros)

# add_library(angle_test src/angle_test.cpp)
# target_link_libraries(angle_test ${catkin_LIBRARIES} ${robocommander_LIBRARIES})
# add_dependencies(angle_test ${catkin_EXPORTED_TARGETS})
#
# add_executable(angle_test_node src/angle_test_node.cpp)
# target_link_libraries(angle_test_node angle_test)

#############
## Install ##
#############
install(TARGETS
     vboats_ros
     vboats_node
     # angle_test
     # angle_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
