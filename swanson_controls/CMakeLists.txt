cmake_minimum_required(VERSION 2.8.3)
project(swanson_controls)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set( ROS_CXX_DEPENDENCIES
     roscpp
     rospy
     roslaunch
     std_msgs
     geometry_msgs
     sensor_msgs
     swanson_msgs
     message_generation
     dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(robocommander REQUIRED COMPONENTS camera_gimbal tca9548a)

message(STATUS "robocommander: Found = ${robocommander_FOUND}")
message(STATUS "     Root Directory --- ${robocommander_ROOT}")
message(STATUS "     Includes Directory --- ${robocommander_INCLUDE_DIR}")
message(STATUS "     CMake Directory --- ${robocommander_INSTALLED_CMAKE_DIR}")
message(STATUS "     Library Objects:")
foreach(obj ${robocommander_LIBRARIES})
     message(STATUS "          ${obj}")
endforeach()

# Find pigpiod libraries
list(APPEND CMAKE_MODULE_PATH "${robocommander_ROOT}/external/pigpiod")
find_package(pigpio REQUIRED)

generate_dynamic_reconfigure_options(
     cfg/Gimbal.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES swanson_controls
  CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
  DEPENDS Boost
)
include_directories(
     include
     ${Boost_INCLUDE_DIRS}
     ${robocommander_INCLUDE_DIR}
     ${robocommander_INCLUDES}
     ${pigpio_INCLUDE_DIR}
     ${catkin_INCLUDE_DIRS}
)
link_directories(
     ${Boost_LIBRARIES}
     ${robocommander_LIBRARIES}
     ${pigpiod_if2_LIBRARY}
)
add_dependencies(${catkin_EXPORTED_TARGETS})

# Libraries
add_library(camera_gimbal_controller_ros src/camera_gimbal_controller_ros.cpp)
target_link_libraries(camera_gimbal_controller_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES} camera_gimbal tca9548a)
add_dependencies(camera_gimbal_controller_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# Executables
add_executable(camera_gimbal_controller_node src/camera_gimbal_controller_ros_node.cpp)
target_link_libraries(camera_gimbal_controller_node camera_gimbal_controller_ros)

# add_executable(swanson_node src/swanson_node.cpp)
# target_link_libraries(swanson_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(swanson_node swanson_msgs_gencpp)

install(TARGETS
     camera_gimbal_controller_ros
     camera_gimbal_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
