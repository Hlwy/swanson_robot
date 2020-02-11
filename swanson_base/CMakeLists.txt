cmake_minimum_required(VERSION 2.8.3)
project(swanson_base)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set( ROS_CXX_DEPENDENCIES
     roscpp
     rospy
     roslaunch
     tf
     std_msgs
     geometry_msgs
     sensor_msgs
     swanson_msgs
     message_generation
     dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(Boost 1.55.0 REQUIRED COMPONENTS system thread)
find_package(RoboCommander REQUIRED COMPONENTS dualclaw)

message(STATUS "RoboCommander: Found = ${RoboCommander_FOUND}")
message(STATUS "     Root Directory --- ${RoboCommander_ROOT}")
message(STATUS "     Includes Directory --- ${RoboCommander_INCLUDE_DIR}")
message(STATUS "     CMake Directory --- ${RoboCommander_INSTALLED_CMAKE_DIR}")
message(STATUS "     Library Objects:")
foreach(obj ${RoboCommander_LIBRARIES})
     message(STATUS "          ${obj}")
endforeach()

generate_dynamic_reconfigure_options(
     cfg/Drivetrain.cfg
)

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES swanson_base
     CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
     DEPENDS Boost RoboCommander
)
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
add_dependencies(${catkin_EXPORTED_TARGETS})

# Libraries
add_library(dualclawskidsteerdrivetrain src/dual_roboclaw_skidsteer_drivetrain.cpp)
target_link_libraries(dualclawskidsteerdrivetrain ${catkin_LIBRARIES} dualclaw)
add_dependencies(dualclawskidsteerdrivetrain ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg swanson_msgs_generate_messages_cpp)

# Executables
add_executable(dualclaw_skidsteer_drivetrain_node src/dualclaw_skidsteer_drivetrain_node.cpp)
target_link_libraries(dualclaw_skidsteer_drivetrain_node dualclawskidsteerdrivetrain)

install(TARGETS
     dualclawskidsteerdrivetrain
     dualclaw_skidsteer_drivetrain_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
