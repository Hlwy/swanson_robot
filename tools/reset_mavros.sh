#!/bin/bash

# Get absolute path of this script to get the catkin ws root path for sourcing
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
CATKIN_DEVEL_PATH=$(echo $SCRIPTPATH | sed -e 's;\/[^/]*$;;' | sed -e 's;\/[^/]*$;;' | sed -e 's;\/[^/]*$;;')
CATKIN_SOURCE_PATH="$CATKIN_DEVEL_PATH/devel/setup.bash"

# Check if a ROS namespace has been set to run necessary commands
# for starting ROS nodes w/ appropriate namespacing
MAVROS_NS=""
ROS_NS=""
if [ -z ${ROS_NAMESPACE+x} ]; then
     MAVROS_NS="mavros"
     ROS_NS=""
else
     MAVROS_NS="/${ROS_NAMESPACE}/mavros"
     ROS_NS="/${ROS_NAMESPACE}"
fi

# Disarm autopilot so that the autopilot will accept our request for reboot
echo " ---------------------------------------- "
echo "           Disarming Autopilot...         "
echo " ---------------------------------------- "
sleep 1
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsafety -n "${MAVROS_NS}" disarm)

# Send the request for autopilot reboot to the autopilot
echo " ---------------------------------------- "
echo "           Rebooting Autopilot...         "
echo " ---------------------------------------- "
sleep 1

REBOOT_CMD=$(cat <<- END
{broadcast: false, command: 246, confirmation: 0, param1: 1,
  param2: 0.0, param3: 0.0,
  param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}
END
)
rosservice call /$MAVROS_NS/cmd/command "$REBOOT_CMD"

# Re-initialize the autopilot's EKF origin to allow pose estimate publishing
echo " ---------------------------------------- "
echo "      Resetting EKF Home Position...      "
echo " ---------------------------------------- "
sleep 1
(source $CATKIN_SOURCE_PATH; roslaunch mavros initialize_ekf.launch)&
sleep 7

# Reset the origin used by the node responsible for sending visual odometer
# pose estimates to the autopilot via MAVLINK messages
echo " ---------------------------------------- "
echo "  Starting Visual Odometry Republisher... "
echo " ---------------------------------------- "
sleep 1
(source $CATKIN_SOURCE_PATH; rosservice call $ROS_NS/vision_pose_republisher/reset "{}")&
# (source $CATKIN_SOURCE_PATH; rosrun mavros path_publisher.py)&
# (source $CATKIN_SOURCE_PATH; rosrun mavros encoder_pose_republisher.py)&
sleep 5

# Set the autopilot's mode to GUIDED for control via MAVROS
echo " ---------------------------------------- "
echo "   Setting Autopilot Mode to GUIDED...    "
echo " ---------------------------------------- "
sleep 1
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" mode -c GUIDED)&
sleep 5

# Change the publish rates for the various data streams available from the autopilot
echo " ---------------------------------------- "
echo " Setting GCS Data Telemetry Stream Rates... "
echo " ---------------------------------------- "
sleep 1
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" rate --ext-status 1)&
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" rate --rc-channels 1)&
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" rate --position 10)&
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" rate --raw-sensors 20 --extra1 20)&
sleep 5

# Arm the autopilot allowing motor control
echo " ---------------------------------------- "
echo "          Arming the autopilot... "
echo " ---------------------------------------- "
sleep 1
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsafety -n "${MAVROS_NS}" arm)&

# Finally run additional commands for any other ROS nodes that may be needing for reconfiguration
echo " ---------------------------------------- "
echo "  Running Auxillery Reconfiguration and/or Service Commands ... "
echo " ---------------------------------------- "
sleep 1
# (source $CATKIN_SOURCE_PATH; rosrun mavros altitude_fixer.py)&
# (source $CATKIN_SOURCE_PATH; rosrun dynamic_reconfigure dynparam set ${ROS_NS}/vboats correction_angle_offset_deg -1.0 &> /dev/null)&
# (source $CATKIN_SOURCE_PATH; rosrun dynamic_reconfigure dynparam set ${ROS_NS}/vboats time_offset -0.3)&
# # (source $CATKIN_SOURCE_PATH; rosrun dynamic_reconfigure dynparam set /${ROS_NAMESPACE}/vboats_node gnd_line_intercept_offset 10)&
echo " "
echo " ================================= "
echo "    MavROS Reset Setup Complete    "
echo " ================================= "
