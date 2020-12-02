#!/bin/bash

# Get absolute path of this script to get the catkin ws root path for sourcing
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
CATKIN_DEVEL_PATH=$(echo $SCRIPTPATH | sed -e 's;\/[^/]*$;;' | sed -e 's;\/[^/]*$;;' | sed -e 's;\/[^/]*$;;')
CATKIN_SOURCE_PATH="$CATKIN_DEVEL_PATH/devel/setup.bash"

trap "trap - SIGTERM && kill -- $$" SIGINT SIGTERM EXIT
unset ROS_IP
unset ROS_NAMESPACE
unset ROS_MASTER_URI

# Get IP of target device for setting ROS_IP
wlanIp="$(ip addr show wlan0 | sed -En -e 's/.*inet ([0-9.]+).*/\1/p' | head -1)"
# ROBOT_IP="$(ifconfig | grep -A 1 'wl' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)"
# ROBOT_IP="$(ifconfig $dfltIface | sed -En -e 's/.*inet addr: ([0-9.]+).*/\1/p')"
# rosIp="$(ip addr show $rosIface | awk '$1 == "inet" {gsub(/\/.*$/, "", $2); print $2}')"
# rosIp="$(ip addr show wlan0 | sed -En -e 's/.*inet ([0-9.]+).*/\1/p' | head -1)"
rosIp="$(ip addr show eth0 | sed -En -e 's/.*inet ([0-9.]+).*/\1/p' | head -1)"

# Set the ROS_IP variable
if [[ -z "$rosIp" ]]; then
     echo "   ROS_IP         = Unsetting"
elif [[ -n "$rosIp" ]]; then
     echo "   ROS_IP         = $rosIp"
     export ROS_IP=$rosIp
fi

# rosMasterAddr="localhost"
# rosMasterAddr=10.0.0.133
rosMasterAddr=192.168.2.3
rosMaster="http://$rosMasterAddr:11311"
if [[ -z "$rosMaster" ]]; then
     echo "   ROS_MASTER_URI = Unsetting"
elif [[ -n "$rosMaster" ]]; then
     echo "   ROS_MASTER_URI = $rosMaster"
fi
export ROS_MASTER_URI=$rosMaster

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

# Start Mavros node
(source $CATKIN_SOURCE_PATH; roslaunch mavros apm_custom.launch gcs_url:="udp://192.168.2.3:9000@192.168.2.5:6000?ids=1,255,252")&

# Set logger levels to hide annoying printouts from mavros during parameter retrieval
HIDE_MAVROS_PARAM_PRINTOUTS=$(cat <<- END
logger: 'ros.mavros.param'
level: 'Warn'
END
)
(source $CATKIN_SOURCE_PATH; rosservice call $ROS_NS/mavros/set_logger_level "$HIDE_MAVROS_PARAM_PRINTOUTS")&

# Set logger level to show debugging printouts from the visual odometer mavros plugin
SHOW_VO_PLUGIN_PRINTOUTS=$(cat <<- END
logger: 'ros.mavros_extras.vision_pose'
level: 'Debug'
END
)
(source $CATKIN_SOURCE_PATH; rosservice call $ROS_NS/mavros/set_logger_level "$SHOW_VO_PLUGIN_PRINTOUTS")&

# Set autopilot's EKF origin and start visual odometer MAVLINK message publisher
(source $CATKIN_SOURCE_PATH; roslaunch mavros initialize_ekf.launch)&
(source $CATKIN_SOURCE_PATH; rosrun mavros vison_pose_republisher.py)&
sleep 10

# Set Autopilot mode to GUIDED for control via ROS
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" mode -c GUIDED)& # APM
# (source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" mode -c 15)& # PX4
sleep 5

# Start RGB-D Camera ROS node
(source $CATKIN_SOURCE_PATH; roslaunch swanson_sensors camera_d4xx.launch)&

# Run commands for resetting any additional ROS nodes
# (source $CATKIN_SOURCE_PATH; rosservice call $ROS_NS/mavros/wheel_encoders_data/reset_service "{}")
# (source $CATKIN_SOURCE_PATH; rosrun mavros encoder_pose_republisher.py)&
# (source $CATKIN_SOURCE_PATH; rosrun mavros path_publisher.py)&
# (source $CATKIN_SOURCE_PATH; rosservice call $ROS_NS/rtabmap/reset_odom "{}")&
# sleep 5

# Change the publish rates for the various data streams available from the autopilot
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" rate --ext-status 1)&
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" rate --rc-channels 1)&
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" rate --position 20)&
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsys -n "${MAVROS_NS}" rate --raw-sensors 5 --extra1 5)&

# Start any additional ROS nodes
# (source $CATKIN_SOURCE_PATH; roslaunch move_base move_base.launch)&
# (source $CATKIN_SOURCE_PATH; roslaunch swanson_algorithms vboats.launch)&
read asdf
