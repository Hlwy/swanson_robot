#!/bin/bash
trap "trap - SIGTERM && kill -- $$" SIGINT SIGTERM EXIT
unset ROS_IP
unset ROS_NAMESPACE
unset ROS_MASTER_URI
wlanIp="$(ip addr show wlan0 | sed -En -e 's/.*inet ([0-9.]+).*/\1/p' | head -1)"
#ROBOT_IP="$(ifconfig | grep -A 1 'wl' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)"
#ROBOT_IP="$(ifconfig $dfltIface | sed -En -e 's/.*inet addr: ([0-9.]+).*/\1/p')"
#rosIp="$(ip addr show $rosIface | awk '$1 == "inet" {gsub(/\/.*$/, "", $2); print $2}')"
# rosIp="$(ip addr show wlan0 | sed -En -e 's/.*inet ([0-9.]+).*/\1/p' | head -1)"
rosIp="$(ip addr show eth0 | sed -En -e 's/.*inet ([0-9.]+).*/\1/p' | head -1)"
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

# (source $HOME/swanson_ws/devel/setup.bash; rosservice call /rtabmap/pause_odom "{}")&
# (source $HOME/swanson_ws/devel/setup.bash; roslaunch mavros apm_custom.launch gcs_url:="udp://$wlanIp:9000@10.0.0.74:6000?ids=1,255,252")&
(source $HOME/swanson_ws/devel/setup.bash; roslaunch mavros apm_custom.launch gcs_url:="udp://192.168.2.3:9000@192.168.2.4:6000?ids=1,255,252")&
sleep 10
# (source $HOME/swanson_ws/devel/setup.bash; roslaunch mavros apm_custom.launch gcs_url:="udp://$rosIp:9000@10.0.0.133:6000?ids=1,255,252")&
# (source $HOME/swanson_ws/devel/setup.bash; roslaunch mavros apm_custom.launch gcs_url:="udp://$rosIp:9000@192.168.2.4:6000?ids=1,255,252")&
# (source $HOME/swanson_ws/devel/setup.bash; roslaunch mavros apm_custom.launch gcs_url:="udp://$rosIp:9000@10.0.0.70:6000?ids=1,255,252")&
(source $HOME/swanson_ws/devel/setup.bash; roslaunch mavros initialize_ekf.launch)&
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros vison_pose_republisher.py)&
sleep 10
# (source $HOME/swanson_ws/devel/setup.bash; rosservice call /mavros/wheel_encoders_data/reset_service "{}")
# (source $HOME/swanson_ws/devel/setup.bash; rosrun mavros encoder_pose_republisher.py)&
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys mode -c GUIDED)& # APM
# (source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys mode -c 15)& # PX4
# (source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsafety arm)&
# (source $HOME/swanson_ws/devel/setup.bash; rosrun mavros altitude_fixer.py)&
sleep 5
# (source $HOME/swanson_ws/devel/setup.bash; rosrun mavros path_publisher.py)&
# (source $HOME/swanson_ws/devel/setup.bash; roslaunch swanson_sensors camera_d4xx.launch)&
# (source $HOME/swanson_ws/devel/setup.bash; rosservice call /rtabmap/reset_odom "{}")&
# sleep 5
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys rate --ext-status 1)&
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys rate --rc-channels 1)&
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys rate --position 10)&
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys rate --raw-sensors 5 --extra1 10)&
# ------
CMD1=$(cat <<- END
logger: 'ros.mavros_extras.vision_pose'
level: 'Debug'
END
)
CMD2=$(cat <<- END
logger: 'ros.mavros.param'
level: 'Warn'
END
)
(source $HOME/swanson_ws/devel/setup.bash; rosservice call /mavros/set_logger_level "$CMD1")&
(source $HOME/swanson_ws/devel/setup.bash; rosservice call /mavros/set_logger_level "$CMD2")&
# (source $HOME/swanson_ws/devel/setup.bash; roslaunch move_base move_base.launch)&
# (source $HOME/swanson_ws/devel/setup.bash; roslaunch swanson_algorithms vboats.launch)&
read asdf
