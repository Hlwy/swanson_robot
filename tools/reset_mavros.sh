#!/bin/bash
(source $HOME/swanson_ws/devel/setup.bash; rosnode kill /mavros_vision_pose_republisher)
(source $HOME/swanson_ws/devel/setup.bash; rosnode kill /mavros_encoder_pose_republisher)
(source $HOME/swanson_ws/devel/setup.bash; rosnode kill /mavros_path_publisher)
(source $HOME/swanson_ws/devel/setup.bash; rosservice call /mavros/wheel_encoders_data/reset_service "{}")
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsafety disarm)

CMD1=$(cat <<- END
{broadcast: false, command: 246, confirmation: 0, param1: 1,
  param2: 0.0, param3: 0.0,
  param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}
END
)

rosservice call /mavros/cmd/command "$CMD1"
echo " "
echo " ----- Resetting EKF Home Position... "
echo " "
(source $HOME/swanson_ws/devel/setup.bash; roslaunch mavros initialize_ekf.launch)&
sleep 5
echo " "
echo " ----- Starting Visual Odometry Republisher... "
echo " "
# (source $HOME/swanson_ws/devel/setup.bash; rosrun mavros path_publisher.py)&
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros vison_pose_republisher.py)&
# (source $HOME/swanson_ws/devel/setup.bash; rosrun mavros encoder_pose_republisher.py)&
sleep 5
echo " "
echo " ----- Setting Autopilot Mode to GUIDED... "
echo " "
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys mode -c GUIDED)&
sleep 5
echo " "
echo " ----- Setting GCS Data Telemetry Stream Rates... "
echo " "
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys rate --ext-status 1)&
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys rate --rc-channels 1)&
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys rate --position 15)&
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsys rate --raw-sensors 5 --extra1 30)&
# echo " "
# echo " ================================= "
# echo "    MavROS Reset Setup Complete    "
# echo " ================================= "
# echo " "
# echo " ******** Press Enter to arm Autopilot. "
# echo " "
# read asdf
(source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsafety arm)&
# echo " "
# echo " ******** Press Enter again to stop any ROS nodes started by this script. "
# echo " "
# read asdf
# (source $HOME/swanson_ws/devel/setup.bash; rosnode kill /mavros_vision_pose_republisher)
# (source $HOME/swanson_ws/devel/setup.bash; rosnode kill /mavros_encoder_pose_republisher)
# (source $HOME/swanson_ws/devel/setup.bash; rosnode kill /mavros_path_publisher)
# (source $HOME/swanson_ws/devel/setup.bash; rosrun mavros mavsafety disarm)
echo " "
echo " ================================= "
echo "    MavROS Reset Setup Complete    "
echo " ================================= "
echo " "
