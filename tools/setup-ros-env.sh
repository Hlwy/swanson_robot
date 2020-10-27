#!/bin/bash

full_reset(){
     echo "[INFO] Resetting ROS network environment variables to localhost defaults."
     echo "   ROS_NAMESPACE  = None"
     echo "   ROS_IP         = 127.0.0.1"
     echo "   ROS_MASTER_URI = http://localhost:11311"

     unset ROS_NAMESPACE
     export ROS_IP="127.0.0.1"
     export ROS_MASTER_URI=http://localhost:11311
}

warnings(){
     echo "[WARN] Input arguments bad."
}

preset_pi4(){
     local ns='swansonPi4'
     local myip='192.168.2.2'
     local master='http://192.168.2.3:11311'

     echo "[INFO] Setting ROS network environment variables using preset for Raspberry Pi 4."
     echo "   ROS_NAMESPACE  = $ns"
     echo "   ROS_IP         = $myip"
     echo "   ROS_MASTER_URI = $master"

     export ROS_NAMESPACE=$ns
     export ROS_IP=$myip
     export ROS_MASTER_URI=$master
}

preset_tx2(){
     local ns='hytx'
     local myip='192.168.2.3'
     local master='http://192.168.2.3:11311'

     echo "[INFO] Setting ROS network environment variables using preset for Nvidia Jetson TX2."
     echo "   ROS_NAMESPACE  = $ns"
     echo "   ROS_IP         = $myip"
     echo "   ROS_MASTER_URI = $master"

     export ROS_NAMESPACE=$ns
     export ROS_IP=$myip
     export ROS_MASTER_URI=$master
}

preset_pc(){
     local ns='redtop'
     local myip='192.168.2.4'
     local master='http://192.168.2.3:11311'

     echo "[INFO] Setting ROS network environment variables using preset for Host PC."
     echo "   ROS_NAMESPACE  = $ns"
     echo "   ROS_IP         = $myip"
     echo "   ROS_MASTER_URI = $master"

     export ROS_NAMESPACE=$ns
     export ROS_IP=$myip
     export ROS_MASTER_URI=$master
}

query_envs(){
     echo "[INFO] ROS network environment variables currently set"
     echo "   ROS_NAMESPACE  = $ROS_NAMESPACE"
     echo "   ROS_IP         = $ROS_IP"
     echo "   ROS_MASTER_URI = $ROS_MASTER_URI"
}

device_preset(){
     ptype=$1
     if [[ -z "$ptype" ]]; then
          echo "[ERROR] Platform preset given is empty."
          return 2
     elif [[ -n "$ptype" ]]; then
          case "$ptype" in
               pi | pi4 )     preset_pi4
                              ;;
               tx | tx2 )     preset_tx2
                              ;;
               pc | host )    preset_pc
                              ;;
               * )            echo "[ERROR] Platform preset given '$ptype' is invalid."
                              return 3
                              ;;
          esac
     fi
}

display_help() {
    echo "Usage: $0 [option...]" >&2
    echo
    echo "   -n, --namespace NAME       Set the value of the ROS_NAMESPACE environment variable. [Default = this device's hostname]."
    echo "   -N, --no-namespace         Unsets the ROS_NAMESPACE environment variable."
    echo "   -i, --iface DEV            Choose the network interface (i.e. eth0, wlan0, etc.) used to set the ROS_IP environment variable."
    echo "   -I, --no-iface             Resets the ROS_IP environment variable back to localhost."
    echo "   -m, --master IP            Sets the ip address associated with the ROS_MASTER_URI environment variable. [Default = localhost]"
    echo "   -M, --no-master            Resets the ROS_MASTER_URI environment variable back to localhost."
    echo "   -p, --preset TYPE          Sets environment variables using platform specific presets. Supported options - pi4, tx2, pc."
    echo "   -q, --query                Prints out the current ROS network environment variables."
    echo "   -R, --reset                Resets environment variables to device local settings (i.e ROS_MASTER_URI/ROS_IP = localhost) and unsets ROS_NAMESPACE."
    echo
}

flagSkipDefaults=0
rosNs="$(hostname -s)"
rosIface="lo"
rosMasterAddr="localhost"
preset=

while [ "$1" != "" ]; do
     case $1 in
        -n | --namespace )    shift
                              rosNs=$1
                              ;;
        -N | --no-namespace ) shift
                              rosNs=""
                              ;;
        -i | --iface )        shift
						rosIface=$1
                              ;;
        -I | --no-iface )     shift
						rosIface="lo"
                              ;;
        -m | --master )       shift
						rosMasterAddr=$1
                              ;;
        -M | --no-master )    shift
						rosMasterAddr="localhost"
                              ;;
        -p | --preset )       shift
                              device_preset $1
                              flagSkipDefaults=1
                              ;;
        -R | --reset )        full_reset
                              flagSkipDefaults=1
                              ;;
        -q | --query )        query_envs
                              flagSkipDefaults=1
                              ;;
        -h | --help )         display_help
                              flagSkipDefaults=1
                              ;;
        * )                   warnings
                              flagSkipDefaults=1
    esac
    shift
done


if [[ $flagSkipDefaults -ne 1 ]]; then
     #ROBOT_IP="$(ifconfig | grep -A 1 'wl' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)"
     #ROBOT_IP="$(ifconfig $dfltIface | sed -En -e 's/.*inet addr: ([0-9.]+).*/\1/p')"
     rosIp="$(ip addr show $rosIface | awk '$1 == "inet" {gsub(/\/.*$/, "", $2); print $2}')"
     rosMaster="http://$rosMasterAddr:11311"

     echo "[INFO] Setting the following ROS network environment variables:"
     if [[ -z "$rosNs" ]]; then
          echo "   ROS_NAMESPACE  = Unsetting"
          unset ROS_NAMESPACE
     else
          echo "   ROS_NAMESPACE  = $rosNs"
          export ROS_NAMESPACE=$rosNs
     fi

     if [[ -z "$rosIp" ]]; then
          echo "   ROS_IP         = Unsetting"
     elif [[ -n "$rosIp" ]]; then
          echo "   ROS_IP         = $rosIp"
     fi
     export ROS_IP=$rosIp

     if [[ -z "$rosMaster" ]]; then
          echo "   ROS_MASTER_URI = Unsetting"
     elif [[ -n "$rosMaster" ]]; then
          echo "   ROS_MASTER_URI = $rosMaster"
     fi
     export ROS_MASTER_URI=$rosMaster
fi

