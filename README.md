# swanson_robot
ROS meta-package containing all of the code necessary to control, interact, and collect on-board sensor data from my homemade 3D printed robot, named "Swanson".


/hytx/camera/color/camera_info /hytx/camera/color/image_raw /hytx/camera/depth/camera_info /hytx/camera/depth/image_rect_raw /hytx/rtabmap/odom_info /hytx/rtabmap/odom_last_frame /hytx/rtabmap/odom_local_map /hytx/rtabmap/odom_local_scan_map /swansonPi4/cmd_vel /swansonPi4/dead_reckoning /swansonPi4/dualclaw/info /swansonPi4/dualclaw/pose /swansonPi4/imu/data/filtered /swansonPi4/imu/data/raw /swansonPi4/imu/mag /swansonPi4/imu/pose/raw /swansonPi4/odometry/filtered /tf_static /vo /tf

# Network setup

## host PC


127.0.0.1       localhost
127.0.1.1       TheRedtop

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters

#192.168.2.3 hytx
#192.168.2.2 pi4swanson
10.0.0.56       hytx
10.42.0.42      pi4swanson
#73.135.169.83  pi4swanson
#proxy22.rt3.io pi4swanson # via port 32619

route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         10.0.0.1        0.0.0.0         UG    600    0        0 wlan0
10.0.0.0        0.0.0.0         255.255.255.0   U     600    0        0 wlan0
10.42.0.0       10.0.0.56       255.255.255.0   UG    0      0        0 wlan0
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 docker0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0


## TX2
sudo iptables -I FORWARD -o eth0 -s 10.0.0/24 -j ACCEPT
OR
sudo iptables -I FORWARD -o eth0 -s 10.42.0/24 -j ACCEPT

nvidia@hytx:~$ cat /etc/hosts
127.0.0.1 localhost
127.0.1.1 hytx

10.42.0.42	pi4swanson
10.0.0.133	TheRedtop
192.168.0.102	balder

nvidia@hytx:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         10.0.0.1        0.0.0.0         UG    600    0        0 wlan0
10.0.0.0        0.0.0.0         255.255.255.0   U     600    0        0 wlan0
10.42.0.0       0.0.0.0         255.255.255.0   U     100    0        0 eth0
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 l4tbr0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
192.168.55.0    0.0.0.0         255.255.255.0   U     0      0        0 l4tbr0

## Pi4

pi@pi4swanson:~ $ cat /etc/hosts
127.0.0.1	localhost
::1		localhost ip6-localhost ip6-loopback
ff02::1		ip6-allnodes
ff02::2		ip6-allrouters

127.0.1.1	pi4swanson
10.42.0.1	hytx
192.168.0.102	balder
10.0.0.133	TheRedtop

pi@pi4swanson:~ $ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         10.42.0.1       0.0.0.0         UG    100    0        0 eth0
0.0.0.0         10.42.0.1       0.0.0.0         UG    202    0        0 eth0
10.42.0.0       0.0.0.0         255.255.255.0   U     100    0        0 eth0
10.42.0.0       0.0.0.0         255.255.255.0   U     202    0        0 eth0
