#! /bin/bash
#  This is a script which is used to run the mid360-UAV-odomtry.

gnome-terminal \
 --window -e  'bash -c " sleep 1s;source /home/nvidia/work_zjz/livox/livox_mid360_driver_ws/devel/setup.bash;roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"' \
 --tab -e 'bash -c " sleep 8s;roslaunch fast_lio mapping_mid360.launch; exec bash"' \
 --tab -e 'bash -c " sleep 10s;roslaunch geometric_controller takeoff_px4.launch; exec bash"' \
 --tab -e 'bash -c " sleep 15s;roslaunch geometric_controller takeoff_vrpn.launch; exec bash"' \

