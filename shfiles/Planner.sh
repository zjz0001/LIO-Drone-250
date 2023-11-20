#! /bin/bash
#  This is a script which is used to run the ego-planner.

gnome-terminal \
 --window -e  'bash -c " sleep 1s;roslaunch realsense2_camera rs_camera.launch; exec bash"' \
 --tab -e 'bash -c " sleep 5s;roslaunch ego_planner single_run_in_exp_type1.launch; exec bash"' \
 --tab -e 'bash -c " sleep 20s;./pub.sh; exec bash"' \

