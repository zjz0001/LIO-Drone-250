#! /bin/bash
#  This is a script which is used to run the UAV-controller.

gnome-terminal \
 --window -e  'bash -c " sleep 1s;roslaunch geometric_controller takeoff_group.launch; exec bash"' \

