#! /bin/bash

gnome-terminal \
--window -e 'bash -c " sleep 2; ./ssh_uav1/ssh_uav1_1.sh; exec bash"' \
--tab -e 'bash -c " sleep 7; ./ssh_uav1/ssh_uav1_2.sh; exec bash"' \
--tab -e 'bash -c " sleep 7; ./ssh_uav1/ssh_uav1_3.sh; exec bash"' \
--tab -e 'bash -c " sleep 15; ./ssh_uav1/ssh_uav1_4.sh; exec bash"' \
--tab -e 'bash -c " sleep 15; ./ssh_uav1/ssh_uav1_5.sh; exec bash"' \
--tab -e 'bash -c " sleep 15; ./ssh_uav1/ssh_uav1_7.sh; exec bash"' \
--tab -e 'bash -c " sleep 15; ./ssh_uav1/ssh_uav1_8.sh; exec bash"' \
#--tab -e 'bash -c " sleep 20; ./ssh_uav1/ssh_uav1.sh; exec bash"' \
#--tab -e 'bash -c " sleep 23; ./ssh_uav1/ssh_uav1.sh; exec bash"' \
#--tab -e 'bash -c " sleep 25; ./ssh_uav1/ssh_uav1.sh; exec bash"' \
#--tab -e 'bash -c " sleep 28; ./ssh_uav1/ssh_uav1.sh; exec bash"' \
#--tab -e 'bash -c " sleep 20; ./ssh_uav1/ssh_uav1_3.sh; exec bash"' \
#--tab -e 'bash -c " sleep 30; ./ssh_uav1/ssh_uav1_4.sh; exec bash"' \
#--tab -e 'bash -c " sleep 40; ./ssh_uav1/ssh_uav1_5.sh; exec bash"' \
#--tab -e 'bash -c " ./ssh_uav2/ssh_uav2_6.sh; exec bash"' \
#--tab -e 'bash -c " ./ssh_nuc.sh; exec bash"' \
#--tab -e 'bash -c " ./ssh_nuc.sh; exec bash"' \

