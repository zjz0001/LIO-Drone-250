#!/usr/bin/expect
set timeout 30
set host "192.168.50.160"
set username "nvidia"
set password "nvidia"
set user nvidia
spawn ssh $username@$host
expect "*password*" {send "$password\r"}		
expect $user@* {send "source /home/nvidia/work_zjz/livox/livox_mid360_driver_ws/devel/setup.bash\r"}	
expect $user@* {send "roslaunch livox_ros_driver2 msg_MID360.launch\r"}

interact
