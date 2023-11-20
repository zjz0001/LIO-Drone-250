#!/usr/bin/expect
set timeout 30
set host "192.168.50.160"
set username "nvidia"
set password "nvidia"
set user nvidia
spawn ssh $username@$host
expect "*password*" {send "$password\r"}	
expect $user@* {send "source /home/nvidia/Fast-Drone-250/devel/setup.bash\r"}	
expect $user@* {send "roslaunch ego_planner single_run_in_exp_type1.launch\r"}

interact
