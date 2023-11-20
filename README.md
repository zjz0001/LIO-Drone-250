# LIO-Drone-250
This repository enables fully autonomous drone flight based on Mid360. It incorporates an odometry reference from Fast-LIO2 and a planning module reference from Ego-Planner.


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 16.04**

For **Ubuntu 18.04 or higher**, the **default** PCL and Eigen is enough for FAST-LIO to work normally.

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Mavros**

  ``` 
    sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod +x install_geographiclib_datasets.sh
    sudo ./install_geographiclib_datasets.sh
 ```
  
### 1.3. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

### 1.4. **Livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

*Remarks:*
- Since the FAST-LIO must support Livox serials LiDAR firstly, so the **livox_ros_driver** must be installed and **sourced** before run any FAST-LIO luanch file.
- How to source? The easiest way is add the line ``` source $Livox_ros_driver_dir$/devel/setup.bash ``` to the end of file ``` ~/.bashrc ```, where ``` $Livox_ros_driver_dir$ ``` is the directory of the livox ros driver workspace (should be the ``` ws_livox ``` directory if you completely followed the livox official document).
 ### 1.5. **Realsense_camera**
  ```
      sudo apt-get install ros-melodic-realsense2-camera
      sudo apt-get install ros-melodic-realsense2-description
```

## 2.Build from source
Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/hku-mars/FAST_LIO.git
    cd FAST_LIO
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver before build (follow 1.4 **livox_ros_driver**)

## 3.Deirctly Run!
### 3.1 Start with the command.
- Running  Odometry
- 
First, start the mavors:

```roslaunch geometric_controller takeoff_px4.launch```

start the lidar (e.g., Livox Mid-360):
```source <your_file_to_livox_ros_driver>/devel/setup.bash```
```roslaunch livox_ros_driver2 msg_MID360.launch```
start Fast-LIO and `camera_pose_node`:
```roslaunch fast_lio mapping_mid360.launch```
```roslaunch geometric_controller takeoff_vrpn.launch```

- Running  Controller
next, run the UAV controller:
`roslaunch geometric_controller takeoff_group.launch`
Then, the drone will hover at the specified position (init_position).

- Running  Ego-Planner
```
    roslaunch realsense2_camera rs_camera.launch
    roslaunch ego_planner single_run_in_exp_type1.launch
    rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{
    header: {
    seq: 8 ,
    stamp: {
    secs: 0.0,
    nsecs: 0.0},
    frame_id: "world" },
    pose: {
    position: {
    x: 3.0,
    y: 0.0,
    z: 0.6 },
    orientation: {
    x: 0.0,
    y: 0.0,
    z: 0.0,
    w: 1.0}
    },
    }"
```
Alternatively, you can run `default.rviz` and specify the `/move_base_simple/goal position` with the **2D Nav Goal**, but we do not recommend doing so because it's hard to know the precise goal position this way.

