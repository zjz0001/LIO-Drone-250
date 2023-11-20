# LIO-Drone-250
This repository enables fully autonomous drone flight based on Mid360. It incorporates an odometry reference from [Fast-LIO2](https://github.com/hku-mars/FAST_LIO) and a planning module reference from [Ego-Planner](https://github.com/ZJU-FAST-Lab/ego-planner-swarm). Also, this contribution is inspired by the [Fast-Drone-250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250) from the FAST-LAB laboratory.

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 16.04**

For **Ubuntu 18.04 or higher**, the **default** PCL and Eigen is enough for FAST-LIO to work normally.

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Mavros**
Follow [mavros_installation](https://docs.px4.io/main/en/ros/mavros_installation.html) to install mavros:

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
 Follow [realsense_ros Installation](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy).
 
 Or, you can directly install the Realsense package under ROS：

  ```
      sudo apt-get install ros-melodic-realsense2-camera
      sudo apt-get install ros-melodic-realsense2-description
```

## 2.Build from source
Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$
    git clone https://github.com/zjz0001/LIO-Drone-250.git
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver before build (follow 1.4 **livox_ros_driver**)

## 3.Deirctly Run
Noted:

**It is necessary to start the Ego_Planner after the controller is started and the drone has risen to the specified height, otherwise it will not be able to take off or hover at the specified height！！！！**

### 3.1 Start With the Command.
**Start  Odometry**
  
First, start the mavors:

```
  roslaunch geometric_controller takeoff_px4.launch
```

Publish the lidar messages (e.g., Livox Mid-360):

```
  source $LIVOX_ROS_DRIVER_DIR$/devel/setup.bash
  roslaunch livox_ros_driver2 msg_MID360.launch
```
run Fast-LIO and `camera_pose_node`:
```
  roslaunch fast_lio mapping_mid360.launch
  roslaunch geometric_controller takeoff_vrpn.launch
```

**Start  Controller**
  
next, run the UAV controller:

```
  roslaunch geometric_controller takeoff_group.launch
```

Then, the drone will hover at the specified position (init_position).

**Start  Ego-Planner**
  
```
    roslaunch realsense2_camera rs_camera.launch
    roslaunch ego_planner single_run_in_exp_type1.launch
```

**Pub Nav Goal**
```
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

### 3.2 Start With the Script.

```
    cd ~/$LIO_DRONE_250_DIR$/shfiles
    sudo chmod 777  *.sh
    ./Odom.sh
    ./Controller.sh
    ./Planner.sh
    ./pub.sh
```
