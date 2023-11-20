/**
 * @file vision_pose.cpp
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <time.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class vision_pose
{
public:
    vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);

    double pi;
    struct attitude
    {
        double pitch;
        double roll;
        double yaw;
    };
    enum ESTIMATED_POSE_SOURCE
    {
        REALSENSE_T265,
        OPEN_VINS
    } pose_source_;

    attitude vrpnAttitude;
    attitude estimatedAttitude;
    attitude px4Attitude;

    geometry_msgs::PoseStamped vrpnPose;
    geometry_msgs::PoseStamped px4Pose;
    geometry_msgs::PoseStamped estimatedPose;

    bool vrpnPoseRec_flag;
    bool realsenseBridgePoseRec_flag;
    bool ovPoseRec_flag;
    bool vrpnPose_initilized;
    bool only_vrpn;
    bool estimatedOdomRec_flag;

    ros::Rate *rate;
    ros::Time start_time;
    double setupTime;
    double errx_const, erry_const, errz_const;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    std::string uavName;
    //估计位姿来源
    std::string pose_estimate_source_;

    ros::Subscriber vrpn_pose_sub;
    ros::Subscriber px4Pose_sub;
    ros::Subscriber openvins_pose_sub;
    ros::Publisher vision_pose_pub;
    ros::Subscriber odom_sub;

    void vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void openvins_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void estimator_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void start();

};
/* 构造函数 */
vision_pose::vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_)
:nh(nh_), nh_private(nh_private_),pose_source_(OPEN_VINS),vrpnPose_initilized(false),pose_estimate_source_("open_vins")
{

    nh_private.param<std::string>("uavname", uavName, "px4_uav1");
    nh_private.param<std::string>("pose_estimate_source",pose_estimate_source_,"open_vins");
    nh_private.param<bool>("only_vrpn", only_vrpn,"false");

    if (pose_estimate_source_=="t265") pose_source_=REALSENSE_T265;

    pi = 3.1415926;
    setupTime = 10;
    rate = new ros::Rate(30);


   // vrpn_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/" + uavName + "/pose", 1, &vision_pose::vrpn_pose_cb,this);
    // openvins_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/ov_msckf/poseimu", 1, &vision_pose::openvins_pose_cb,this);
    px4Pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10,&vision_pose::px4Pose_cb,this);
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
    // odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 2, &vision_pose::estimator_odom_cb,this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 2, &vision_pose::estimator_odom_cb,this);
    realsenseBridgePoseRec_flag = false;
    vrpnPoseRec_flag = false;
    ovPoseRec_flag = false;
    only_vrpn = true;
    estimatedOdomRec_flag = false;

    estimatedAttitude.pitch = 0;
    estimatedAttitude.roll = 0;
    estimatedAttitude.yaw = 0;

    vrpnAttitude.pitch = 0;
    vrpnAttitude.roll = 0;
    vrpnAttitude.yaw = 0;

    px4Attitude.pitch = 0;
    px4Attitude.roll = 0;
    px4Attitude.yaw = 0;   
}

void vision_pose::estimator_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    estimatedPose.pose = msg->pose.pose;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    estimatedAttitude.pitch = pitch * 180 / pi;
    estimatedAttitude.roll = roll * 180 / pi;
    estimatedAttitude.yaw = yaw * 180 / pi;

    estimatedOdomRec_flag = true;
}


void vision_pose::px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    px4Pose.pose = msg->pose;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    px4Attitude.pitch = pitch * 180 / pi;
    px4Attitude.roll = roll * 180 / pi;
    px4Attitude.yaw = yaw * 180 / pi;
}

/*通过vrpn接受bebop位置消息*/
void vision_pose::vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    /*Motive通过 VRPN 发布的位置消息 单位是 米
    */
    vrpnPose.pose = msg->pose;
    estimatedPose = *msg;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    vrpnAttitude.pitch = pitch * 180 / pi;
    vrpnAttitude.roll = roll * 180 / pi;
    vrpnAttitude.yaw = yaw * 180 / pi;

    vrpnPoseRec_flag = true;
}


// void vision_pose::openvins_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
// {
//     estimatedPose.pose.position.x = msg->pose.pose.position.y;
//     estimatedPose.pose.position.y = -msg->pose.pose.position.x;
//     estimatedPose.pose.position.z = msg->pose.pose.position.z;
//     tf2::Quaternion q_ov, q_rot_r, q_rot_y, q_est, q_gamma;
//     tf2::fromMsg(msg->pose.pose.orientation,q_ov);
//     // double r = 1.5707963, p = 0, y = 0; //Rotate the previous pose by 90degree about Z
//     q_rot_r.setRPY(1.5707963,0,0);
//     q_rot_y.setRPY(0,0,1.5707963);
//     q_gamma.setRPY(0,0,-1.5707963);
//     q_est = q_gamma * q_ov * q_rot_r * q_rot_y;
//     q_est.normalize();
//     tf2::convert(q_est,estimatedPose.pose.orientation);   
//     double roll_ov, pitch_ov, yaw_ov;
//     tf2::Matrix3x3(q_est).getRPY(roll_ov, pitch_ov, yaw_ov);
//     estimatedAttitude.pitch = pitch_ov * 180 / pi;
//     estimatedAttitude.roll = roll_ov * 180 / pi;
//     estimatedAttitude.yaw = yaw_ov * 180 /pi;
//     //cout<<"pitch: "<<pitch_ov*180/pi<<" roll: "<<roll_ov*180/pi<<" yaw: "<<yaw_ov*180/pi<<endl;
//     ovPoseRec_flag = true;
// }

void vision_pose::start()
{
    double errx, erry, errz;
    while(ros::ok())
    {
     	
        if(estimatedOdomRec_flag == false){
            cout << "\033[K" << "\033[31m estimatedPose no receive!!! \033[0m" << endl;
	    }else {
            estimatedPose.header.stamp = ros::Time::now();
		    vision_pose_pub.publish(estimatedPose);
            cout << "\033[K"  << "\033[32m vrpn ok !\033[0m" << endl;
            cout << "\033[K"  << "       estimatedPose                  vrpnPose               px4Pose" << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "x      " << estimatedPose.pose.position.x << "\t\t" << vrpnPose.pose.position.x << "\t\t" << px4Pose.pose.position.x << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "y      " << estimatedPose.pose.position.y << "\t\t" << vrpnPose.pose.position.y << "\t\t" << px4Pose.pose.position.y << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "z      " << estimatedPose.pose.position.z << "\t\t" << vrpnPose.pose.position.z << "\t\t" << px4Pose.pose.position.z << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "pitch  " << estimatedAttitude.pitch << "\t\t" << vrpnAttitude.pitch << "\t\t" << px4Attitude.pitch << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "roll   " << estimatedAttitude.roll << "\t\t" << vrpnAttitude.roll << "\t\t" << px4Attitude.roll << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "yaw    " << estimatedAttitude.yaw << "\t\t" << vrpnAttitude.yaw << "\t\t" << px4Attitude.yaw << endl;
            cout << "\033[9A" << endl;
        }
        ros::spinOnce();
        rate->sleep();
    }
    // cout << "\033[2J" << endl;
    cout << "\033[9B" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_pose_node");
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");
    vision_pose vision(nh_,nh_private_);
    vision.start();

    return 0;
}
