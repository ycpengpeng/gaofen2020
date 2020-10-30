//
// Created by uav on 2020/10/30.
//
#include "core.h"

/**
 * callback_function
 */
void stateVisionCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void stateT265Cb(const nav_msgs::Odometry::ConstPtr& msg);
void stateHeightCb(const mavros_msgs::Altitude::ConstPtr &msg);
void statePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void stateModeCb(const mavros_msgs::State::ConstPtr& msg);
void stateDownCamerePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);

void ros_callback_func(){
    ros::NodeHandle nh;

    /**
     * topic subscriber and publisher
     */
    subStateVision = nh.subscribe<geometry_msgs::PoseStamped>("/topicStateVision",1,stateVisionCb);
    subStateT265 = nh.subscribe<nav_msgs::Odometry>("/camera/odometry",1,stateT265Cb);
    subStateHeight = nh.subscribe<mavros_msgs::Altitude>("mavros/altitude",1,stateHeightCb);
    subStatePose = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position",1,statePoseCb);
    subStateMode = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateModeCb);
    subStateDownCamerePose = nh.subscribe<geometry_msgs::PoseStamped>("/zzw",1,stateDownCamerePoseCb);

    pubTargetPoint = nh.advertise<geometry_msgs::PoseStamped>("/topicTargetPoint",1);
    pubPx4Point = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
    pubPvaTargetPoint = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/topicPvaTarget",1);
    pubCurrentPose = nh.advertise<geometry_msgs::PoseStamped>("/topicDroneCurrentPose",1);
}

/**
 * callback_function
 */


void stateModeCb(const mavros_msgs::State::ConstPtr& msg){
    currentStateMsg = *msg;
    ROS_ERROR("current_state_msg.armed: ");
    cout << "####################################current_state_msg.armed" << to_string(currentStateMsg.armed) << endl;
}


void stateVisionCb(const geometry_msgs::PoseStamped::ConstPtr& msg){

}


void stateT265Cb(const nav_msgs::Odometry::ConstPtr& msg){
    dronePoseT265 = *msg;
}


void stateHeightCb(const mavros_msgs::Altitude::ConstPtr &msg){
    Eigen::Quaterniond currentAttitude;
    Eigen::Quaterniond relativeHeight;
    Eigen::Quaterniond realHeight;
    currentAttitude.x()=dronePoseLp.pose.orientation.x;
    currentAttitude.y()=dronePoseLp.pose.orientation.y;
    currentAttitude.z()=dronePoseLp.pose.orientation.z;
    currentAttitude.w()=dronePoseLp.pose.orientation.w;

    relativeHeight.x()=-0.05;
    relativeHeight.y()=0.05;
    relativeHeight.z()=-(*msg).local - 0.1;
    relativeHeight.w()=0;

    realHeight=currentAttitude.inverse() * relativeHeight * currentAttitude;

    cout<<"height"<<-realHeight.z()<<endl;
    planeCurrHeight = -realHeight.z();
}


void statePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    dronePoseLp = *msg;
    dronePoseLp.pose.position.x += drift.x();  //发给pp
    dronePoseLp.pose.position.y += drift.y();
}


void stateDownCamerePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    droneDownCameraPose = *msg;
}