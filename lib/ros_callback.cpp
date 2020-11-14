//
// Created by uav on 2020/10/30.
//
#include "core.h"

/**
 * ros_callback.cpp
 */
ros::Subscriber subStateVision;
ros::Subscriber subStateT265;
ros::Subscriber subStateHeight;
ros::Subscriber subStatePose;
ros::Subscriber subStateMode;
ros::Subscriber subStateDownCamerePose;

ros::Publisher pubPx4Point;
ros::Publisher pubTargetPoint;
ros::Publisher pubPvaTargetPoint;
ros::Publisher pubDroneCurrentPose;


void ros_callback_func(){

    ros::NodeHandle nh;
    /**
     * topic subscriber
     */
    subStateVision = nh.subscribe<geometry_msgs::PoseStamped>("/drone_pos_vision",1,stateVisionCb);
    subStateT265 = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample",1,stateT265Cb);
    subStateHeight = nh.subscribe<mavros_msgs::Altitude>("/mavros/altitude",1,stateHeightCb);
    subStatePose = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",1,statePoseCb);
    subStateMode = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateModeCb);
    subStateDownCamerePose = nh.subscribe<geometry_msgs::PoseStamped>("/zzw",1,stateDownCamerePoseCb);

    /**
     * topic publisher
     */
    pubTargetPoint = nh.advertise<geometry_msgs::PoseStamped>("/topicTargetPoint",1);
    pubPx4Point = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
    pubPvaTargetPoint = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/zuan_quan_setpoint",1);
    pubDroneCurrentPose = nh.advertise<geometry_msgs::PoseStamped>("/topicDroneCurrentPose",1);
}

/**
 * callback_function
 */

void stateModeCb(const mavros_msgs::State::ConstPtr& msg){
    currentStateMsg = *msg;

}


void stateVisionCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
/*    if(visionPose.pose.orientation.w==-1000)
    {
        visionPose.pose.position.x=0;
        visionPose.pose.position.y=0;
        visionPose.pose.position.z=0;
    }*/
/*    else
    {*/
    visionPose = *msg;
//    ROS_INFO("W:%f",visionPose.pose.orientation.w);
// ROS_INFO("x:%f",visionPose.pose.orientation.x);
    //}
}


void stateT265Cb(const nav_msgs::Odometry::ConstPtr& msg){
    dronePoseT265 = *msg;
    // dronePoseCurrent.pose.position.x= dronePoseT265.pose.pose.position.y;
    // dronePoseCurrent.pose.position.y =- dronePoseT265.pose.pose.position.x;


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
    relativeHeight.z()=-(*msg).local;
    relativeHeight.w()=0;

    realHeight=currentAttitude.inverse() * relativeHeight * currentAttitude;

    //cout<<"height"<<-realHeight.z()<<endl;

   // planeCurrHeight = -realHeight.z();

}



void stateDownCamerePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    droneDownCameraPose = *msg;
}

void statePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    dronePoseLp = *msg;
    planeCurrHeight=dronePoseLp.pose.position.z;
    dronePoseCurrent.pose.position.x=dronePoseLp.pose.position.x;
    dronePoseCurrent.pose.position.y=dronePoseLp.pose.position.y;
    ///TODO: Only fo Simulation!!!!!!!
/*    planeCurrHeight=dronePoseLp.pose.position.z;
    dronePoseCurrent.pose.position.x=dronePoseLp.pose.position.x;
    dronePoseCurrent.pose.position.y=dronePoseLp.pose.position.y;*/
}