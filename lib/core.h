//
// Created by uav on 2020/10/30.
//

#ifndef GAOFEN2020_CORE_H
#define GAOFEN2020_CORE_H

#include<gaofen2020/ros_related.h>
#include<gaofen2020/common_usage.h>

/**
 * topic variable
 */
geometry_msgs::PoseStamped dronePoseLp;
nav_msgs::Odometry dronePoseT265;
geometry_msgs::PoseStamped droneDownCameraPose;

/**
 * program process
 */
int stateStep = 0;

/**
 * droneState
 */
double initYaw = 0;
double suminitYaw = 0;

Eigen::Vector2d droneHomeSum = Eigen::Vector2d::Zero();
Eigen::Vector2d droneHome = Eigen::Vector2d::Zero();
Eigen::Vector3d droneEuler = Eigen::Vector3d::Zero();
int prepareFunCount = 100;

mavros_msgs::State currentStateMsg;
double planeCurrHeight;
double homeHoverHeight = 1.0;


geometry_msgs::PoseStamped px4PointMsg;


/**
 *  prepare.cpp
 */
bool get_yaw_fun();
bool take_off_fun();
bool hover_and_adjust_func();
Eigen::Vector2d hoverDriftSum = Eigen::Vector2d::Zero();
Eigen::Vector2d hover2homeDrift = Eigen::Vector2d::Zero();
Eigen::Vector2d drift = Eigen::Vector2d::Zero();
int hoverFunCount = 100;


/**
 *  common.cpp
 */
Eigen::Vector3d quaternion2euler_eigen(float x, float y, float z, float w);
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);
Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw);
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);


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
ros::Publisher pubCurrentPose;
void ros_callback_func();

/**
 * jump_hoops.cpp
 */
bool isArrived();
int loopStep = 0;
double frontPoints[6][10] = {{0}};

double centerPoints[6][10] ={{0}};

bool go_to_loop(int numberLoop);
void setPva(int numberLoop);
trajectory_msgs::JointTrajectoryPoint pvaTargetPointMsg;


#endif //GAOFEN2020_CORE_H
