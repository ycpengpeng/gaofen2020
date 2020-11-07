//
// Created by uav on 2020/11/6.
//
#include <core.h>
/**
 * topic variable
 */
geometry_msgs::PoseStamped dronePoseLp;
nav_msgs::Odometry dronePoseT265;
geometry_msgs::PoseStamped droneDownCameraPose;
geometry_msgs::PoseStamped visionPose;

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
int getYawFuncCount = 100;

mavros_msgs::State currentStateMsg;
double planeCurrHeight;
double homeHoverHeight = 1.0;


geometry_msgs::PoseStamped px4PointMsg;