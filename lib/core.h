//
// Created by uav on 2020/10/30.
//

#ifndef GAOFEN2020_CORE_H
#define GAOFEN2020_CORE_H

#include<gaofen2020/ros_related.h>
#include<gaofen2020/common_usage.h>


/**
 * blind_flyting,cpp
 */
extern bool blind_fly(int blindPoint);
extern int blindStep;
extern void setBlindPva(int blindPoint);
extern bool isArrivedBlind(int blindPoint);

/**
 *  common.cpp
 */
extern Eigen::Vector3d quaternion2euler_eigen(float x, float y, float z, float w);
extern geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);
extern Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw);
extern geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);

/**
 * topic variable
 */
extern geometry_msgs::PoseStamped dronePoseLp;
extern geometry_msgs::PoseStamped dronePoseCurrent;
extern nav_msgs::Odometry dronePoseT265;
extern geometry_msgs::PoseStamped droneDownCameraPose;
extern geometry_msgs::PoseStamped visionPose;
extern ros::NodeHandle nh;

/**
 * program process
 */
extern int stateStep;

/**
 * droneState
 */
extern double initYaw;
extern double suminitYaw;

extern Eigen::Vector2d droneHomeSum;
extern Eigen::Vector2d droneHome;
extern Eigen::Vector3d droneEuler;
extern int getYawFuncCount;

extern mavros_msgs::State currentStateMsg;
extern double planeCurrHeight;
extern double homeHoverHeight;


extern geometry_msgs::PoseStamped px4PointMsg;

/**
 * jump_hoops.cpp
 */
extern bool isArrivedFront(int numberLoop);
extern bool isArrivedCenter(int numberLoop);
extern void update_drift(int numberLoop);

extern bool go_to_loop(int numberLoop);
extern void setFrontPva(int numberLoop);
extern void setTakeOffPva();
extern void setCenterPva(int numberLoop);

extern int loopStep;
extern double frontLoopDistance;
extern double behindLoopDistance;
extern double height;
extern double velocityX;

extern double frontPoints[6][10];
extern double blindPoints[4][10];
extern double centerPoints[7][10];
extern trajectory_msgs::JointTrajectoryPoint pvaTargetPointMsg;


/**
 * landing_off.cpp
 */
extern int landOffStep;
extern void setLandPva(int landOffStep);
extern bool isArrivedLand(int landPoint);


/**
 *  prepare.cpp
 */

//悬停漂移总量
extern Eigen::Vector2d hoverDriftSum;
//悬停时相对于起飞点的漂移量
extern Eigen::Vector2d hover2homeDrift;
//
extern Eigen::Vector3d drift;
extern int hoverFunCount;
//extern double takeOffHeight;
extern bool get_yaw_fun();
extern bool take_off_func();
extern bool hover_and_adjust_func();

/**
 * ros_callback.cpp
 */
extern ros::Subscriber subStateVision;
extern ros::Subscriber subStateT265;
extern ros::Subscriber subStateHeight;
extern ros::Subscriber subStatePose;
extern ros::Subscriber subStateMode;
extern ros::Subscriber subStateDownCamerePose;

extern ros::Publisher pubPx4Point;
extern ros::Publisher pubTargetPoint;
extern ros::Publisher pubPvaTargetPoint;
extern ros::Publisher pubDroneCurrentPose;
extern void ros_callback_func();

/**
 * callback_function
 */
extern void stateVisionCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
extern void stateT265Cb(const nav_msgs::Odometry::ConstPtr& msg);
extern void stateHeightCb(const mavros_msgs::Altitude::ConstPtr &msg);
extern void statePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
extern void stateModeCb(const mavros_msgs::State::ConstPtr& msg);
extern void stateDownCamerePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);

extern bool land_off(int landOffStep);

#endif //GAOFEN2020_CORE_H
