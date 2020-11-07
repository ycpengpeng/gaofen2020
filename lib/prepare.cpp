//
// Created by uav on 2020/10/30.
//
#include "core.h"

/**
 *  prepare.cpp
 */

//悬停漂移总量
Eigen::Vector2d hoverDriftSum = Eigen::Vector2d::Zero();
//悬停时相对于起飞点的漂移量
Eigen::Vector2d hover2homeDrift = Eigen::Vector2d::Zero();
//
Eigen::Vector3d drift = Eigen::Vector3d::Zero();
int hoverFunCount = 100;



/**
 * get yawdegree before taking off
 */
bool get_yaw_fun(){

    droneEuler = quaternion2euler_eigen(dronePoseLp.pose.orientation.x,dronePoseLp.pose.orientation.y,dronePoseLp.pose.orientation.z,dronePoseLp.pose.orientation.w);
    initYaw = droneEuler.z();

    ///get average-init-number through 100 times of calculation
    getYawFuncCount--;
    if(getYawFuncCount)
    {
        droneHomeSum.x() += dronePoseT265.pose.pose.position.x;
        droneHomeSum.y() += dronePoseT265.pose.pose.position.y;
        suminitYaw += initYaw;
        return false;
    }
    else
    {
        initYaw = suminitYaw/100.0;
        for (int i = 0; i < 6; i++)
        {
            frontPoints[i][3] = initYaw;
            centerPoints[i][3] = initYaw;
            blindPoints[i][3] = initYaw;
        }
        droneHome.x() = droneHomeSum.x()/100.0;
        droneHome.y() = droneHomeSum.y()/100.0;
        return true;
    }
}

/**
 * take off precess
 */
bool take_off_func(){
    ///checking Armed before taking off
    if(currentStateMsg.armed) {  ///jiesuo
        ROS_WARN("Armed !");
    }
    else{
        ROS_INFO("Not armed !");
    }

    ///checking Mode before taking off
    if(currentStateMsg.mode == "OFFBOARD")
    {
        ROS_WARN("OFFBOARD NOW !");
        ROS_WARN("Ready to climb to home hover height");
        ROS_WARN("Height now is %f", planeCurrHeight);
        ROS_WARN("Required height is %f", homeHoverHeight);
        if(abs(planeCurrHeight-homeHoverHeight) < 0.2){
            return true;
        }
    }

    ///else, continuing send target point to px4
    px4PointMsg.header.stamp = ros::Time::now();
    px4PointMsg.pose.position.x = droneHome.x();
    px4PointMsg.pose.position.y = droneHome.y();
    px4PointMsg.pose.position.z = homeHoverHeight;

    pubPx4Point.publish(px4PointMsg);
    cout << "px4_position_msg.pose.position.x: " << px4PointMsg.pose.position.x << endl;
    cout << "px4_position_msg.pose.position.y: " << px4PointMsg.pose.position.y << endl;
    cout << "px4_position_msg.pose.position.z: " << px4PointMsg.pose.position.z << endl;

    return false;
}

/**
 * hovering and adjusting process
 */
bool hover_and_adjust_func()
{
    ///get drifting-number through 100 times of calculation
    hoverFunCount--;
    if(hoverFunCount)
    {
        hoverDriftSum.x() += droneDownCameraPose.pose.position.x;
        hoverDriftSum.y() += droneDownCameraPose.pose.position.y;
        ///continuing send target-hovering point to px4
        pubPx4Point.publish(px4PointMsg);
        return false;
    }
    else{
        hover2homeDrift.x() = hoverDriftSum.x()/100.0;
        hover2homeDrift.y() = hoverDriftSum.y()/100.0;
        ///calculate drift values
        drift.x() += hover2homeDrift.x();
        drift.y() += hover2homeDrift.y();
        return true;
    }

}

