//
// Created by uav on 2020/10/30.
//
#include "core.h"

/**
 * get Yawdegree
 */
bool get_yaw_fun(){
    droneEuler = quaternion2euler_eigen(dronePoseLp.pose.orientation.x,dronePoseLp.pose.orientation.y,dronePoseLp.pose.orientation.z,dronePoseLp.pose.orientation.w);
    initYaw = droneEuler.z();


    prepareFunCount--;
    if(prepareFunCount)
    {
        droneHomeSum.x() += dronePoseT265.pose.pose.position.x;
        droneHomeSum.y() += dronePoseT265.pose.pose.position.y;
        suminitYaw += initYaw;
        return false;
    }
    else
    {
        initYaw = suminitYaw/100.0;
        droneHome.x() = droneHomeSum.x()/100.0;
        droneHome.y() = droneHomeSum.y()/100.0;
        return true;
    }

}

/**
 * take off
 */
bool take_off_fun(){

    if(currentStateMsg.armed) {
        ROS_WARN("armed !");
    }else{
        ROS_INFO("not armed !");
    }

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
 * hovering and adjusting
 */
bool hover_and_adjust_func()
{
    hoverFunCount--;
    if(hoverFunCount)
    {
        hoverDriftSum.x() += droneDownCameraPose.pose.position.x;
        hoverDriftSum.y() += droneDownCameraPose.pose.position.y;
        pubPx4Point.publish(px4PointMsg);
        return false;
    }
    else{
        hover2homeDrift.x() = hoverDriftSum.x()/100.0;
        hover2homeDrift.y() = hoverDriftSum.y()/100.0;
        drift.x() = hover2homeDrift.x() - dronePoseT265.pose.pose.position.x;
        drift.y() = hover2homeDrift.y() - dronePoseT265.pose.pose.position.y;
        return true;
    }

}


