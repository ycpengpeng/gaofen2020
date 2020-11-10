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
int hoverFunCount=1000;
void setHoverPva();


void setBeforeOffbPva()
{
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(dronePoseLp.pose.position.x);
    pvaTargetPointMsg.positions.push_back(dronePoseLp.pose.position.y);
    pvaTargetPointMsg.positions.push_back(dronePoseLp.pose.position.z);
    pvaTargetPointMsg.positions.push_back(0);

    pvaTargetPointMsg.velocities.push_back(0);
    pvaTargetPointMsg.velocities.push_back(0);
    pvaTargetPointMsg.velocities.push_back(0);

    pvaTargetPointMsg.accelerations.push_back(0);
    pvaTargetPointMsg.accelerations.push_back(0);
    pvaTargetPointMsg.accelerations.push_back(0);

    pvaTargetPointMsg.effort.push_back(0);

}



double takeOffPoint[10]=
        {
                ///x, y, z, yaw, vx, vy, vz, ax, ay, az
                0 , 0 , 0 , 0, 0, 0, 0, 0, 0, 0
        };

/**
 * get yawdegree before taking off
 */
bool get_yaw_fun(){

/*    droneEuler = quaternion2euler_eigen(dronePoseLp.pose.orientation.x,dronePoseLp.pose.orientation.y,dronePoseLp.pose.orientation.z,dronePoseLp.pose.orientation.w);
    //droneEuler = quaternion2euler_eigen(dronePoseT265.pose.pose.orientation.x,dronePoseT265.pose.pose.orientation.y,dronePoseT265.pose.pose.orientation.z,dronePoseT265.pose.pose.orientation.w);
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
    }*/
return true;
}


/**
 * take off precess
 */
bool take_off_func()
{

/*    if(currentStateMsg.armed) {
        ROS_WARN("Armed !");
    }
    else{
        ROS_INFO("Not armed !");
    }*/

    ///checking Mode before taking off

    ROS_INFO_ONCE("TAKE_OFF MODE!!!");
    if(currentStateMsg.mode == "OFFBOARD")
    {
/*        ROS_WARN("OFFBOARD NOW !");
        ROS_WARN("Ready to climb to home hover height");
        ROS_WARN("Height now is %f", planeCurrHeight);
        ROS_WARN("Required height is %f", homeHoverHeight);*/
        if(abs(planeCurrHeight-homeHoverHeight) < 0.1){
            return true;
        }
    }

    ///else, continuing send target point to px4
//    px4PointMsg.header.stamp = ros::Time::now();
//    px4PointMsg.pose.position.x = droneHome.x();
//    px4PointMsg.pose.position.y = droneHome.y();
//    px4PointMsg.pose.position.z = homeHoverHeight;

    ///set takeoff point
    takeOffPoint[0] = 0;
    takeOffPoint[1] = 0;
    takeOffPoint[2] = homeHoverHeight;

    setTakeOffPva();
    pubPvaTargetPoint.publish(pvaTargetPointMsg);

    //pubPx4Point.publish(px4PointMsg);
//    cout << "px4_position_msg.pose.position.x: " << px4PointMsg.pose.position.x << endl;
//    cout << "px4_position_msg.pose.position.y: " << px4PointMsg.pose.position.y << endl;
//    cout << "px4_position_msg.pose.position.z: " << px4PointMsg.pose.position.z << endl;
/*      cout << "takeOffPoints.x: " << droneHome.x() << endl;
      cout << "takeOffPoints.y: " << droneHome.y()<< endl;
      cout << "takeOffPoints.z: " << homeHoverHeight << endl;  //homeHoverHeight =  1.0 m*/


    return false;
}


/**
 * hovering and adjusting process
 */
bool hover_and_adjust_func()
{

    hoverFunCount--;
    ROS_INFO_ONCE("HOVER MODE!!!!!");
/*    if(hoverFunCount)
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
    }*/

    if(hoverFunCount==0)
    {
        return true;
    }
    else if(hoverFunCount==-1000)
    {
        return true;
    }
    else
    {
        setHoverPva();
        pubPvaTargetPoint.publish(pvaTargetPointMsg);
        return false;
    }

}


/**
 * set frontPoint value in front of loop
 */
void setTakeOffPva(){
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(takeOffPoint[0]);
    pvaTargetPointMsg.positions.push_back(takeOffPoint[1]);
    pvaTargetPointMsg.positions.push_back(takeOffPoint[2]);
    pvaTargetPointMsg.positions.push_back(takeOffPoint[3]);

    pvaTargetPointMsg.velocities.push_back(takeOffPoint[4]);
    pvaTargetPointMsg.velocities.push_back(takeOffPoint[5]);
    pvaTargetPointMsg.velocities.push_back(takeOffPoint[6]);

    pvaTargetPointMsg.accelerations.push_back(takeOffPoint[7]);
    pvaTargetPointMsg.accelerations.push_back(takeOffPoint[8]);
    pvaTargetPointMsg.accelerations.push_back(takeOffPoint[9]);

    pvaTargetPointMsg.effort.push_back(-1);
}
 void setHoverPva()
 {
     pvaTargetPointMsg.positions.clear();
     pvaTargetPointMsg.velocities.clear();
     pvaTargetPointMsg.accelerations.clear();
     pvaTargetPointMsg.effort.clear();

     pvaTargetPointMsg.positions.push_back(1.1);
     pvaTargetPointMsg.positions.push_back(1.2);
     pvaTargetPointMsg.positions.push_back(1.3);
     pvaTargetPointMsg.positions.push_back(0);

     pvaTargetPointMsg.velocities.push_back(0);
     pvaTargetPointMsg.velocities.push_back(0);
     pvaTargetPointMsg.velocities.push_back(0);

     pvaTargetPointMsg.accelerations.push_back(0);
     pvaTargetPointMsg.accelerations.push_back(0);
     pvaTargetPointMsg.accelerations.push_back(0);

     pvaTargetPointMsg.effort.push_back(-3);


 }
