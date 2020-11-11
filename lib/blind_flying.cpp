//
// Created by uav on 2020/11/6.
//

#include "core.h"

int blindStep = 0;


bool blind_fly(int blindPoint)
{
    if(blindPoint==0)
    {
        ROS_INFO_ONCE("go to blindPoint:%d",blindPoint);
    }
    if(blindPoint==1)
    {
        ROS_INFO_ONCE("go to blindPoint:%d",blindPoint);
    }
    if(blindPoint==2)
    {
        ROS_INFO_ONCE("go to blindPoint:%d",blindPoint);
    }
    if(blindPoint==3)
    {
        ROS_INFO_ONCE("go to blindPoint:%d",blindPoint);
    }
    if(blindPoint==4)
    {
        ROS_INFO_ONCE("go to blindPoint:%d",blindPoint);
    }
    update_drift(blindPoint);
    /*drift.x() =  0;
    drift.y() = 0;
    drift.z() = 0;*/
    setBlindPva(blindPoint);
    pubPvaTargetPoint.publish(pvaTargetPointMsg);
    if(isArrivedBlind(blindPoint))
    {
        return true;
    }
    else
    {
        return false;
    }

}


/**
 * set blindPoint value in front of loop
 */
void setBlindPva(int blindPoint){
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(blindPoints[blindPoint][0]-drift.x());
    pvaTargetPointMsg.positions.push_back(blindPoints[blindPoint][1]-drift.y());
    pvaTargetPointMsg.positions.push_back(blindPoints[blindPoint][2]-drift.z());
    pvaTargetPointMsg.positions.push_back(blindPoints[blindPoint][3]);

    pvaTargetPointMsg.velocities.push_back(blindPoints[blindPoint][4]);
    pvaTargetPointMsg.velocities.push_back(blindPoints[blindPoint][5]);
    pvaTargetPointMsg.velocities.push_back(blindPoints[blindPoint][6]);

    pvaTargetPointMsg.accelerations.push_back(blindPoints[blindPoint][7]);
    pvaTargetPointMsg.accelerations.push_back(blindPoints[blindPoint][8]);
    pvaTargetPointMsg.accelerations.push_back(blindPoints[blindPoint][9]);

    pvaTargetPointMsg.effort.push_back(10);
}

/**
 * check if drone has arrived the blind target point
 */
bool isArrivedBlind(int blindPoint){
    if(abs(dronePoseLp.pose.position.x-(blindPoints[blindPoint][0]-drift.x())) < 0.1 &&
       abs(dronePoseLp.pose.position.y-(blindPoints[blindPoint][1]-drift.y()) < 0.1 &&
       abs(planeCurrHeight-(blindPoints[blindPoint][2]-drift.z())) < 0.1))
    {
        return true;
    }
    else
    {
        return false;

    }

}

