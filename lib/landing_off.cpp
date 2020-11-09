//
// Created by uav on 2020/11/6.
//
#include "core.h"



int landOffStep = 0;

double landOffPoints[2][10] =
        {
                ///x, y, z, yaw, vx, vy, vz, ax, ay, az
                {22.0 , 0, height , 0, 0, 0, 0, 0, 0, 0},
                {22.0 , 0, -1     , 0, 0, 0, 0, 0, 0, 0}
        };

bool land_off(int landOffStep)
{
/*    switch(landOffStep)
    {
        case 0:
            setLandPva(landOffStep);
            pubPvaTargetPoint.publish(pvaTargetPointMsg);
            if (isArrivedLand(landOffStep))
                landOffStep++;
            break;

    }*/
    ROS_INFO_ONCE("START LAND---");
    setLandPva(landOffStep);
    pubPvaTargetPoint.publish(pvaTargetPointMsg);
    return false;

}

//check if drone has arrived the blind target point

bool isArrivedLand(int landPoint){
    if(abs(dronePoseLp.pose.position.x-landOffPoints[landPoint][0]+drift.x())<0.1 &&
       abs(dronePoseLp.pose.position.y-landOffPoints[landPoint][1]+drift.y())<0.1 &&
       abs(planeCurrHeight-landOffPoints[landPoint][2]+drift.z())<0.1)
        return true;
    else
        return false;
}

/**
 * set blindPoint value in front of loop
 */
void setLandPva(int landOffStep){
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(landOffPoints[landOffStep][0]-drift.x());
    pvaTargetPointMsg.positions.push_back(landOffPoints[landOffStep][1]-drift.y());
    pvaTargetPointMsg.positions.push_back(landOffPoints[landOffStep][2]-drift.z());
    pvaTargetPointMsg.positions.push_back(landOffPoints[landOffStep][3]);

    pvaTargetPointMsg.velocities.push_back(landOffPoints[landOffStep][4]);
    pvaTargetPointMsg.velocities.push_back(landOffPoints[landOffStep][5]);
    pvaTargetPointMsg.velocities.push_back(landOffPoints[landOffStep][6]);

    pvaTargetPointMsg.accelerations.push_back(landOffPoints[landOffStep][7]);
    pvaTargetPointMsg.accelerations.push_back(landOffPoints[landOffStep][8]);
    pvaTargetPointMsg.accelerations.push_back(landOffPoints[landOffStep][9]);

    pvaTargetPointMsg.effort.push_back(-2);
}

