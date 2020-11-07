//
// Created by uav on 2020/10/30.
//

#include "core.h"


int loopStep = 0;
double frontLoopDistance = 1.5;
double behindLoopDistance = 0.5;
double height = 1.0;

double blindPoints[4][10] =
        {
                ///x, y, z, yaw, vx, vy, vz, ax, ay, az
                {10.5 , 2.5, height , 0, 1, 0, 0, 0, 0, 0},
                {10.5 , 2.5, height+2 , 0, 1, 0, 0, 0, 0, 0},
                {12.0 , 2.5, height+2 , 0, 1, 0, 0, 0, 0, 0},
                {12.0 , 2.0, height , 0, 1, 0, 0, 0, 0, 0},
        };
double frontPoints[6][10] =
        {
                ///x,   y,   z,yaw,vx,vy,vz,ax,ay,az
                {2.5 - frontLoopDistance, -1  , height, 0, 1, 0, 0, 0, 0, 0},
                {5.2 - frontLoopDistance, -2.5, height, 0, 1, 0, 0, 0, 0, 0},
                {7.2 - frontLoopDistance, -1.5, height, 0, 1, 0, 0, 0, 0, 0},
                {9.5 - frontLoopDistance, 2.5 , height, 0, 1, 0, 0, 0, 0, 0},
                {13.5- frontLoopDistance, 2   , height, 0, 1, 0, 0, 0, 0, 0},
                {18  - frontLoopDistance, -2.8, height, 0, 1, 0, 0, 0, 0, 0},

        };

double centerPoints[6][10] =
        {
                ///x, y, z, yaw, vx, vy, vz, ax, ay, az
                {2.5 + behindLoopDistance, -1   , height , 0, 1, 0, 0, 0, 0, 0},
                {5.2 + behindLoopDistance, -2.5 , height , 0, 1, 0, 0, 0, 0, 0},
                {7.2 + behindLoopDistance, -1.5 , height , 0, 1, 0, 0, 0, 0, 0},
                {9.5 + behindLoopDistance, 2.5  , height , 0, 1, 0, 0, 0, 0, 0},
                {13.5+ behindLoopDistance, 2    , height , 0, 1, 0, 0, 0, 0, 0},
                {18  + behindLoopDistance, -2.8 , height , 0, 1, 0, 0, 0, 0, 0},
        };
trajectory_msgs::JointTrajectoryPoint pvaTargetPointMsg;




bool go_to_loop(int numberLoop){

    switch(loopStep){
        ///send frontpoint to px4
        case 0:
            ///set pvaTargetPointMsg values
            update_drift(numberLoop);
            setFrontPva(numberLoop);
            pubPvaTargetPoint.publish(pvaTargetPointMsg);
            if(isArrivedFront(numberLoop))
                loopStep++;
            break;

        ///send centerpoint to px4
        case 1:
            update_drift(numberLoop);
            setCenterPva(numberLoop);
            pubPvaTargetPoint.publish(pvaTargetPointMsg);
            if(isArrivedCenter(numberLoop))
                loopStep++;
            break;

        case 2:
            return true;

        default:
            return false;

    }


}

/**
 * set frontPoint value in front of loop
 */
void setFrontPva(int numberLoop){
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][0]-drift.x());
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][1]-drift.y());
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][2]-drift.z());
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][3]);

    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][4]);
    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][5]);
    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][6]);

    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][7]);
    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][8]);
    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][9]);

    pvaTargetPointMsg.effort.push_back(numberLoop);
}

/**
 * set centerPoint value of loop
 */
void setCenterPva(int numberLoop){
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(centerPoints[numberLoop][0]-drift.x());
    pvaTargetPointMsg.positions.push_back(centerPoints[numberLoop][1]-drift.y());
    pvaTargetPointMsg.positions.push_back(centerPoints[numberLoop][2]-drift.z());
    pvaTargetPointMsg.positions.push_back(centerPoints[numberLoop][3]);

    pvaTargetPointMsg.velocities.push_back(centerPoints[numberLoop][4]);
    pvaTargetPointMsg.velocities.push_back(centerPoints[numberLoop][5]);
    pvaTargetPointMsg.velocities.push_back(centerPoints[numberLoop][6]);

    pvaTargetPointMsg.accelerations.push_back(centerPoints[numberLoop][7]);
    pvaTargetPointMsg.accelerations.push_back(centerPoints[numberLoop][8]);
    pvaTargetPointMsg.accelerations.push_back(centerPoints[numberLoop][9]);

    pvaTargetPointMsg.effort.push_back(numberLoop);
    pvaTargetPointMsg.effort.push_back(loopStep);
}


/**
 * check if drone has arrived the front target point
 */
bool isArrivedFront(int numberLoop){
    if(abs(dronePoseLp.pose.position.x-frontPoints[numberLoop][0]+drift.x())<0.1 &&
       abs(dronePoseLp.pose.position.y-frontPoints[numberLoop][1]+drift.y())<0.1 &&
       abs(planeCurrHeight-frontPoints[numberLoop][2]+drift.z())<0.1)
        return true;
    else
        return false;
}

/**
 * check if drone has arrived the center target point
 */
bool isArrivedCenter(int numberLoop){
    if(abs(dronePoseLp.pose.position.x-centerPoints[numberLoop][0]+drift.x())<0.1 &&
       abs(dronePoseLp.pose.position.y-centerPoints[numberLoop][1]+drift.y())<0.1 &&
       abs(planeCurrHeight-centerPoints[numberLoop][2]+drift.z())<0.1)
        return true;
    else
        return false;
}

/**
 * update drift values through visionPose and dronePoseLp
 */
void update_drift(int numberLoop)
{
    ///drift = given - visionPose
    drift.x() += (1.5 - visionPose.pose.position.x);
    drift.y() += (-visionPose.pose.position.y);
    drift.z() += (1 - visionPose.pose.position.z);

}
