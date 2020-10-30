//
// Created by uav on 2020/10/30.
//

#include "core.h"

bool go_to_loop(int numberLoop){

    switch(loopStep){
        case 0:
            setPva(numberLoop);
            pubPvaTargetPoint.publish(pvaTargetPointMsg);
            pubCurrentPose.publish(dronePoseLp);
            if(isArrived())
                loopStep++;
            break;
        case 1:




    }


    setPva(numberLoop);
    pubPvaTargetPoint.publish(pvaTargetPointMsg);
    pubCurrentPose.publish(dronePoseLp);

}

void setPva(int numberLoop){
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][0]);
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][1]);
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][2]);
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][3]);

    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][4]);
    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][5]);
    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][6]);

    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][7]);
    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][8]);
    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][9]);

    pvaTargetPointMsg.effort.push_back(numberLoop);
}

bool isArrived(int numberLoop){
    if(abs(dronePoseLp.pose.position.x-frontPoints[numberLoop][0])<0.1 &&
       abs(dronePoseLp.pose.position.y-frontPoints[numberLoop][1])<0.1 &&
       abs(planeCurrHeight-frontPoints[numberLoop][2])<0.1)
        return true;
    else
        return false;
}
