//
// Created by uav on 2020/10/30.
//


#include "core.h"
using namespace std;
using namespace Eigen;




int main(int argc, char **argv) {
    ros::init(argc, argv, "decisionV1");
    ros::Rate rate(30);
    ros_callback_func();
    /**
     * main loop
     */

    while(ros::ok())
    {   ros::spinOnce();
        switch (stateStep) {
            case 0: if(get_yaw_fun()){
                stateStep += 1;
            }
            break;

            case 1: if(take_off_fun()){
                stateStep += 1;
            }
            break;

            case 2: if(hover_and_adjust_func()){
                stateStep += 1;
            }
            break;

            case 3: if(go_to_loop(0)){
                stateStep += 1;
            }
            break;

        }

        rate.sleep();
    }




}

