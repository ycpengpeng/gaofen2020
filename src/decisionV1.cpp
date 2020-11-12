//
// Created by uav on 2020/10/30.
//


#include "core.h"
using namespace std;
using namespace Eigen;

void stateVisionCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void stateT265Cb(const nav_msgs::Odometry::ConstPtr& msg);
void stateHeightCb(const mavros_msgs::Altitude::ConstPtr &msg);
void statePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void stateModeCb(const mavros_msgs::State::ConstPtr& msg);
void stateDownCamerePoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void setBeforeOffbPva();


int main(int argc, char **argv) {
    ros::init(argc, argv, "decisionV1");
    ros::NodeHandle nh;
    ros::Rate rate(40);

    ros_callback_func();
    /**
     * main loop
     */
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        //if(currentStateMsg.mode != "OFFBOARD" || !currentStateMsg.armed)
        if(currentStateMsg.mode != "OFFBOARD" )
        {
          //  ROS_INFO("090909");
            stateStep=1;
            geometry_msgs::PoseStamped pose;

            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0.01;
            double theta=90.0/180.0*3.1415926;
            pose.pose.orientation.w=cos(theta/2);
            pose.pose.orientation.x=0;
            pose.pose.orientation.y=0;
            pose.pose.orientation.z=sin(theta/2);
            local_pos_pub.publish(pose);

            // setBeforeOffbPva();
            // pubPvaTargetPoint.publish(pvaTargetPointMsg);

            // continue;
        }
        switch (stateStep)
        {
            case 0: if(get_yaw_fun()){
                stateStep += 1;
            }
            break;

            case 1: if(take_off_func(currentStateMsg)){
                stateStep += 1;
            }
            
            break;
            
            case 2: if(hover_and_adjust_func()){
                stateStep += 1;
            }
            break;

            ///first loop
            case 3: if(go_to_loop(0)){
                stateStep += 1;
            }
            break;

            ///second loop
            case 4: if(go_to_loop(1)){
                    stateStep += 1;
            }
            break;

            ///third loop
            case 5: if(go_to_loop(2)){
                    stateStep += 1;
            }
            break;

            ///forth loop
            case 6: if(go_to_loop(3)){
                    stateStep += 1;
            }
            break;

            ///fifth loop
            case 7: if(go_to_loop(4)){
                    stateStep += 1;
            }
            break;


            //blind flight 1
            case 8: if(blind_fly(0)){
                    stateStep +=1;
            }
            break;
            //blind flight 1
            case 9: if(blind_fly(1)){
                    stateStep +=1;
                }
            break;

            case 10: if(go_to_loop(5)){
                    stateStep +=1;
                }
            break;

            case 11: if(blind_fly(2)){
                    stateStep +=1;
                }
            break;

            case 12: if(hover_and_adjust_func()){
                    stateStep += 1;
                }
                break;

            //land off
            case 13: if(land_off(landOffStep)){
                    stateStep +=1;
                    }
                    break;

        }
    }
    return 0;
}