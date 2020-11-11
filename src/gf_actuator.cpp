//
// Created by cc on 2020/8/5.
//

#include <gf_actuator.h>

using namespace Eigen;

int COUNT_QUAN=1;
mavros_msgs::State current_state;
ros::Publisher pva_pub;


euler ang;

pva_table *table = (pva_table*)malloc(sizeof(pva_table));

MatrixXd p_t, v_t, a_t;
VectorXd yaw_t;
Vector3d current_p(0,0,0);
Vector3d current_v(0,0,0);
Vector3d last_current_v(0,0,0);
Vector3d current_a(0,0,0);
Vector3d last_a(0,0,0);
Vector3d planned_p,planned_v,planned_a;
Vector3d last_planned_p(0,0,0);
double planned_yaw;

Eigen::Vector3d current_euler;

Quaterniond current_att;
int I=0;

//Quaterniond _q(0,0.7071,0.7071,0);  //从北东地到东北天的四元数
Quaterniond _q(1,0,0,0);
ros::Time last_time;
mavros_msgs::AttitudeTarget att_setpoint;
ros::Publisher att_ctrl_pub;
ros::Publisher odom_sp_enu_pub;
ros::Publisher path_pub;
geometry_msgs::PoseStamped pose;

unsigned int t_number=0;     //离散点的数量


Eigen::Vector3d quaternion2euler_eigen(float x, float y, float z, float w)
{
    Eigen::Vector3d temp;
    temp.x() = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    // I use ENU coordinate system , so I plus ' - '
    temp.y() = - asin(2.0 * (z * x - w * y));
    temp.z() = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

void positionCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
/*    /// ENU frame to NWU
    current_p << msg->pose.position.y, -msg->pose.position.x, msg->pose.position.z;*/

    static ros::Time position_cb_time=ros::Time::now();
    position_cb_time=ros::Time::now();


}

void attCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    ///TODO 以下部分仅仅用于测试
    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    current_att.w() = msg->pose.orientation.w;
    current_att.x() = msg->pose.orientation.x;
    current_att.y() = msg->pose.orientation.y;
    current_att.z() = msg->pose.orientation.z;
    current_att=_q*current_att*_q.inverse();
    current_euler=quaternion2euler_eigen(current_att.x(), current_att.y(), current_att.z(), current_att.w());
}



void velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_v << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
    current_a=(current_v-last_current_v)/(ros::Time::now().toSec()-last_time.toSec());
    if(current_a(0)>100||current_a(0)<-100)
    {
        current_a=last_a;
    }
    last_time=ros::Time::now();
    last_current_v=current_v;
    last_a=current_a;
}


void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
void motion_primitives_with_table(Vector3d p0,Vector3d v0,Vector3d a0,Vector3d pf,Vector3d vf,Vector3d af,unsigned int &t_num,
                                  double yawf);

void pva_takeoff(double take_off_height);
void pva_land();

void setpoint_takeoff(double take_off_height);
void setpoint_land();


void zuan_quan_set_point_cb(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
{
    //ros::Time tmp=ros::Time::now();

    planned_p << msg->positions[0], msg->positions[1], msg->positions[2];
    if(last_planned_p(0)==planned_p(0)&&last_planned_p(1)==planned_p(1)&&last_planned_p(2)==planned_p(2))
    {
        return;
    }
    int numberloop=msg->effort[0];

    if(numberloop==-1)//Take Off!!!!!!!!!!
    {
        //pva_takeoff(planned_p(2));
        setpoint_takeoff(planned_p(2));
        last_planned_p=planned_p;
        I=0;
    }
    else if(numberloop==-2)//Land!!!!
    {
        //pva_land();
        setpoint_land();
        last_planned_p=planned_p;
        I=0;
    }

    else if(numberloop==-3)//hover!!!!
    {
        Vector3d hover_position=current_p;
        Vector3d p_sp(hover_position(0), hover_position(1), hover_position(2));
        Vector3d v_sp(0, 0, 0);
        Vector3d a_sp(0, 0, 0);
        t_number=1;
        p_t.row(0)=p_sp;
        v_t.row(0)=v_sp;
        a_t.row(0)=a_sp;
        yaw_t(0)=current_euler.z();
        last_planned_p=planned_p;
        I=0;
    }

    else if(numberloop>=0)//zuan_quan!!!!!!!!!!!!!!!!!!!!
    {
        //ROS_INFO("gf_actuator:NOW target number loop %d",numberloop);
        ROS_INFO("gf_actuator:planned_P %f %f %f",planned_p(0),planned_p(1),planned_p(2));
        planned_yaw = msg->positions[3];
        planned_v << msg->velocities[0], msg->velocities[1], msg->velocities[2];
        planned_a << msg->accelerations[0], msg->accelerations[1], msg->accelerations[2];

        static bool init=1;
        if(init==0)
        {
            current_p=p_t.row(t_number-1);
            current_v=v_t.row(t_number-1);
            current_a=a_t.row(t_number-1);
        }
        init=1;  //如果不把这句话注释掉，那么飞机将会按照当前实际位置进行下一次规划

/*        //path_pub用于rqt_plot测试, 发送到/path话题
        nav_msgs::Odometry path_position;
        path_position.header.stamp = ros::Time::now();
        path_position.pose.pose.position.y=current_p(0);
        path_pub.publish(path_position);*/

        motion_primitives_with_table(current_p,current_v,current_a,planned_p,planned_v,planned_a,t_number,planned_yaw);
        last_planned_p=planned_p;
        I=0;
        COUNT_QUAN=-COUNT_QUAN;
        // ROS_INFO("planned_p(0)  %f  planned_p(1)  %f  planned_p(2)   %f",planned_p(0),planned_p(1),planned_p(2));
        //ROS_INFO("duration:   %f",ros::Time::now().toSec()-tmp.toSec());
    }
}

void setpoint_takeoff(double take_off_height)
{
    Vector3d take_off_position=current_p;
    double take_off_yaw=current_euler.z();
    double take_off_acc = 0.1;
    double take_off_time=sqrt(take_off_height/take_off_acc)*2.0;
    double take_off_send_times = take_off_time / delta_t;
    double times=0;
    t_number=take_off_send_times+100;
//    take_off_init=0;
    ros::Time last_time=ros::Time::now();

    p_t = Eigen::MatrixXd::Zero(t_number, 3);
    v_t = Eigen::MatrixXd::Zero(t_number, 3);
    a_t = Eigen::MatrixXd::Zero(t_number, 3);
    yaw_t = Eigen::VectorXd::Zero(t_number);

    for(times=0;times<t_number;times++)
    {
        if(times < take_off_send_times / 2.0)
        {
            double z_sp = 0.5*take_off_acc*times*delta_t*times*delta_t;
            double vz_sp = times*delta_t*take_off_acc;
            Vector3d p_sp(take_off_position(0), take_off_position(1), z_sp);
            Vector3d v_sp(0, 0, vz_sp);
            Vector3d a_sp(0, 0, 0);
            p_t.row(times)=p_sp;
            v_t.row(times)=v_sp;
            a_t.row(times)=a_sp;
            yaw_t(times)=take_off_yaw;
        }
        else if(times < take_off_send_times)
        {
            double t_this = (times-take_off_send_times/2.0)*delta_t;
            double vz_sp = take_off_send_times/2.0*delta_t*take_off_acc - take_off_acc*t_this;
            double z_sp=1.0/2.0*take_off_acc*(take_off_time/2.0)*(take_off_time/2.0)+(vz_sp+take_off_time/2.0*take_off_acc)*t_this/2.0;

            //ROS_INFO("Z_SP:%f",z_sp);
            Vector3d p_sp(take_off_position(0), take_off_position(1), z_sp);
            Vector3d v_sp(0, 0, vz_sp);
            Vector3d a_sp(0, 0, -0);
            p_t.row(times)=p_sp;
            v_t.row(times)=v_sp;
            a_t.row(times)=a_sp;
            yaw_t(times)=0;
        }
        else
        {
            Vector3d tmp(take_off_position(0),take_off_position(1),take_off_height);
            p_t.row(times)=tmp;
            v_t.row(times)=Vector3d::Zero();
            a_t.row(times)=Vector3d::Zero();
            yaw_t(times)=0;
        }
    }
}




void pva_takeoff(double take_off_height)
{
    Vector3d take_off_position=current_p;
    double take_off_yaw=current_euler.z();
    double take_off_acc = 0.1;
    double take_off_time=sqrt(take_off_height/take_off_acc)*2.0;
    double take_off_send_times = take_off_time / delta_t;
    double times=0;
    t_number=take_off_send_times+100;
//    take_off_init=0;
    ros::Time last_time=ros::Time::now();

    p_t = Eigen::MatrixXd::Zero(t_number, 3);
    v_t = Eigen::MatrixXd::Zero(t_number, 3);
    a_t = Eigen::MatrixXd::Zero(t_number, 3);
    yaw_t = Eigen::VectorXd::Zero(t_number);

    for(times=0;times<t_number;times++)
    {
        if(times < take_off_send_times / 2.0)
        {
            double z_sp = 0.5*take_off_acc*times*delta_t*times*delta_t;
            double vz_sp = times*delta_t*take_off_acc;
            Vector3d p_sp(take_off_position(0), take_off_position(1), z_sp);
            Vector3d v_sp(0, 0, vz_sp);
            Vector3d a_sp(0, 0, 0);
            p_t.row(times)=p_sp;
            v_t.row(times)=v_sp;
            a_t.row(times)=a_sp;
            yaw_t(times)=take_off_yaw;
        }
        else if(times < take_off_send_times)
        {
            double t_this = (times-take_off_send_times/2.0)*delta_t;
            double vz_sp = take_off_send_times/2.0*delta_t*take_off_acc - take_off_acc*t_this;
            double z_sp=1.0/2.0*take_off_acc*(take_off_time/2.0)*(take_off_time/2.0)+(vz_sp+take_off_time/2.0*take_off_acc)*t_this/2.0;

            //ROS_INFO("Z_SP:%f",z_sp);
            Vector3d p_sp(take_off_position(0), take_off_position(1), z_sp);
            Vector3d v_sp(0, 0, vz_sp);
            Vector3d a_sp(0, 0, -0);
            p_t.row(times)=p_sp;
            v_t.row(times)=v_sp;
            a_t.row(times)=a_sp;
            yaw_t(times)=0;
        }
        else
        {
            Vector3d tmp(take_off_position(0),take_off_position(1),take_off_height);
            p_t.row(times)=tmp;
            v_t.row(times)=Vector3d::Zero();
            a_t.row(times)=Vector3d::Zero();
            yaw_t(times)=0;
        }
    }
}

void setpoint_land()
{

    Vector3d land_position=current_p;

    double land_acc = 0.1;
    double land_time=sqrt(land_position(2)/land_acc)*2.0;
    double land_send_times = land_time / delta_t;
    double times=0;
    t_number=land_send_times+100;

//    take_off_init=0;

    ros::Time last_time=ros::Time::now();

    p_t = Eigen::MatrixXd::Zero(t_number, 3);
    v_t = Eigen::MatrixXd::Zero(t_number, 3);
    a_t = Eigen::MatrixXd::Zero(t_number, 3);

    for(times=0;times<t_number;times++)
    {
        if(times < land_send_times / 2.0)
        {
            double z_sp = land_position(2)-0.5*land_acc*times*delta_t*times*delta_t;
            double vz_sp = -times*delta_t*land_acc;
            Vector3d p_sp(land_position(0), land_position(1), z_sp);
            Vector3d v_sp(0, 0, vz_sp);
            Vector3d a_sp(0, 0, 0);
            p_t.row(times)=p_sp;
            v_t.row(times)=v_sp;
            a_t.row(times)=a_sp;
        }
        else if(times < land_send_times)
        {
            double t_this = (times-land_send_times/2.0)*delta_t;
            double vz_sp = -(land_send_times/2.0*delta_t*land_acc - land_acc*t_this);
            double  z_sp = land_position(2)-(1.0/2.0*land_acc*(land_time/2.0)*(land_time/2.0)+(-vz_sp+land_time/2.0*land_acc)*t_this/2.0);
            //ROS_INFO("Z_SP:%f",z_sp);
            Vector3d p_sp(land_position(0), land_position(1), z_sp);
            Vector3d v_sp(0, 0, vz_sp);
            Vector3d a_sp(0, 0, -0);
            p_t.row(times)=p_sp;
            v_t.row(times)=v_sp;
            a_t.row(times)=a_sp;
        }
        else
        {
            // ROS_INFO("LAND OVER---------------------------");
            p_t.row(times)<<land_position(0),land_position(1),-100;
            v_t.row(times)=Vector3d::Zero();
            a_t.row(times)=Vector3d::Zero();
        }
    }
}





void pva_land()
{

    Vector3d land_position=current_p;

    double land_acc = 0.1;
    double land_time=sqrt(land_position(2)/land_acc)*2.0;
    double land_send_times = land_time / delta_t;
    double times=0;
    t_number=land_send_times+100;

//    take_off_init=0;

    ros::Time last_time=ros::Time::now();

    p_t = Eigen::MatrixXd::Zero(t_number, 3);
    v_t = Eigen::MatrixXd::Zero(t_number, 3);
    a_t = Eigen::MatrixXd::Zero(t_number, 3);

    for(times=0;times<t_number;times++)
    {
        if(times < land_send_times / 2.0)
        {
            double z_sp = land_position(2)-0.5*land_acc*times*delta_t*times*delta_t;
            double vz_sp = -times*delta_t*land_acc;
            Vector3d p_sp(land_position(0), land_position(1), z_sp);
            Vector3d v_sp(0, 0, vz_sp);
            Vector3d a_sp(0, 0, 0);
            p_t.row(times)=p_sp;
            v_t.row(times)=v_sp;
            a_t.row(times)=a_sp;
        }
        else if(times < land_send_times)
        {
            double t_this = (times-land_send_times/2.0)*delta_t;
            double vz_sp = -(land_send_times/2.0*delta_t*land_acc - land_acc*t_this);
            double  z_sp = land_position(2)-(1.0/2.0*land_acc*(land_time/2.0)*(land_time/2.0)+(-vz_sp+land_time/2.0*land_acc)*t_this/2.0);
            //ROS_INFO("Z_SP:%f",z_sp);
            Vector3d p_sp(land_position(0), land_position(1), z_sp);
            Vector3d v_sp(0, 0, vz_sp);
            Vector3d a_sp(0, 0, -0);
            p_t.row(times)=p_sp;
            v_t.row(times)=v_sp;
            a_t.row(times)=a_sp;
        }
        else
        {
           // ROS_INFO("LAND OVER---------------------------");
            p_t.row(times)<<land_position(0),land_position(1),-100;
            v_t.row(times)=Vector3d::Zero();
            a_t.row(times)=Vector3d::Zero();
        }
    }
}

void motion_primitives_with_table(Vector3d p0,Vector3d v0,Vector3d a0,Vector3d pf,Vector3d vf,Vector3d af,unsigned int &t_num,
                                  double yawf)
{
    double T1, T2, T3, T;
    double delt_x, delt_y, delt_z;
    //ROS_INFO("pf(0):  %f,pf(1):   %f,pf(2):   %f",pf(0),pf(1),pf(2));
    delt_x=pf(0)-p0(0);
    delt_y=pf(1)-p0(1);
    delt_z=pf(2)-p0(2);

    if(fabs(delt_x)>5)
    {
        delt_x = 5.0 * delt_x / fabs(delt_x);
    }
    if(fabs(delt_y)>5)
    {
        delt_y = 5.0 * delt_y / fabs(delt_y);
    }
    if(fabs(delt_z)>5)
    {
        delt_z = 5.0 * delt_z / fabs(delt_z);
    }

    T1 = table->query_pva_table(delt_x, v0(0), vf(0), a0(0));
    T2 = table->query_pva_table(delt_y, v0(1), vf(1), a0(1));
    T3 = table->query_pva_table(delt_z, v0(2), vf(2), a0(2));
    T = T1 > T2 ? T1 : T2;
    T = T > T3 ? T : T3;
    T = T < 0.5 ? 0.5 : T;
    if(T==-1)
    {
        ROS_INFO("T=-1////////////////");
/*        delt_x=pf(0)-p0(0);
        delt_y=pf(1)-p0(1);
        delt_z=pf(2)-p0(2);*/

    }

    //ROS_INFO("T:%f",T);
    t_num=T/delta_t;  //number of dots
    //ROS_INFO("computer:t_number:%d",t_num);
    //ROS_INFO("t_num:%d   qqq",t_num);
    p_t = Eigen::MatrixXd::Zero(t_num, 3);
    v_t = Eigen::MatrixXd::Zero(t_num, 3);
    a_t = Eigen::MatrixXd::Zero(t_num, 3);
    //t = Eigen::VectorXd::Zero(t_num);
    for (int column = 0; column < 3; column++)
    {
        double delt_a = af(column) - a0(column);
        double delt_v = vf(column) - v0(column) - a0(column)*T;
        double delt_p = pf(column) - p0(column) - v0(column)*T - 0.5*a0(column)*T*T;

        // % if vf is not free
        double alpha = delt_a*60/pow(T,3) - delt_v*360/pow(T,4) + delt_p*720/pow(T,5);
        double beta = -delt_a*24/pow(T,2) + delt_v*168/pow(T,3) - delt_p*360/pow(T,4);
        double gamma = delt_a*3/T - delt_v*24/pow(T,2) + delt_p*60/pow(T,3);


        for (int times = 0; times < t_num; times++)
        {

            //ROS_INFO("times=%d",times);
            double tt = (times + 1)*delta_t;
            p_t(times, column) = alpha / 120 * pow(tt, 5) + beta / 24 * pow(tt, 4) + gamma / 6 * pow(tt, 3) +
                                 a0(column) / 2 * pow(tt, 2) +v0(column) * tt + p0(column);
            v_t(times, column) = alpha / 24 * pow(tt, 4) + beta / 6 * pow(tt, 3) + gamma / 2 * pow(tt, 2) +
                                 a0(column) * tt + v0(column);
            a_t(times, column) = alpha / 6 * pow(tt, 3) + beta / 2 * pow(tt, 2) + gamma * tt + a0(column);

        }
        //ROS_INFO("p_t(t_num-1,0):  %f,p_t(t_num-1,1):  %f,p_t(t_num-1,2):  %f",p_t(t_num-1,0),p_t(t_num-1,1),p_t(t_num-1,2));

    }
    //ROS_INFO("p_t(t_num-1,0):  %f,p_t(t_num-1,1):  %f,p_t(t_num-1,2):  %f",p_t(t_num-1,0),p_t(t_num-1,1),p_t(t_num-1,2));

    yaw_t = Eigen::VectorXd::Zero(t_num);
    for(int times=0;times<t_num;times++)
    {
        yaw_t(times)=yawf;
    }
}

void setPVA(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw=0.0)
{
    trajectory_msgs::JointTrajectoryPoint pva_setpoint;
    static double last_send_px=0;

    if(p(0)-last_send_px>0.5)
    {
        ROS_INFO("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
        ROS_INFO("now p(0):%f",p(0));
        ROS_INFO("last_send_px:%f",last_send_px);
        ROS_INFO("planned_p(0):%f",planned_p(0));
        ROS_INFO("NOW I:%d",I);
        ROS_INFO("NOW T_NUMBBER:%d",t_number);
//        ROS_INFO("current_p(0):%f",current())
        ROS_INFO("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");

    }

    pva_setpoint.positions.push_back(p(0)); //x
    pva_setpoint.positions.push_back(p(1)); //y
    pva_setpoint.positions.push_back(p(2)); //z
    pva_setpoint.positions.push_back(yaw);

    pva_setpoint.velocities.push_back(v(0));
    pva_setpoint.velocities.push_back(v(1));
    pva_setpoint.velocities.push_back(v(2));
    pva_setpoint.accelerations.push_back(a(0));
    pva_setpoint.accelerations.push_back(a(1));
    pva_setpoint.accelerations.push_back(a(2));
    /// Publish to record in rosbag
    nav_msgs::Odometry odom_sp_enu;
    odom_sp_enu.header.stamp = ros::Time::now();
    odom_sp_enu.pose.pose.position.x = p(0);
    odom_sp_enu.pose.pose.position.y = p(1);
    odom_sp_enu.pose.pose.position.z = p(2);
    odom_sp_enu.twist.twist.linear.x = v(0);
    odom_sp_enu.twist.twist.linear.y = v(1);
    odom_sp_enu.twist.twist.linear.z = v(2);
    odom_sp_enu_pub.publish(odom_sp_enu);

    pva_pub.publish(pva_setpoint);
    last_send_px=p(0);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gf_actuator");
    ros::NodeHandle nh;
    table->csv2pva_table("/home/pengpeng/Desktop/p5_v0.5_a2_res0-1.csv");

    ros::Rate loop_rate(LOOPRATE);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);

    //ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1, positionCallback);

    ///TODO:以下代码仅仅用于仿真


    ros::Subscriber att_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, attCallback);

    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1, velocity_sub_cb);
    ros::Subscriber traj_point_sub = nh.subscribe<trajectory_msgs::JointTrajectoryPoint>("/zuan_quan_setpoint", 1, zuan_quan_set_point_cb);

    ros::Publisher velocity_pub=nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    odom_sp_enu_pub = nh.advertise<nav_msgs::Odometry>("/odom_sp_enu", 1);
    path_pub=nh.advertise<nav_msgs::Odometry>("/path", 1);
    Vector3d hover_position;

    while(ros::ok())
    {
        ros::spinOnce();
        for(I=0;I<t_number;I++)
        {
            //ROS_INFO("DDDDD");
            //setPVA(p_t.row(I), v_t.row(I), a_t.row(I), yaw_t(I));//a_t.row(last_index));
            pose.pose.position.x = p_t(I,0);
            pose.pose.position.y = p_t(I,1);
            pose.pose.position.z = p_t(I,2);

            pose.pose.orientation.w=cos(yaw_t(I)/2);
            pose.pose.orientation.x=0;
            pose.pose.orientation.y=0;
            pose.pose.orientation.z=sin(yaw_t(I)/2);
            local_pos_pub.publish(pose);




            ros::spinOnce();
            loop_rate.sleep();
        }
/*        if(I==t_number&&t_number!=0)  //在钻完上一个圈之后，没有收到下一个圈的目标点，就进入悬停模式
        {
            Vector3d p_sp(hover_position(0), hover_position(1), hover_position(2));
            Vector3d v_sp(0, 0, 0);
            setPVA(p_sp, v_sp, Vector3d::Zero(), yaw_set);
        }*/
//ROS_INFO("0------------I:%d  t_number:%d",I,t_number);
        while(t_number!=0&&I!=0&&I==t_number)
        {
            //ROS_INFO("AAAASSSDADADAD");
            //setPVA(p_t.row(t_number-1), Vector3d::Zero(), Vector3d::Zero(), yaw_t(t_number-1));//a_t.row(last_index));
            local_pos_pub.publish(pose);
            ros::spinOnce();
            loop_rate.sleep();
        }
        //ROS_INFO("I:%d  t_number:%d",I,t_number);


    }
    return 0;
}
