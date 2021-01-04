#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

using namespace Eigen;

//imu
Quaterniond qi_wk0(1.0,0.0,0.0,0.0);
Vector3d vel(0.0,0.0,0.0);
Vector3d acc_0(0.0,0.0,9.8),gyr_0(0.0,0.0,0.0);
Vector3d g_w(0.0,0.0,9.8);
//double detal_t(0.005);

//odom
bool start_flag=false;
int iter_count=0;
Quaterniond q_ekw(1.0,0.0,0.0,0.0),q_e0n0(1.0,0.0,0.0,0.0);
Vector3d p_e(0.0,0.0,0.0),p_w0(0.0,0.0,0.0),p_w1(0.0,0.0,0.0),p_e01(0.0,0.0,0.0);
double vel_0(0.0),w_0(0.0),vel_1(0.0),w_1(0.0);
double detal_t = 0.005;

void imu_vel_compute(const sensor_msgs::Imu::ConstPtr &msg)
{
    /*Vector3d acc_1(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z),vel_k;
    Vector3d gyr_1(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    Quaterniond qi_wk1(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
    //Vector3d half_theta= 0.0025 * (gyr_0 + gyr_1);
    //qi_wk1 = qi_wk0*Quaterniond(1.0,half_theta.x(),half_theta.y(),half_theta.z());//计算不准
    vel += 0.0025*(qi_wk0*acc_0-g_w + qi_wk1*acc_1-g_w);
    vel_k=qi_wk1.inverse()*vel;
    acc_0 = acc_1;
    gyr_0 = gyr_1;
    qi_wk0 = qi_wk1;

    ROS_INFO("The vel is:%f %f %f",vel_k.x(),vel_k.y(),vel_k.z());*/	
}

void odom_post_compute(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!start_flag)
    {
        q_ekw = Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z).inverse();
        q_e0n0 = Quaterniond(1.0,0.0,0.0,0.0);
        p_e << 0.0,0.0,0.0;
        p_w0 = Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        vel_0 = msg->twist.twist.linear.x;
        w_0 =msg->twist.twist.angular.z;
        start_flag = true;
    }

    else
    {        
        Quaterniond q_e0n1;
        q_e0n1 = q_ekw*Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
        vel_1 = msg->twist.twist.linear.x;
        w_1 =msg->twist.twist.angular.z;
        double un_vel = 0.5 * (vel_0 + vel_1);
        double un_w = 0.5 * (w_0+w_1);
        p_e01 = Vector3d(un_vel * detal_t * cos(0.5*un_w*detal_t), un_vel * detal_t * sin(0.5*un_w*detal_t),0);
        p_e += q_e0n0*p_e01;
        q_e0n0 = q_e0n1;
        vel_0 = vel_1;
        w_0 = w_1;
        iter_count++;
        if(iter_count==20)
        {
            Vector3d p_w1,p_e_true,detal_p;
            p_w1 = Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
            p_e_true = q_ekw*(p_w1 - p_w0);
            detal_p = p_e_true - p_e;
            ROS_INFO("The p_e_true is:%f %f %f",p_e_true.x(),p_e_true.y(),p_e_true.z());
            ROS_INFO("The p_e is:%f %f %f;the error is:%f",p_e.x(),p_e.y(),p_e.z(),detal_p.norm());
            q_ekw = Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z).inverse();
            q_e0n0 = Quaterniond(1.0,0.0,0.0,0.0);
            p_e << 0.0,0.0,0.0;
            p_w0 = p_w1;
            iter_count = 0;
        }

    }

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"imu_odom_node");
    ros::NodeHandle n;
    ros::Subscriber sub0=n.subscribe("/imu",1000,imu_vel_compute);
    ros::Subscriber sub1=n.subscribe("/odom",1000,odom_post_compute);
    ros::spin();
    return 0;
}
