#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"
#include <math.h>
#include <iostream>
#include <fstream>

using namespace std;

double last_t=0;
int wrong_cnt=0;
void info0_show(const sensor_msgs::Imu::ConstPtr &msg)
{
	std_msgs::Header head;
	head = msg->header;

	double now_t = head.stamp.toSec();
	if(now_t - last_t > 0.012)
	{
		wrong_cnt++;		
		ROS_WARN("wrong_seq:%d %d\n\n",msg->header.seq,wrong_cnt);	
	}
	last_t = now_t;
	if(fabs(msg->angular_velocity.x)>1||fabs(msg->angular_velocity.y>1)||fabs(msg->angular_velocity.z)>1||
	fabs(msg->linear_acceleration.x)>3||fabs(msg->linear_acceleration.y>3)||msg->linear_acceleration.z>14||msg->linear_acceleration.z<8)
	{
		ROS_WARN("imu_seq:%d",msg->header.seq);	
        	ROS_WARN("imu_seconds:%f",head.stamp.toSec());
		ROS_WARN("imu_ang_vel:%f %f %f",msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
		ROS_WARN("imu_linear_acc:%f %f %f",msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z); 
	}
	else{
		ROS_INFO("imu_seq:%d",msg->header.seq);	
        	ROS_INFO("imu_seconds:%f",head.stamp.toSec());
		ROS_INFO("imu_ang_vel:%f %f %f",msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
		ROS_INFO("imu_linear_acc:%f %f %f",msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z); 
	}
	ofstream imu_file;
	//imu_file.open("/home/yiluzhang/experiment/vins/camera/kinect2/paper/lab_rectangle/imu_data.txt", ios::app);
	imu_file.open("/home/yiluzhang/experiment/vins/camera/kinect2/paper/lab_rectangle/vins_imu_data_40_140.txt", ios::app);
	imu_file<<fixed<<setprecision(6)<<head.stamp.toSec()<<" "<<msg->angular_velocity.x<<" "<<msg->angular_velocity.y<<" "<<msg->angular_velocity.z<<" "
				    <<msg->linear_acceleration.x<<" "<<msg->linear_acceleration.y<<" "<<msg->linear_acceleration.z<<endl;
	imu_file.close();       
}


int main(int argc, char **argv)
{
        ros::init(argc,argv,"show_imu");
        ros::NodeHandle n;
        //ros::Subscriber sub0=n.subscribe("/loui_robot1/imu",1000,info0_show);
	ros::Subscriber sub0=n.subscribe("/imu0",1000,info0_show);
        ros::spin();
        return 0;
}
