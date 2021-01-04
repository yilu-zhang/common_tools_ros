#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"

double last_t=0,last_t1=0;
int wrong_cnt=0;
void info0_show(const sensor_msgs::Image::ConstPtr &msg)
{
	std_msgs::Header head;
	head = msg->header;

	double now_t = head.stamp.toSec();
	if(now_t - last_t > 0.041)
	{
		wrong_cnt++;		
		ROS_WARN("wrong_seq:%d %d\n\n",msg->header.seq,wrong_cnt);	
	}
	last_t = now_t;

	ROS_INFO("image0_seq:%d",msg->header.seq);	
        ROS_INFO("image0_seconds:%f",head.stamp.toSec());        
}

void info1_show(const sensor_msgs::Image::ConstPtr &msg)
{
	double now_t = msg->header.stamp.toSec();
	if(now_t - last_t1 > 0.035)
	{
		wrong_cnt++;		
		ROS_WARN("wrong_seq:%d %d\n\n",msg->header.seq,wrong_cnt);	
	}
	last_t1 = now_t;

	ROS_INFO("image1_seq:%d",msg->header.seq);	
        ROS_INFO("image1_seconds:%f",msg->header.stamp.toSec());       
}

int main(int argc, char **argv)
{
        ros::init(argc,argv,"show_image_raw");
        ros::NodeHandle n;
        ros::Subscriber sub0=n.subscribe("/zkhy_stereo/left/color",10,info0_show);
	ros::Subscriber sub1=n.subscribe("/kinect2/qhd/image_color",10,info1_show);
        ros::spin();
        return 0;
}
