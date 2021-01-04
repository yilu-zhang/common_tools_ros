#include "ros/ros.h"
#include "control_can_motor/Motor.h"

void info_show(const control_can_motor::Motor::ConstPtr& msg)
{
        ROS_INFO("motor_id:%d",msg->motor_id);
        ROS_INFO("current:%d",msg->current);
        ROS_INFO("velocity:%d",msg->velocity);
        ROS_INFO("position:%d\n",msg->position);
        ROS_INFO("acclX_scaled:%d\n",msg->acclX_scaled);
        ROS_INFO("acclY_scaled:%d\n",msg->acclY_scaled);
        ROS_INFO("acclZ_scaled:%d\n",msg->acclZ_scaled);
        ROS_INFO("gyroX_scaled:%d\n",msg->gyroX_scaled);
        ROS_INFO("gyroY_scaled:%d\n",msg->gyroY_scaled);
        ROS_INFO("gyroZ_scaled:%d\n",msg->gyroZ_scaled);
}

int main(int argc, char **argv)
{
        ros::init(argc,argv,"listener1");
        ros::NodeHandle n;
        ros::Subscriber sub=n.subscribe("motor_info",10,info_show);
        ros::spin();
        return 0;
}
