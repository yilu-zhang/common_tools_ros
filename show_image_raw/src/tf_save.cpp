//
// Created by yiluzhang on 1/6/21.
//
#include <fstream>
#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
double time_offset;

void chatterCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    int msg_size = msg->transforms.size();
    for(int i = 0;i < msg_size; i++){
        if(msg->transforms[i].header.frame_id == "map" && msg->transforms[i].child_frame_id == "laser_link"){
            std::ofstream f;
            f.open("/home/yiluzhang/Desktop/experiment/carto/data/carto_tf_tum4.txt", std::ios::app);
            f << std::fixed;
            f << std::setprecision(6) << msg->transforms[i].header.stamp.toSec()-(10703473.977941-9901275.383173) << std::setprecision(7) <<
              " " << msg->transforms[i].transform.translation.x <<
              " " << msg->transforms[i].transform.translation.y <<
              " " << msg->transforms[i].transform.translation.z <<
              " " << msg->transforms[i].transform.rotation.x <<
              " " << msg->transforms[i].transform.rotation.y <<
              " " << msg->transforms[i].transform.rotation.z <<
              " " << msg->transforms[i].transform.rotation.w << std::endl;
            f.close();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_save");

    ros::NodeHandle n;
    std::ofstream f;
    f.open("/home/yiluzhang/Desktop/experiment/carto/data/carto_tf_tum4.txt");
    f.close();

    ros::Subscriber sub = n.subscribe("tf", 1000, chatterCallback);

    ros::spin();

    return 0;
}
