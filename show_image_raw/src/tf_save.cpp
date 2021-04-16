//
// Created by yiluzhang on 1/6/21.
//
#include <fstream>
#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
double time_offset;

Eigen::Vector3d t_bl(0.12,0.0,0.0);
Eigen::Matrix3d R_bl;
int cout_cnt = 0;

void chatterCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    int msg_size = msg->transforms.size();
    for(int i = 0;i < msg_size; i++){
        if(msg->transforms[i].header.frame_id == "map" && msg->transforms[i].child_frame_id == "laser_link"){
            Eigen::Vector3d tmp_v(msg->transforms[i].transform.translation.x,msg->transforms[i].transform.translation.y,msg->transforms[i].transform.translation.z);
            Eigen::Quaterniond tmp_q;
	    tmp_q.x()=msg->transforms[i].transform.rotation.x;
            tmp_q.y()=msg->transforms[i].transform.rotation.y;
	    tmp_q.z()=msg->transforms[i].transform.rotation.z;
	    tmp_q.w()=msg->transforms[i].transform.rotation.w;
	    tmp_v = R_bl*tmp_v+t_bl;
            tmp_q=R_bl*tmp_q;
            std::ofstream f;
            f.open("/home/yiluzhang/Desktop/experiment/mini_3399/data/carto_tf_tum.txt", std::ios::app);
            f << std::fixed;
            f << std::setprecision(6) << msg->transforms[i].header.stamp.toSec() << std::setprecision(7) <<
              " " << tmp_v.x() <<
              " " << tmp_v.y() <<
              " " << tmp_v.z() <<
              " " << tmp_q.x() <<
              " " << tmp_q.y() <<
              " " << tmp_q.z() <<
              " " << tmp_q.w() << std::endl;
            /*f << std::setprecision(6) << msg->transforms[i].header.stamp.toSec() << std::setprecision(7) <<
              " " << msg->transforms[i].transform.translation.x <<
              " " << msg->transforms[i].transform.translation.y <<
              " " << msg->transforms[i].transform.translation.z <<
              " " << msg->transforms[i].transform.rotation.x <<
              " " << msg->transforms[i].transform.rotation.y <<
              " " << msg->transforms[i].transform.rotation.z <<
              " " << msg->transforms[i].transform.rotation.w << std::endl;*/
            f.close();
        }
	if(msg->transforms[i].header.frame_id == "odom" && msg->transforms[i].child_frame_id == "base_footprint"){
	    //Eigen::Vector3d tmp_v;
            Eigen::Quaterniond tmp_q;
	    tmp_q.x()=msg->transforms[i].transform.rotation.x;
            tmp_q.y()=msg->transforms[i].transform.rotation.y;
	    tmp_q.z()=msg->transforms[i].transform.rotation.z;
	    tmp_q.w()=msg->transforms[i].transform.rotation.w;
	    Eigen::Matrix3d tmp_r(tmp_q);
            std::ofstream f;
            f.open("/home/yiluzhang/Desktop/experiment/mini_3399/data/tf_odom_kitti.txt", std::ios::app);
            f << std::fixed;
            f << std::setprecision(6) << msg->transforms[i].header.stamp.toSec() << std::setprecision(7) <<
              " " << tmp_r(0,0) <<
              " " << tmp_r(0,1) <<
              " " << tmp_r(0,2) <<
              " " << msg->transforms[i].transform.translation.x <<
              " " << tmp_r(1,0) <<
              " " << tmp_r(1,1) <<
              " " << tmp_r(1,2) <<
              " " << msg->transforms[i].transform.translation.y <<
              " " << tmp_r(2,0) <<
              " " << tmp_r(2,1) <<
              " " << tmp_r(2,2) <<
              " " << msg->transforms[i].transform.translation.z << std::endl;
            f.close();

	    f.open("/home/yiluzhang/Desktop/experiment/mini_3399/data/tf_odom_tum.txt", std::ios::app);
            f << std::fixed;
            f << std::setprecision(6) << msg->transforms[i].header.stamp.toSec() << std::setprecision(7) <<
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
    f.open("/home/yiluzhang/Desktop/experiment/mini_3399/data/carto_tf_tum.txt");
    f.close();
    f.open("/home/yiluzhang/Desktop/experiment/mini_3399/data/tf_odom_kitti.txt");
    f.close();
    f.open("/home/yiluzhang/Desktop/experiment/mini_3399/data/tf_odom_tum.txt");
    f.close();
    
    double theta = 6.35*3.1415926536/180;
    //R_bl << 0.993392403509094, -0.114767297826624, 0.0, 0.114767297826624, 0.993392403509094, 0.0, 0.0, 0.0, 1.0;
    R_bl << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0.0, 0.0, 1.0;
    if(cout_cnt == 0){
    Eigen::Quaterniond tmp_q(R_bl);
    std::cout << "Q_bl: " << tmp_q.x() << " " <<  tmp_q.y() << " " <<  tmp_q.z() << " " <<  tmp_q.w() << std::endl;
    cout_cnt++;
    }

    ros::Subscriber sub = n.subscribe("tf", 1000, chatterCallback);

    ros::spin();

    return 0;
}
