//
// Created by yiluzhang on 1/6/21.
//
#include <fstream>
#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>

std::string save_dir_name = "/home/yiluzhang/admire/experiment/apriltag/accuracy/";
using namespace std;

void chatterCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    int msg_size = msg->transforms.size();
    for(int i = 0;i < msg_size; i++){
        if(msg->transforms[i].header.frame_id == "rgb_camera_link" && msg->transforms[i].child_frame_id.substr(0,4) == "tag_"){
            Eigen::Vector3d tmp_v(msg->transforms[i].transform.translation.x,msg->transforms[i].transform.translation.y,msg->transforms[i].transform.translation.z);
            Eigen::Quaterniond tmp_q;
	        tmp_q.x()=msg->transforms[i].transform.rotation.x;
            tmp_q.y()=msg->transforms[i].transform.rotation.y;
	        tmp_q.z()=msg->transforms[i].transform.rotation.z;
	        tmp_q.w()=msg->transforms[i].transform.rotation.w;

            tmp_q = tmp_q.inverse();
	        tmp_v = -(tmp_q*tmp_v);
	        Eigen::Matrix3d r_cl;
	        string link_name = msg->transforms[i].child_frame_id;
	        if(link_name == "tag_0" || link_name == "tag_6" || link_name == "tag_2"){
	            r_cl << 1.0, 0.0, 0.0,
	                    0.0, -1.0, 0.0,
	                    0.0, 0.0, -1.0;
	        }
	        else if(link_name == "tag_7"){
                r_cl << 0.0, 0.0, 1.0,
                        0.0, -1.0, 0.0,
                        1.0, 0.0, 0.0;
	        }

	        tmp_q = r_cl*tmp_q;
	        tmp_v = r_cl*tmp_v;
            Eigen::Matrix3d tmp_r(tmp_q);
            //ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw,0表示X轴,1表示Y轴,2表示Z轴
            Eigen::Vector3d tmp_euler = tmp_r.eulerAngles(1, 0, 2); //210
            /*if(tmp_euler.x() > 0.5*M_PI){
                tmp_euler.x() -= M_PI;
            }
            else if(tmp_euler.x() < -0.5*M_PI){
                tmp_euler.x() += M_PI;
            }

            if(tmp_euler.y() > 0.5*M_PI){
                tmp_euler.y() -= M_PI;
            }
            else if(tmp_euler.y() < -0.5*M_PI){
                tmp_euler.y() += M_PI;
            }

            if(tmp_euler.z() > 0.5*M_PI){
                tmp_euler.z() -= M_PI;
            }
            else if(tmp_euler.z() < -0.5*M_PI){
                tmp_euler.z() += M_PI;
            }*/

            std::ofstream f;
            f.open(save_dir_name+msg->transforms[i].child_frame_id + "_tum.txt", std::ios::app);
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

            //yaw(y) pitch(z) roll(z)
            std::cout << msg->transforms[i].child_frame_id << ": " << std::fixed << std::setprecision(6) << msg->transforms[i].header.stamp.toSec() << std::setprecision(7) <<
              " " << tmp_v.x() <<
              " " << tmp_v.y() <<
              " " << tmp_v.z() <<
              " " << tmp_euler.y() * 180/M_PI <<
              " " << tmp_euler.x() * 180/M_PI  <<
              " " << tmp_euler.z() * 180/M_PI  <<
              " " << acos(tmp_r(0,0)) * 180/M_PI  <<
                                               " " << acos(tmp_r(1,1)) * 180/M_PI  <<
                                               " " << acos(tmp_r(0,0)) * 180/M_PI  <<std::endl;
            f.open(save_dir_name+msg->transforms[i].child_frame_id + "_euler.txt", std::ios::app);
            f << std::fixed << std::setprecision(6) << msg->transforms[i].header.stamp.toSec() << std::setprecision(7) <<
                          " " << tmp_v.x() <<
                          " " << tmp_v.y() <<
                          " " << tmp_v.z() <<
                          " " << tmp_euler.y() * 180/M_PI <<
                          " " << tmp_euler.x() * 180/M_PI  <<
                          " " << tmp_euler.z() * 180/M_PI  <<
                                                           " " << acos(tmp_r(0,0)) * 180/M_PI  <<
                                                           " " << acos(tmp_r(1,1)) * 180/M_PI  <<
                                                           " " << acos(tmp_r(0,0)) * 180/M_PI  <<std::endl;
            f.close();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_save");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("tf", 100, chatterCallback);

    ros::spin();

    return 0;
}
