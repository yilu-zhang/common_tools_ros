#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/format.hpp> 


bool save_flag = false;
int i=1;
void save_image(const sensor_msgs::Image::ConstPtr &msg)
{
        cv_bridge::CvImageConstPtr ptr;
        boost::format fmt( "/home/yiluzhang/experiment/pcl/astra/image/%d.%s" ); //图像文件格式
        if(save_flag==true)
        {
  
                ptr = cv_bridge::toCvShare(msg, "16UC1");
                cv::imwrite((fmt%i%"png").str(),ptr->image);
                i++;
                ROS_INFO("write_sucess!");
                save_flag = false;
        }

}

void setting_flag(const geometry_msgs::Twist::ConstPtr &msg)
{
        if(msg->linear.x == 2)
        {
                save_flag = true;
        }
	 
}

int main(int argc, char **argv)
{
        ros::init(argc,argv,"save_image");
        ros::NodeHandle n;
        ros::Subscriber sub0=n.subscribe("/camera/depth/image_raw",1,save_image);
	ros::Subscriber sub1=n.subscribe("/turtle1/cmd_vel",1,setting_flag);
        ros::spin();
        return 0;
}

	//std_msgs::Header head;
	//head = msg->header;
        

        //cv::Mat img(480,640,CV_16UC1);


             // unsigned short int x,y,z;
               //  for(int i=0;i<10;i++)
       // {
               //if(msg->data[i]>255)
                        //std::cout << msg->data[i] << " " << msg->data[480*640+10] <<" " << msg->data[480*640+20]<<std::endl;
                 //       x=msg->data[i];
                 //       std::cout << x << " ";
        //}
                
         //unsigned char y;
                //x=msg->data[480*640+10];
                //y=msg->data[480*640+20];
                //std::cout << x << " " << y <<std::endl;
                //img.at<uchar>(1,2)=msg->data[0];
                //int i=0;
                //for(;i<480*640;)
                //{
                        //img.at<uchar>(i/640,i%640)=msg->data[i];
                        //i=i+1;
                //}
                //x=msg->data[0];
                //y=msg->data[0];
                //ROS_INFO(msg->data[0].type());
                //ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
