/**
 * @ Author: Yuang Tong
 * @ Create Time: 2023-05-15 15:31:35
 * @ Modified by: Yuang Tong
 * @ Modified time: 2023-05-15 19:53:06
 * @ Description:
 */



#include "ros/ros.h"
#include "perception/crop.h"
#include "perception/downsample.h"
#include "sensor_msgs/PointCloud2.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_demo");
    ros::NodeHandle nh;
    ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    // ros::Publisher downsp_pub = 
    //   nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    perception::Cropper cropper(crop_pub);

    // perception::Downsampler downsampler(downsp_pub);
    ros::Subscriber sub = nh.subscribe("cloud_in",1,&perception::Cropper::Callback,&cropper);
    // ros::Subscriber sub = nh.subscribe("cloud_in",1,&perception::Downsampler::Callback,&downsampler);
    ros::spin();
    return 0;
}