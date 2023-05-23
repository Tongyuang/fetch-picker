/**
 * @ Author: Yuang Tong
 * @ Create Time: 2023-05-15 13:02:12
 * @ Modified by: Yuang Tong
 * @ Modified time: 2023-05-15 18:05:16
 * @ Description:
 */

#include <iostream>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "rosbag/bag.h"

sensor_msgs::PointCloud2ConstPtr Obtain_point_cloud(std::string PointTopic){
    sensor_msgs::PointCloud2ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(PointTopic);
    return cloud;
}

void print_usage() {
    std::cout << "Saves a point cloud on head_camera/depth_registered/points to "
    "Name.bag in the current directory"
    << std::endl;
    std::cout << "Usage: rosrun perception save_cloud [NAME]" << std::endl;
}

int main(int argc, char** argv){
    
    ros::init(argc,argv,"save_cloud_main");
    if (argc < 2){
        print_usage();
        return 1;
    }
    std::string PointTopic("head_camera/depth_registered/points");
    sensor_msgs::PointCloud2ConstPtr cloud = Obtain_point_cloud(PointTopic);
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("base_link",cloud->header.frame_id,ros::Time(0),ros::Duration(5.0));
    tf::StampedTransform transform;
    try {
        tf_listener.lookupTransform("base_link",cloud->header.frame_id,ros::Time(0),transform);
    } catch (tf::LookupException& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    } catch (tf::ExtrapolationException& e) {                                             
        std::cerr << e.what() << std::endl;                                                 
        return 1;                                                                           
    } 
    sensor_msgs::PointCloud2 cloud_out;
    pcl_ros::transformPointCloud("base_link",transform,*cloud,cloud_out);
    
    // save the bag
    std::string name(argv[1]);
    std::string filename(name + ".bag");
    rosbag::Bag bag;
    bag.open(filename,rosbag::bagmode::Write);
    bag.write(PointTopic,ros::Time::now(),cloud_out);
    bag.close();

    return 0;
}