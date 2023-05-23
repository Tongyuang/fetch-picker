/**
 * @ Author: Yuang Tong
 * @ Create Time: 2023-05-15 18:05:46
 * @ Modified by: Yuang Tong
 * @ Modified time: 2023-05-15 18:22:33
 * @ Description:
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace perception{
    class Downsampler {
        public:
            Downsampler(const ros::Publisher& pub);
            void Callback(const sensor_msgs::PointCloud2& msg);

        private:
            ros::Publisher pub_;
    };
}