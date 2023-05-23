/**
 * @ Author: Yuang Tong
 * @ Create Time: 2023-05-15 15:36:41
 * @ Modified by: Yuang Tong
 * @ Modified time: 2023-05-15 17:34:53
 * @ Description:
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace perception{
    class Cropper {
        public:
            Cropper(const ros::Publisher& pub);
            void Callback(const sensor_msgs::PointCloud2& msg);

        private:
            ros::Publisher pub_;
    };
}