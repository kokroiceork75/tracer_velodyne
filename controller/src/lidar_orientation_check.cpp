#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <iostream>

class LidarOrientationCheck {
private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    std::vector<float> laser_temp_;
    
    void printOrientationDistances(const std::vector<float>& distances) {
        std::cout << "前方 (0度): " << distances[0] << " m" << std::endl;
        std::cout << "右方 (90度): " << distances[90] << " m" << std::endl;
        std::cout << "後方 (180度): " << distances[180] << " m" << std::endl;
        std::cout << "左方 (270度): " << distances[270] << " m" << std::endl;
    }

public:
    LidarOrientationCheck() : laser_temp_(360, 0.0) {
        laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &LidarOrientationCheck::laserCallback, this);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        std::vector<float> laser_temp_scan(scan->ranges.size());
        
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            laser_temp_scan[i] = scan->ranges[i];
        }

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            double angle_rad = scan->angle_min + i * scan->angle_increment + M_PI;
            int angle_deg = static_cast<int>((angle_rad * 180 / M_PI)) % 360;

            if (!std::isinf(laser_temp_scan[i]) && laser_temp_scan[i] != 0) {
                laser_temp_[angle_deg] = laser_temp_scan[i];
            }
        }

        printOrientationDistances(laser_temp_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_orientation_check");
    LidarOrientationCheck lidar_check;
    ros::spin();
    return 0;
}