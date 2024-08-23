#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>

class LidarAngleTester
{
public:
    LidarAngleTester()
    {
        sub_ = nh_.subscribe("/scan", 1, &LidarAngleTester::callback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    float laser_temp[361];
    float laser_temp_scan[897];

    float minimun(int start, int end, int& k)
    {
        float min_val = std::numeric_limits<float>::max();
        for (int i = start; i <= end; ++i)
        {
            if (laser_temp[i] < min_val && laser_temp[i] != 0)
            {
                min_val = laser_temp[i];
                k = i;
            }
        }
        return min_val;
    }

    void callback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        int k;
        // 初始化 laser_temp 和 laser_temp_scan
        std::fill_n(laser_temp, 361, 0.0f);
        std::fill_n(laser_temp_scan, 897, 0.0f);

        for(int i = 1; i <= scan->ranges.size(); i++)
        {
            laser_temp_scan[i] = scan->ranges[i-1]; // 注意：arrays是從0開始的
        }

        for (int i = 0; i <= scan->ranges.size(); ++i)
        {
			// int a = scan->ranges.size();
			// printf("%d",a);
            double angle_rad = (scan->angle_min + i * scan->angle_increment) + M_PI;
            int angle_deg = static_cast<int>(angle_rad * 180 / M_PI);
            if (!std::isinf(laser_temp_scan[i]) && laser_temp_scan[i] != 0)
            {
                laser_temp[angle_deg] = laser_temp_scan[i];
            }
        }

        

        // 設置輸出精度為小數點後幾位
        std::cout << std::fixed << std::setprecision(4);

        // 每5度輸出一次數值
        for (int angle = 90; angle <= 270; angle += 5)
        {
            std::cout << "角度 " << angle << "°: " << laser_temp[angle] << std::endl;
        }
        std::cout << "269 degree " << laser_temp[269] << std::endl;
        
        std::cout << "-----------------------------" << std::endl;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_angle_tester");
    LidarAngleTester tester;
    ros::spin();
    return 0;
}