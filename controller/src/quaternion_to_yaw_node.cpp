#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class QuaternionToYaw
{
public:
    QuaternionToYaw()
    {
        sub_ = nh_.subscribe("/odom", 1, &QuaternionToYaw::odomCallback, this);
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        ROS_INFO("Yaw: %.2f", yaw);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quaternion_to_yaw_node");
    QuaternionToYaw qToYaw;
    ros::spin();
    return 0;
}