#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
ros::Publisher fake_height_puber;
void height_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    std_msgs::Float64 current_height_msg;
    current_height_msg.data = msg->pose.pose.position.z;
    fake_height_puber.publish(current_height_msg);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_height_puber");
    ros::NodeHandle nh("~");

    fake_height_puber = nh.advertise<std_msgs::Float64>("fake_height", 100);
    ros::Subscriber height_odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "height_odom", 100, height_odom_callback);
    ros::spin();
}