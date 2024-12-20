#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nlink_parser/TofsenseFrame0.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float64.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
double current_height, roll, pitch, yaw;
double range_sensor_offset_;
ros::Publisher odom_fusion_pub;
ros::Publisher fusion_pose_pub;
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion imu_q =
        tf2::Quaternion(msg->orientation.x, msg->orientation.y,
                        msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3(imu_q).getRPY(roll, pitch, yaw);
}
void height_estimate_cb_nlink(
    const nlink_parser::TofsenseFrame0::ConstPtr& tof_msg) {
    current_height = tof_msg->dis * cos(roll) * cos(pitch);
}
void height_estimate_cb(const sensor_msgs::RangeConstPtr& rng) {
    if (rng->range == -1) {
        if (current_height < 0.5) {
            // ROS_ERROR("RNG SENSOR FIALED");
            current_height = 0.0;
        }
        return;
    }

    double height_with_coax_offset = rng->range + range_sensor_offset_;
    // ROS_INFO("range_sensor_offset_ : %.5f",range_sensor_offset_);
    // ROS_INFO("height_with_offset : %f",height_with_coax_offset);
    current_height = height_with_coax_offset * cos(roll) * cos(pitch);
    // current_height = rng->range * cos(roll) * cos(pitch);
}
void odom_raw_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry odom_fusion = *msg;
    odom_fusion.pose.pose.position.z = current_height;
    odom_fusion_pub.publish(odom_fusion);
    geometry_msgs::PoseStamped pose_fusion;
    pose_fusion.header = odom_fusion.header;
    pose_fusion.pose = odom_fusion.pose.pose;
    fusion_pose_pub.publish(pose_fusion);
    // ROS_INFO("[%f, %f, %f ]Roll: %f, Pitch: %f, Yaw: %f",
    //          odom_fusion.pose.pose.position.x,
    //          odom_fusion.pose.pose.position.y,
    //          odom_fusion.pose.pose.position.z, roll * 180 / 3.14,
    //          pitch * 180 / 3.14, yaw * 180 / 3.14);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "height_fusion_node");
    ros::NodeHandle nh("~");

    nh.param("/range_sensor_offset", range_sensor_offset_, 0.0);
    fusion_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/pose_height_fusion", 100);
    ros::Subscriber height_sub = nh.subscribe<sensor_msgs::Range>(
        "tof_measure", 100, height_estimate_cb);
    ros::Subscriber height_sub_nlink =
        nh.subscribe<nlink_parser::TofsenseFrame0>(
            "/nlink_tofsense_frame0", 100, height_estimate_cb_nlink);
    ros::Subscriber raw_odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom_raw", 100, odom_raw_cb);
    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, imu_cb);

    odom_fusion_pub = nh.advertise<nav_msgs::Odometry>("odom_fusion", 100);

    ros::spin();
}