#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

double yaw_offset_; // x axis of mocap frame relate to x axis of enu frame. z axis is up
ros::Publisher enu_pub_;

void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Eigen::Quaterniond q;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    // transfer to Eigen
    q = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    R = q.toRotationMatrix();
    p = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // transfer to enu local frame  just add yaw offset
    Eigen::Matrix3d R_yaw;
    R_yaw << cos(yaw_offset_), -sin(yaw_offset_), 0,
        sin(yaw_offset_), cos(yaw_offset_), 0,
        0, 0, 1;

    p = R_yaw * p;
    R = R * R_yaw;

    //! if not use q
    q = Eigen::Quaterniond(1,0,0,0);


    // publish
    geometry_msgs::PoseStamped enu_pose;
    enu_pose.header = msg->header;
    enu_pose.pose.position.x = p(0);
    enu_pose.pose.position.y = p(1);
    enu_pose.pose.position.z = p(2);
    Eigen::Quaterniond q_enu(R);

    //! if not use q
    q_enu = Eigen::Quaterniond(1,0,0,0);
    enu_pose.pose.orientation.w = q_enu.w();
    enu_pose.pose.orientation.x = q_enu.x();
    enu_pose.pose.orientation.y = q_enu.y();
    enu_pose.pose.orientation.z = q_enu.z();
    enu_pub_.publish(enu_pose);
}

// main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mocap_to_enu");
    ros::NodeHandle nh("~");
    // parameters
    nh.param<double>("yaw_offset", yaw_offset_, 0.0);
    enu_pub_ = nh.advertise<geometry_msgs::PoseStamped>("enu_pose", 10);
    ros::Subscriber mocap_sub = nh.subscribe("vrpn_pose", 10, mocap_pose_cb);

    ros::spin();
    return 0;
}