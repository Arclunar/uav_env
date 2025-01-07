#include <nav_msgs/Odometry.h>
#include <nlink_parser/LinktrackTagframe0.h>
#include <odom_converter_uwb.h>

namespace odom_converter {

void odom_converter::Init(ros::NodeHandle &n) {

  registerSub(n);
  registerPub(n);
}

void odom_converter::registerSub(ros::NodeHandle &n) {

  pose_sub_ = n.subscribe("pose", 100, &odom_converter::Pose_callback, this,
                          ros::TransportHints().tcpNoDelay(true));

  odom_sub_ = n.subscribe("odom", 100, &odom_converter::Odom_callback, this,
                          ros::TransportHints().tcpNoDelay(true));
  //   std::make_unique<message_filters::Subscriber<nav_msgs::Odometry>>(
  //       n, "odom", 100, ros::TransportHints().tcpNoDelay(true));
  uwb_sub_ = n.subscribe("uwb", 100, &odom_converter::Uwb_callback, this,
                         ros::TransportHints().tcpNoDelay(true));
  //
}
void odom_converter::Odom_callback(const nav_msgs::OdometryConstPtr &odom_ptr) {

  geometry_msgs::PoseStamped pose;
  pose.header = odom_ptr->header;

  pose.pose.position.x = uwb_data_.pos_3d[0];
  pose.pose.position.y = uwb_data_.pos_3d[1];
  pose.pose.position.z = uwb_data_.pos_3d[2];
  pose.pose.orientation = odom_ptr->pose.pose.orientation;

  pose_pub_.publish(pose);
}

void odom_converter::Pose_callback(
    const geometry_msgs::PoseStampedConstPtr &pose_ptr) {
  geometry_msgs::PoseStamped pose;
  pose.header = pose_ptr->header;

  pose.pose.position.x = uwb_data_.pos_3d[0];
  pose.pose.position.y = uwb_data_.pos_3d[1];
  pose.pose.position.z = uwb_data_.pos_3d[2];
  pose.pose.orientation = pose_ptr->pose.orientation;

  pose_pub_.publish(pose);
}

void odom_converter::Uwb_callback(
    const nlink_parser::LinktrackTagframe0ConstPtr &uwb_ptr) {
    uwb_data_ = *uwb_ptr;
}

void odom_converter::registerPub(ros::NodeHandle &n) {
  //   odom_pub_.resize(capture_num_);
  //   for (int i = 0; i < capture_num_; i++) {
  //     string name = "converted_odom" + std::to_string(i);
  //     odom_pub_[i] = n.advertise<nav_msgs::Odometry>(name, 10);
  //   }
  pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("converted_pose", 10);
}

} // namespace odom_converter