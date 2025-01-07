#ifndef _OCC_MAP_UWB_H
#define _OCC_MAP_UWB_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <iostream>
#include <memory>

#include <nlink_parser/LinktrackTagframe0.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <functional>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/subscriber.h>

using namespace std;

namespace odom_converter{

#define USE_EXACT_TIME_SYNC false

class odom_converter{
private:
    int capture_num_;

    nlink_parser::LinktrackTagframe0 uwb_data_;

public:

    void Odom_callback(const nav_msgs::OdometryConstPtr& odom_ptr);
    void Pose_callback(const geometry_msgs::PoseStampedConstPtr& pose_ptr);
    void Uwb_callback(const nlink_parser::LinktrackTagframe0ConstPtr &uwb_ptr);

    void Init(ros::NodeHandle & n);
    odom_converter(){
    }
    ~odom_converter(){
    }

protected:

    // typedef message_filters::sync_policies::ExactTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> SyncPolicyExact;
    // typedef unique_ptr<message_filters::Synchronizer<SyncPolicyExact>> SynchronizerExact;

    // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nlink_parser::LinktrackTagframe0> Odom_Uwb_SyncPolicyApproximate;
    // typedef message_filters::Synchronizer<Odom_Uwb_SyncPolicyApproximate> Odom_Uwb_SynchronizerApproximate;

    // typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, nlink_parser::LinktrackTagframe0> Pose_Uwb_SyncPolicyApproximate;
    // typedef message_filters::Synchronizer<Pose_Uwb_SyncPolicyApproximate> Pose_Uwb_SynchronizerApproximate;

    ros::Subscriber  pose_sub_, odom_sub_,uwb_sub_;




    // void pose_twist_callback(const geometry_msgs::PoseStampedConstPtr& pose_ptr, const geometry_msgs::TwistStampedConstPtr& twist_ptr);

    
    ros::Publisher pose_pub_;
    void registerSub(ros::NodeHandle &n);
    void registerPub(ros::NodeHandle &n);

};

}

#endif