#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/GoalSet.h>
#include <quadrotor_msgs/TakeOffLandToBridge.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <traj_utils/MINCOTraj.h>
#include <traj_utils/PolyTraj.h>
#include <unistd.h>

#include <iostream>

#include "ground_station_msgs/Task.h"
#include "reliable_bridge.hpp"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "swarm_bridge/battary_velocity.h"
#include "swarm_bridge/correction.h"
#include "swarm_bridge/gps.h"
#include "swarm_bridge/heartbeat.h"
using namespace std;

std::vector<int> id_list_;
std::vector<string> ip_list_;
ros::Subscriber other_odoms_sub_, one_traj_sub_, joystick_sub_, goal_sub_,
    object_odoms_sub_, takeoff_sub;
ros::Publisher other_odoms_pub_, one_traj_pub_, joystick_pub_, goal_pub_,
    object_odoms_pub_, takeoff_pub, task_change_pub, takeoff_bridge_pub,
    diff_pose_pub, heartbeat_pub, gps_mean_pub;
ros::Subscriber goal_exploration_sub_, star_cvx_sub_, frontier_sub_,
    task_change_sub, diff_pos_sub, batteryState_sub_, gps_sub;
ros::Publisher goal_exploration_pub_, star_cvx_pub_, frontier_pub_,
    batteryState_pub_, polytraj_pub;
int self_id_;
int self_id_in_bridge_;
int drone_num_;
int ground_station_num_;
double odom_broadcast_freq_;
bool is_groundstation_;

unique_ptr<ReliableBridge> bridge;

inline int remap_ground_station_id(int id) { return id + drone_num_; }

template <typename T>
int send_to_all_drone_except_me(string topic, T &msg) {
    int err_code = 0;
    for (int i = 0; i < drone_num_; ++i)  // Only send to all drones.
    {
        if (i == self_id_in_bridge_)  // skip myself
        {
            continue;
        }
        err_code += bridge->send_msg_to_one(i, topic, msg);
        if (err_code < 0) {
            ROS_ERROR("[Bridge] SEND ERROR %s !!", typeid(T).name());
        }
    }
    return err_code;
}

template <typename T>
int send_to_all_groundstation_except_me(string topic, T &msg) {
    int err_code = 0;
    for (int i = 0; i < ground_station_num_;
         ++i)  // Only send to all groundstations.
    {
        int ind = remap_ground_station_id(i);
        if (ind == self_id_in_bridge_)  // skip myself
        {
            continue;
        }
        err_code += bridge->send_msg_to_one(ind, topic, msg);
        if (err_code < 0) {
            ROS_ERROR("[Bridge] SEND ERROR %s !!", typeid(T).name());
        }
    }
    return err_code;
}

void register_callbak_to_all_groundstation(
    string topic_name, function<void(int, ros::SerializedMessage &)> callback) {
    for (int i = 0; i < ground_station_num_; ++i) {
        int ind = remap_ground_station_id(i);
        if (ind == self_id_in_bridge_)  // skip myself
        {
            continue;
        }
        bridge->register_callback(ind, topic_name, callback);
    }
}

void register_callbak_to_all_drones(
    string topic_name, function<void(int, ros::SerializedMessage &)> callback) {
    for (int i = 0; i < drone_num_; ++i) {
        if (i == self_id_in_bridge_)  // skip myself
        {
            continue;
        }
        bridge->register_callback(i, topic_name, callback);
    }
}

// Here is callback from local topic.
void odom_sub_cb(const nav_msgs::OdometryPtr &msg) {
    static ros::Time t_last;
    ros::Time t_now = ros::Time::now();
    if ((t_now - t_last).toSec() * odom_broadcast_freq_ < 1.0) {
        return;
    }
    t_last = t_now;

    msg->child_frame_id = string("drone_") + std::to_string(self_id_);

    other_odoms_pub_.publish(msg);               // send to myself
    send_to_all_drone_except_me("/odom", *msg);  // Only send to drones.
    send_to_all_groundstation_except_me("/odom",
                                        *msg);  // Only send to ground stations.
}

void object_odom_sub_udp_cb(const nav_msgs::OdometryPtr &msg) {
    msg->child_frame_id = string("obj_") + std::to_string(self_id_);

    object_odoms_pub_.publish(msg);                     // send to myself
    send_to_all_drone_except_me("/object_odom", *msg);  // Only send to drones.
    send_to_all_groundstation_except_me("/object_odom",
                                        *msg);  // Only send to ground stations.
}

void one_traj_sub_cb(const traj_utils::MINCOTrajPtr &msg) {
    one_traj_pub_.publish(msg);  // Send to myself.
    if (bridge->send_msg_to_all("/traj_from_planner", *msg)) {
        ROS_ERROR("[Bridge] SEND ERROR (ONE_TRAJ)!!!");
    }
}

void joystick_sub_cb(const sensor_msgs::JoyPtr &msg) {
    joystick_pub_.publish(msg);  // Send to myself.
    send_to_all_drone_except_me("/joystick", *msg);
}

void goal_sub_cb(const quadrotor_msgs::GoalSetPtr &msg) {
    ROS_INFO("goal user_to_bridge drone_id: %d, self_id: %d", msg->drone_id,
             self_id_in_bridge_);
    if (msg->drone_id == self_id_in_bridge_) {
        goal_pub_.publish(msg);  // Send to myself.
        ROS_INFO("send goal to myself");
        return;
    }
    if (send_to_all_drone_except_me("/goal", *msg) < 0) {
        ROS_ERROR("[Bridge] SEND ERROR (GOAL)!!!");
    }
    ROS_INFO("send success");
}

void takeoff_land_sub_cb(
    const quadrotor_msgs::TakeOffLandToBridge::ConstPtr &msg) {
    if (msg->takeoff_land_cmd == 1) {
        // ROS_INFO("takeoff flag to bridge drone_id [%d]", msg->drone_id);
    } else {
        // ROS_INFO("land flag to bridge drone_id [%d]", msg->drone_id);
    }
    // // quadrotor_msgs::TakeoffLand msg_bridge2planner;
    // msg_bridge2planner.takeoff_land_cmd = msg->takeoff_land_cmd;
    if (msg->drone_id == 255) {
        if (send_to_all_drone_except_me("/takeOffLandFlag", *msg) < 0) {
            ROS_ERROR("send cmd error");
        }
        return;
    }
    if (bridge->send_msg_to_one(msg->drone_id, "/takeOffLandFlag", *msg) < 0) {
        ROS_ERROR("[Bridge] SEND TAKEOFF/LAND_FLAG FAILED!!!");
    }
    quadrotor_msgs::TakeoffLand takeoff_land_msg;
    takeoff_land_msg.takeoff_land_cmd = msg->takeoff_land_cmd;
    if (msg->drone_id == self_id_) {
        takeoff_pub.publish(takeoff_land_msg);
    }
    // ROS_INFO("send success");
}

void task_change_sub_cb(const ground_station_msgs::Task::ConstPtr &msg) {
    if (send_to_all_drone_except_me("/task_change", *msg) < 0) {
        ROS_ERROR("[Bridge] task_change send error ");
    } else {
        ROS_INFO("[BRIDGE] task change send success");
    }
}

void battery_state_msg_cb(const sensor_msgs::BatteryState::ConstPtr &msg) {
    swarm_bridge::battary_velocity battery_velocity;
    battery_velocity.velocity = msg->voltage;
    battery_velocity.drone_id = static_cast<uint8_t>(self_id_);
    if (send_to_all_groundstation_except_me("/battary_velocity",
                                            battery_velocity) < 0) {
        ROS_ERROR("[BRIDGE] battery_velocity send error ");
    } else {
        // ROS_INFO("[BRIDGE] battery_velocity send success");
    }
}

void traj_msg_cb(const traj_utils::PolyTraj::ConstPtr &msg) {
    if (send_to_all_groundstation_except_me("/traj", *msg) < 0) {
        ROS_ERROR("[Bridge] traj send error ");
    } else {
        // ROS_INFO("[BRIDGE] traj send success");
    }
}
void heartbeat_msg_cb(const std_msgs::Empty::ConstPtr &msg) {
    static ros::Time t_last;
    ros::Time t_now = ros::Time::now();
    if((t_now - t_last).toSec() < 0.2) return;
    t_last = t_now;
    if (send_to_all_groundstation_except_me("/heart_beat", *msg) < 0) {
        ROS_ERROR("[Bridge] heart_beat send error ");
    } else {
        // ROS_INFO("[BRIDGE] heart_beat send success");
    }
}
void gps_msg_cb(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    static ros::Time t_last;
    ros::Time t_now = ros::Time::now();
    if((t_now - t_last).toSec() < 0.2) return;
    t_last = t_now;
    if (send_to_all_groundstation_except_me("/gps", *msg) < 0) {
        ROS_ERROR("[Bridge] gps send error ");
    } else {
        // ROS_INFO("[BRIDGE] gps send success");
    }
}

void task5_origin_pos_msg_cb(const nav_msgs::Odometry::ConstPtr &msg){
    static ros::Time t_last;
    double duration = (ros::Time::now() - t_last).toSec();
    if(duration < 0.25) return; 
    t_last = ros::Time::now();
    if (send_to_all_groundstation_except_me("/task5_origin_pos", *msg) < 0) {
        ROS_ERROR("[Bridge] gps_local send error ");
    } else {
        // ROS_INFO("[BRIDGE] gps send success");
    }
}
void task_change_cb(int ID, ros::SerializedMessage &m) {
    ground_station_msgs::Task task_change;
    ros::serialization::deserializeMessage(m, task_change);
    task_change_pub.publish(task_change);
}

void gps_mean_bridge_cb(int ID, ros::SerializedMessage &m) {
    swarm_bridge::gps msg;
    ros::serialization::deserializeMessage(m, msg);
    gps_mean_pub.publish(msg);
}
void goal_exploration_sub_cb(const quadrotor_msgs::GoalSetPtr &msg) {
    if (bridge->send_msg_to_all("/goal_exploration", *msg)) {
        ROS_ERROR("[Bridge] SEND ERROR (goal_exploration)!!!");
    }
}

void star_cvx_sub_cb(const sensor_msgs::PointCloud2Ptr &msg) {
    if (bridge->send_msg_to_all("/star_cvx", *msg)) {
        ROS_ERROR("[Bridge] SEND ERROR (star_cvx)!!!");
    }
}

void frontier_sub_cb(const sensor_msgs::PointCloud2Ptr &msg) {
    if (bridge->send_msg_to_all("/frontier", *msg)) {
        ROS_ERROR("[Bridge] SEND ERROR (frontier)!!!");
    }
}

// Here is callback when the brodge received the data from others.
void odom_bridge_cb(int ID, ros::SerializedMessage &m) {
    nav_msgs::Odometry odom_msg_;
    ros::serialization::deserializeMessage(m, odom_msg_);
    other_odoms_pub_.publish(odom_msg_);
}

void object_odom_bridge_cb(int ID, ros::SerializedMessage &m) {
    nav_msgs::Odometry object_odom_msg_;
    ros::serialization::deserializeMessage(m, object_odom_msg_);
    object_odoms_pub_.publish(object_odom_msg_);
}

void goal_bridge_cb(int ID, ros::SerializedMessage &m) {
    quadrotor_msgs::GoalSet goal_msg_;
    ros::serialization::deserializeMessage(m, goal_msg_);
    goal_pub_.publish(goal_msg_);
}
void takeoff_land_bridge_cb(int ID, ros::SerializedMessage &m) {
    quadrotor_msgs::TakeOffLandToBridge takeoff_land_msg_;
    ros::serialization::deserializeMessage(m, takeoff_land_msg_);
    quadrotor_msgs::TakeoffLand takeoff_land_cmd;
    takeoff_land_cmd.takeoff_land_cmd = takeoff_land_msg_.takeoff_land_cmd;
    if (takeoff_land_msg_.drone_id == 255) {
        takeoff_pub.publish(takeoff_land_cmd);
        // ROS_INFO("takeoff_land_cmd pubed");
        if (self_id_ != drone_num_ - 1) {
            for (int i = self_id_ + 1; i < drone_num_; i++) {
                quadrotor_msgs::TakeOffLandToBridge takeoff_land_msg_tmp;
                takeoff_land_msg_tmp.drone_id = i;
                takeoff_land_msg_tmp.takeoff_land_cmd =
                    takeoff_land_msg_.takeoff_land_cmd;
                takeoff_bridge_pub.publish(takeoff_land_msg_tmp);
            }
        }
    } else if (takeoff_land_msg_.drone_id == self_id_) {
        takeoff_pub.publish(takeoff_land_cmd);
    } else {
        takeoff_bridge_pub.publish(takeoff_land_msg_);
    }
}
void traj_bridge_cb(int ID, ros::SerializedMessage &m) {
    traj_utils::MINCOTraj MINCOTraj_msg_;
    ros::serialization::deserializeMessage(m, MINCOTraj_msg_);
    one_traj_pub_.publish(MINCOTraj_msg_);
}

void joystick_bridge_cb(int ID, ros::SerializedMessage &m) {
    sensor_msgs::Joy joystick_msg_;
    ros::serialization::deserializeMessage(m, joystick_msg_);
    joystick_pub_.publish(joystick_msg_);
}

void goal_exploration_bridge_cb(int ID, ros::SerializedMessage &m) {
    quadrotor_msgs::GoalSet goal_msg_;
    ros::serialization::deserializeMessage(m, goal_msg_);
    goal_exploration_pub_.publish(goal_msg_);
}

void star_cvx_bridge_cb(int ID, ros::SerializedMessage &m) {
    sensor_msgs::PointCloud2 point_msg_;
    ros::serialization::deserializeMessage(m, point_msg_);
    star_cvx_pub_.publish(point_msg_);
}

void frontier_bridge_cb(int ID, ros::SerializedMessage &m) {
    sensor_msgs::PointCloud2 point_msg_;
    ros::serialization::deserializeMessage(m, point_msg_);
    frontier_pub_.publish(point_msg_);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "swarm_bridge");
    ros::NodeHandle nh("~");

    // nh.param("broadcast_ip", udp_ip_, string("127.0.0.255"));
    nh.param("self_id", self_id_, -1);
    nh.param("is_ground_station", is_groundstation_, false);
    nh.param("drone_num", drone_num_, 0);
    nh.param("ground_station_num", ground_station_num_, 0);
    nh.param("odom_max_freq", odom_broadcast_freq_, 1000.0);
    id_list_.resize(drone_num_ + ground_station_num_);
    ip_list_.resize(drone_num_ + ground_station_num_);
    for (int i = 0; i < drone_num_ + ground_station_num_; ++i) {
        nh.param(
            (i < drone_num_ ? "drone_ip_" + to_string(i)
                            : "ground_station_ip_" + to_string(i - drone_num_)),
            ip_list_[i], string("127.0.0.1"));
        id_list_[i] = i;
    }
    self_id_in_bridge_ = self_id_;
    if (is_groundstation_) {
        self_id_in_bridge_ = remap_ground_station_id(self_id_);
    }
    // the ground statation ID = self ID + drone_num_

    if (self_id_in_bridge_ < 0 || self_id_in_bridge_ > 99) {
        ROS_WARN("[swarm bridge] Wrong self_id!");
        exit(EXIT_FAILURE);
    }

    // initalize the bridge
    bridge.reset(
        new ReliableBridge(self_id_in_bridge_, ip_list_, id_list_, 100000));

    other_odoms_sub_ = nh.subscribe("my_odom", 10, odom_sub_cb,
                                    ros::TransportHints().tcpNoDelay());
    other_odoms_pub_ = nh.advertise<nav_msgs::Odometry>("/others_odom", 10);
    // register callback
    register_callbak_to_all_drones("/odom", odom_bridge_cb);

    object_odoms_sub_ =
        nh.subscribe("/object_odom_dtc2brig", 10, object_odom_sub_udp_cb,
                     ros::TransportHints().tcpNoDelay());
    object_odoms_pub_ =
        nh.advertise<nav_msgs::Odometry>("/object_odom_brig2plner", 10);
    // register callback
    // register_callbak_to_all_drones("/object_odom", object_odom_bridge_cb);
    takeoff_bridge_pub = nh.advertise<quadrotor_msgs::TakeOffLandToBridge>(
        "/takeoff_land_user2brig", 100);
    one_traj_sub_ =
        nh.subscribe("/broadcast_traj_from_planner", 100, one_traj_sub_cb,
                     ros::TransportHints().tcpNoDelay());
    one_traj_pub_ =
        nh.advertise<traj_utils::MINCOTraj>("/broadcast_traj_to_planner", 100);
    polytraj_pub = nh.advertise<traj_utils::PolyTraj>("/poly_traj", 100);
    bridge->register_callback_for_all("/traj_from_planner", traj_bridge_cb);

    joystick_sub_ = nh.subscribe("/joystick_from_users", 100, joystick_sub_cb,
                                 ros::TransportHints().tcpNoDelay());
    joystick_pub_ =
        nh.advertise<sensor_msgs::Joy>("/joystick_from_bridge", 100);
    register_callbak_to_all_groundstation("/joystick", joystick_bridge_cb);

    goal_sub_ = nh.subscribe("/goal_user2brig", 100, goal_sub_cb,
                             ros::TransportHints().tcpNoDelay());
    goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_brig2plner", 100);
    register_callbak_to_all_groundstation("/goal", goal_bridge_cb);
    register_callbak_to_all_drones("/goal", goal_bridge_cb);
    takeoff_sub =
        nh.subscribe("/takeoff_land_user2brig", 100, takeoff_land_sub_cb);
    takeoff_pub = nh.advertise<quadrotor_msgs::TakeoffLand>(
        "/takeoffLand_bridge2planner", 100);
    register_callbak_to_all_groundstation("/takeOffLandFlag",
                                          takeoff_land_bridge_cb);
    register_callbak_to_all_drones("/takeOffLandFlag", takeoff_land_bridge_cb);
    task_change_sub =
        nh.subscribe("/taskChange_user2bridge", 100, task_change_sub_cb);
    task_change_pub = nh.advertise<ground_station_msgs::Task>("/taskChange", 100);
    // register_callbak_to_all_groundstation("/task_change", task_change_cb);
    // register_callbak_to_all_drones("/task_change", task_change_cb);

    // batteryState_pub_ =
    //     nh.advertise<swarm_bridge::battary_velocity>("/battary_velocity", 100);
    gps_mean_pub = nh.advertise<swarm_bridge::gps>("/gps_mean", 100);


    ros::Subscriber battery_state_sub = nh.subscribe<sensor_msgs::BatteryState>(
        "/mavros/battery", 100, battery_state_msg_cb);
    ros::Subscriber traj_sub = nh.subscribe<traj_utils::PolyTraj>(
        "/drone_" + to_string(self_id_) + "_planning/trajectory", 100,
        traj_msg_cb);

    ros::Subscriber heartbeat_sub = nh.subscribe<std_msgs::Empty>(
        "/drone_" + to_string(self_id_) + "_traj_server/heartbeat", 100,
        heartbeat_msg_cb);
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(
        "/mavros/global_position/raw/fix", 100, gps_msg_cb);
    register_callbak_to_all_groundstation("/gps_mean", gps_mean_bridge_cb);
    ros::Subscriber task5_origin_pos_sub = nh.subscribe<nav_msgs::Odometry>("/odom_add_bias", 10, task5_origin_pos_msg_cb, ros::TransportHints().tcpNoDelay());
    ros::spin();

    bridge->StopThread();

    return 0;
}
