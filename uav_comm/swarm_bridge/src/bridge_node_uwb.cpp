#include <quadrotor_msgs/GoalSet.h>
#include <quadrotor_msgs/TakeOffLandToBridge.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <std_msgs/Empty.h>
#include <swarm_bridge/MiniMINCO.h>
#include <traj_utils/MINCOTraj.h>

#include "bridge_utils.hpp"
#include "nav_msgs/Odometry.h"
#include "nlink_parser/LinktrackNodeframe0.h"
#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "swarm_bridge/DroneState.h"
#include "swarm_bridge/gps.h"
#include "traj_utils/PolyTraj.h"
#include <geometry_msgs/PoseStamped.h>
#include <mission_msgs/Mission.h>
#include <mission_msgs/ObjectObservation.h>
#include <mission_msgs/TrackTargetState.h>
#include <mission_msgs/TrackTargetSet.h>
#include <std_msgs/Int8.h>

/*
    每个数据帧不得超过0.1KB，一个原生的odom是0.7kb，所以需要适当压缩
    UWB节点数配置为5则所有发送数据的频率不得超过200Hz
    UWB节点数配置为10则所有发送数据的频率不得超过100Hz

    测距频率和通信频率无关，每帧数据量减小也不会提高频率，所以可以将几个话题合并，比如电池里程计等合并为drone_state
    TODO: 实现一个队列管理要发送的数据，稳定通信频率
*/
enum MESSAGE_TYPE {
    DRONE_STATE = 100,  // 时间戳、里程计、电池电量、心跳
    ONE_TRAJ,
    GOAL,
    TAKEOFF_LAND,
    ODOM_ORIGIN,  // UWB修正前的里程计数据，不走数传，直接用uwb和地面站通信
    POLY_TRAJ,    // 多项式轨迹，地面站接收轨迹可视化用
    GPS_ORIGIN,  // 经度和维度，地面站发给飞机的，将这个经纬度设置为坐标原点
    TRIG,        // Trigger 触发
    STOP,       // Mandotory stop 强制悬停
    MS,       // Mission 任务切换
    OB_OV,     // Object Observation 发送的飞机ID、被跟踪物体的ID、被跟踪物体odom
    TR_ST,      // Track Target State 跟踪是否ready
    TR_ID,     // Track Target ID 追踪目标ID设置
};
#define BUF_LEN 1048576  // 1MB, 实测只能发0.1KB
std::mutex send_mtx, rec_mtx, drone_state_mtx;
char uwb_recv_buf_[BUF_LEN], uwb_send_buf_[BUF_LEN];
ros::Publisher uwb_pub, other_odoms_pub_, minco_traj_pub_, takeoff_bridge_pub,
    takeoff_pub, polytraj_pub, goal_pub_, gps_origin_pub, minco_debug_pub_, 
    miniminco_debug_pub_, takeoff_land_from_bridge_pub_, trigger_pub_ , mandatory_stop_pub_;

ros::Publisher other_object_observation_pub_,mission_pub_, tracktargetstate_pub_, track_target_id_pub_;

swarm_bridge::DroneState drone_state_;
ros::Time last_heartbeat_stamp_;
unique_ptr<DataCompression> compresser_;
int self_id_, drone_num_;
template <typename T>
int serializeTopic(const MESSAGE_TYPE msg_type, const T &msg) {
    auto ptr = (uint8_t *)(uwb_send_buf_);

    *((MESSAGE_TYPE *)ptr) = msg_type;
    ptr += sizeof(MESSAGE_TYPE);

    namespace ser = ros::serialization;
    uint32_t msg_size = ser::serializationLength(msg);

    *((uint32_t *)ptr) = msg_size;
    ptr += sizeof(uint32_t);

    ser::OStream stream(ptr, msg_size);
    ser::serialize(stream, msg);

    return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}
template <typename T>
int deserializeTopic(T &msg) {
    auto ptr = (uint8_t *)(uwb_recv_buf_ + sizeof(MESSAGE_TYPE));

    uint32_t msg_size = *((uint32_t *)ptr);
    ptr += sizeof(uint32_t);

    namespace ser = ros::serialization;
    ser::IStream stream(ptr, msg_size);
    ser::deserialize(stream, msg);

    return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}
template <typename T>
bool uwbSend(const MESSAGE_TYPE msg_type, T &msg) {
    static ros::Time last_sent_stamp;
    if ((ros::Time::now() - last_sent_stamp).toSec() < 0.01) {
        ROS_ERROR("[msgType:%d] Your data freq is [%f]Hz, 100Hz max", msg_type,
                  (float)(1 / (ros::Time::now() - last_sent_stamp).toSec()));
        send_mtx.unlock();
        return false;
    }

    send_mtx.lock();
    int len = serializeTopic(msg_type, msg);
    if (len > 100) {
        ROS_ERROR("[msgType:%d] Your data size is %d B, 100B max", msg_type,
                  len);

        send_mtx.unlock();
        return false;
    }
    std::string uwb_msg(uwb_send_buf_, len);
    std_msgs::String msg2send;
    msg2send.data = std::string(uwb_send_buf_, len);
    uwb_pub.publish(msg2send);
    send_mtx.unlock();

    last_sent_stamp = ros::Time::now();
    return true;
}
void uwb_receiver_cb(const nlink_parser::LinktrackNodeframe0::ConstPtr &msg) {
    for (auto node : msg->nodes) {
        auto data = node.data;

        rec_mtx.lock();
        std::memcpy(uwb_recv_buf_, data.data(), data.size());
        switch (*((MESSAGE_TYPE *)uwb_recv_buf_)) {
            case MESSAGE_TYPE::DRONE_STATE: {
                // 飞机之间uwb通信：[里程计_接收]、轨迹
                swarm_bridge::DroneState state_msg_;
                deserializeTopic(state_msg_);
                nav_msgs::Odometry odom_full;
                odom_full.header.frame_id = "world";
                odom_full.child_frame_id = "drone_" + std::to_string(node.id);
                odom_full.header.stamp = state_msg_.odom_stamp;
                odom_full.pose.pose.position.x = state_msg_.x;
                odom_full.pose.pose.position.y = state_msg_.y;
                odom_full.pose.pose.position.z = state_msg_.z;
                odom_full.pose.pose.orientation.x = state_msg_.qx;
                odom_full.pose.pose.orientation.y = state_msg_.qy;
                odom_full.pose.pose.orientation.z = state_msg_.qz;
                odom_full.pose.pose.orientation.w = state_msg_.qw;
                other_odoms_pub_.publish(odom_full);
                break;
            }
            case MESSAGE_TYPE::TAKEOFF_LAND: {
                quadrotor_msgs::TakeOffLandToBridge takeoff_land_cmd_from_bridge;
                deserializeTopic(takeoff_land_cmd_from_bridge);

                // send back to sender
                // quadrotor_msgs::TakeOffLandToBridge cmd_from_bridge = takeoff_land_cmd_from_bridge;
                // cmd_from_bridge.drone_id = node.id;
                // takeoff_land_from_bridge_pub_.publish(cmd_from_bridge);

                if(self_id_ == takeoff_land_cmd_from_bridge.drone_id || takeoff_land_cmd_from_bridge.drone_id == 255){
                    quadrotor_msgs::TakeoffLand takeoff_land_cmd_local;
                    takeoff_land_cmd_local.takeoff_land_cmd =
                    takeoff_land_cmd_from_bridge.takeoff_land_cmd;
                    takeoff_pub.publish(takeoff_land_cmd_local);
                }
                if (takeoff_land_cmd_from_bridge.takeoff_land_cmd == 1) {
                    if(takeoff_land_cmd_from_bridge.drone_id == 255){
                        ROS_INFO("TAKEOFF_CMD FOR ALL DRONES RECEIVED");
                    }
                    else if(takeoff_land_cmd_from_bridge.drone_id == self_id_){
                        ROS_INFO("TAKEOFF_CMD FOR ME RECEIVED");
                    }
                    else{
                        ROS_INFO("TAKEOFF_CMD FOR DRONE %d RECEIVED", takeoff_land_cmd_from_bridge.drone_id);
                    }
                } else if (takeoff_land_cmd_from_bridge.takeoff_land_cmd == 2) {
                    if(takeoff_land_cmd_from_bridge.drone_id == 255){
                        ROS_INFO("LAND_CMD FOR ALL DRONES RECEIVED");
                    }
                    else if(takeoff_land_cmd_from_bridge.drone_id == self_id_){
                        ROS_INFO("LAND_CMD FOR ME RECEIVED");
                    }
                    else{
                        ROS_INFO("LAND_CMD FOR DRONE %d RECEIVED", takeoff_land_cmd_from_bridge.drone_id);
                    }
                    
                }
                else{
                    ROS_ERROR("Unknown takeoff_land_cmd");
                }

                // forward to other drones
                static ros::Time last_send_takeoff_;
                if ((ros::Time::now() - last_send_takeoff_).toSec() > 0.5) {
                    uwbSend(MESSAGE_TYPE::TAKEOFF_LAND, takeoff_land_cmd_from_bridge);
                    last_send_takeoff_ = ros::Time::now();
                }
                break;
            }
            case MESSAGE_TYPE::ONE_TRAJ: {
                // 飞机之间uwb通信：里程计、[轨迹_接收]
                swarm_bridge::MiniMINCO miniminco_;
                deserializeTopic(miniminco_);
                traj_utils::MINCOTraj minco_traj_;
                compresser_->decompressMinco(minco_traj_, miniminco_);
                minco_traj_.drone_id = (uint16_t)node.id;
                minco_traj_.des_clearance = 0.4;
                ROS_INFO("SEND TRAJ");
                minco_traj_pub_.publish(minco_traj_);
                break;
            }
            case MESSAGE_TYPE::GOAL: {
                static std::vector<ros::Time> goal_send_stamp_vec_(drone_num_);
                quadrotor_msgs::GoalSet goal_;
                deserializeTopic(goal_);
                int id = goal_.drone_id;
                ros::Time curr_stamp = ros::Time::now();

                if ((curr_stamp - goal_send_stamp_vec_[id]).toSec() > 0.5) {
                    if (id == self_id_) {
                        goal_pub_.publish(goal_);
                    } else {
                        uwbSend(MESSAGE_TYPE::GOAL, goal_);
                    }
                    goal_send_stamp_vec_[id] = ros::Time::now();
                }
                ROS_INFO("goal received");
                break;
            }
            case MESSAGE_TYPE::GPS_ORIGIN: {
                swarm_bridge::gps msg;
                deserializeTopic(msg);
                if (msg.drone_id == self_id_) {
                    gps_origin_pub.publish(msg);
                    ROS_INFO("gps_mean received");
                }

                break;
            }
            case MESSAGE_TYPE::TRIG:{
                geometry_msgs::PoseStamped msg;
                trigger_pub_.publish(msg);
                std_msgs::Empty empty_msg;
                static ros::Time last_send_trigger_;
                if ((ros::Time::now() - last_send_trigger_).toSec() > 0.5) {
                    uwbSend(MESSAGE_TYPE::TRIG, empty_msg);
                    last_send_trigger_ = ros::Time::now();
                }
                break;
            }
            case MESSAGE_TYPE::STOP:{
                std_msgs::Empty empty_msg;
                mandatory_stop_pub_.publish(empty_msg);
                static ros::Time last_send_stop;
                if ((ros::Time::now() - last_send_stop).toSec() > 0.5) {
                    uwbSend(MESSAGE_TYPE::STOP, empty_msg);
                    last_send_stop = ros::Time::now();
                }
                break;
            }

            case MESSAGE_TYPE::MS:{
                mission_msgs::Mission mission;
                deserializeTopic(mission);
                mission_pub_.publish(mission);
                static ros::Time last_send_mission;
                // forward to other drones
                if ((ros::Time::now() - last_send_mission).toSec() > 0.5) {
                    uwbSend(MESSAGE_TYPE::MS, mission);
                    last_send_mission = ros::Time::now();
                }
                break;
            }
            case MESSAGE_TYPE::TR_ST:{
                mission_msgs::TrackTargetState track_target_state;
                deserializeTopic(track_target_state);
                tracktargetstate_pub_.publish(track_target_state);
                break;
            }
            case MESSAGE_TYPE::OB_OV: {
                mission_msgs::ObjectObservation object_observation;
                deserializeTopic(object_observation);
                other_object_observation_pub_.publish(object_observation);
                // Forward to other drones
                // static ros::Time last_send_object_observation;
                // if ((ros::Time::now() - last_send_object_observation).toSec() > 0.2) {
                //     uwbSend(MESSAGE_TYPE::TR_OV, object_observation);
                //     last_send_object_observation = ros::Time::now();
                // }
                break;
            }
            case MESSAGE_TYPE::TR_ID:{
                mission_msgs::TrackTargetSet track_target_set;
                deserializeTopic(track_target_set);
                if(track_target_set.drone_id == self_id_)
                {
                    std_msgs::Int8 new_target_id_msg;
                    new_target_id_msg.data=track_target_set.target_id;
                    track_target_id_pub_.publish(new_target_id_msg);
                    ROS_INFO_THROTTLE(1.0,"new target : %d",track_target_set.target_id);
                }
                break;
            }
            default:
                ROS_ERROR("Unknown received message");
                break;
        }
        rec_mtx.unlock();
    }
}
void battery_state_msg_cb(const sensor_msgs::BatteryState::ConstPtr &msg) {
    drone_state_mtx.lock();
    drone_state_.battary_velocity = msg->voltage;
    drone_state_mtx.unlock();
}

void gps_msg_cb(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    static ros::Time t_last;
    ros::Time t_now = ros::Time::now();
    if ((t_now - t_last).toSec() < 0.2) return;
    t_last = t_now;
    swarm_bridge::gps gps_msg;
    gps_msg.drone_id = static_cast<uint8_t>(self_id_);
    gps_msg.latitude = msg->latitude;
    gps_msg.longitude = msg->longitude;
    uwbSend(MESSAGE_TYPE::GPS_ORIGIN, gps_msg);
}

void minco_traj_sub_cb(const traj_utils::MINCOTraj::ConstPtr &traj) {
    // 飞机之间uwb通信：里程计、[轨迹_发送]
    swarm_bridge::MiniMINCO mini_minco;
    compresser_->compressMinco(*traj, mini_minco);
    // drone_id 和 des_clearance要重新赋值
    // miniminco_debug_pub_.publish(mini_minco);
    // traj_utils::MINCOTraj full_minco;
    // compresser_->decompressMinco(full_minco, mini_minco);
    // minco_debug_pub_.publish(full_minco);
    uwbSend(MESSAGE_TYPE::ONE_TRAJ, mini_minco);
}
void self_odom_sub_cb(const nav_msgs::Odometry::ConstPtr &odom) {
    // 飞机之间uwb通信：[里程计_发送]、轨迹
    static ros::Time t_last;
    ros::Time t_now = ros::Time::now();
    if ((t_now - t_last).toSec() < 0.002) {
        return;
    }
    t_last = t_now;
    drone_state_mtx.lock();
    drone_state_.odom_stamp = odom->header.stamp;
    drone_state_.qx = odom->pose.pose.orientation.x;
    drone_state_.qy = odom->pose.pose.orientation.y;
    drone_state_.qz = odom->pose.pose.orientation.z;
    drone_state_.qw = odom->pose.pose.orientation.w;
    drone_state_.x = odom->pose.pose.position.x;
    drone_state_.y = odom->pose.pose.position.y;
    drone_state_.z = odom->pose.pose.position.z;
    if ((t_now - last_heartbeat_stamp_).toSec() > 0.2)
        drone_state_.heart_beat = false;
    else
        drone_state_.heart_beat = true;
    drone_state_mtx.unlock();
}
void odom_origin_msg_cb(const nav_msgs::Odometry::ConstPtr &odom) {
    static ros::Time t_last;
    ros::Time t_now = ros::Time::now();
    if ((t_now - t_last).toSec() < 0.002) {
        return;  // 就debug的时候可视化用，没必要太高频
    }
    t_last = t_now;
    drone_state_mtx.lock();
    drone_state_.gps_stamp = odom->header.stamp;
    drone_state_.gps_x = odom->pose.pose.position.x;
    drone_state_.gps_y = odom->pose.pose.position.y;
    if ((t_now - last_heartbeat_stamp_).toSec() > 0.2)
        drone_state_.heart_beat = false;
    else
        drone_state_.heart_beat = true;
    drone_state_mtx.unlock();
}
void traj_msg_cb(const traj_utils::PolyTraj::ConstPtr &msg) {
    uwbSend(MESSAGE_TYPE::POLY_TRAJ, *msg);
}
void goal_sub_cb(const quadrotor_msgs::GoalSet::ConstPtr &goal) {
    // goal_user2brig
    if (goal->drone_id == self_id_) {
        goal_pub_.publish(goal);  // goal_brig2planner
        ROS_INFO("send goal to myself");
        return;
    }
    uwbSend(MESSAGE_TYPE::GOAL, *goal);
}

void takeoff_land_sub_cb(
    const quadrotor_msgs::TakeOffLandToBridge::ConstPtr &msg) {
    // 该飞机通过uwb广播地面站发送的指令：[起飞、降落]、目标点
    // takeoff_land_user2brig
    quadrotor_msgs::TakeoffLand takeoff_land_msg;
    takeoff_land_msg.takeoff_land_cmd = msg->takeoff_land_cmd;
    uwbSend(MESSAGE_TYPE::TAKEOFF_LAND, *msg);
    static ros::Time last_takeoff_pub_stamp;
    ros::Time curr_time = ros::Time::now();
    if ((curr_time - last_takeoff_pub_stamp).toSec() > 0.5) {
        takeoff_pub.publish(takeoff_land_msg);
        last_takeoff_pub_stamp = curr_time;
    }
}

void heartbeat_msg_cb(const std_msgs::Empty::ConstPtr &msg) {
    last_heartbeat_stamp_ = ros::Time::now();
}
void state_sender_cb(const ros::TimerEvent &) {
    drone_state_mtx.lock();
    uwbSend(MESSAGE_TYPE::DRONE_STATE, drone_state_);
    drone_state_mtx.unlock();
}

void self_object_observation_cb(const mission_msgs::ObjectObservation::ConstPtr &msg) {
    // limit sending rate
    static ros::Time last_send_time = ros::Time(0);
    if((ros::Time::now()-last_send_time).toSec() > 0.05){ // 20hz max
        uwbSend(MESSAGE_TYPE::OB_OV, *msg);
        last_send_time = ros::Time::now();
    }
}

void track_target_state_cb(const mission_msgs::TrackTargetState::ConstPtr &msg) {
    // limit sending rate
    static ros::Time last_send_time = ros::Time(0);
    if((ros::Time::now()-last_send_time).toSec() > 0.05){ // 20hz max
        uwbSend(MESSAGE_TYPE::TR_ST, *msg);
        last_send_time = ros::Time::now();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "uwb_bridge");
    ros::NodeHandle nh("~");
    nh.param("self_id", self_id_, -1);
    nh.param("drone_num", drone_num_, -1);

    double full_state_send_rate = 30.0;
    nh.param("state_send_rate",full_state_send_rate,30.0);

    uwb_pub = nh.advertise<std_msgs::String>(
        "/nlink_linktrack_data_transmission", 100);
    other_odoms_pub_ = nh.advertise<nav_msgs::Odometry>("/others_odom", 10);

    minco_traj_pub_ =
        nh.advertise<traj_utils::MINCOTraj>("/broadcast_traj_to_planner", 100);

    takeoff_pub =
        nh.advertise<quadrotor_msgs::TakeoffLand>("takeoffland_cmd", 100);
    gps_origin_pub = nh.advertise<swarm_bridge::gps>("/gps_mean", 100);
    minco_debug_pub_ = nh.advertise<traj_utils::MINCOTraj>("/minco_debug", 100);
    miniminco_debug_pub_ =
        nh.advertise<swarm_bridge::MiniMINCO>("/miniminco_debug", 100);
    takeoff_land_from_bridge_pub_ =
        nh.advertise<quadrotor_msgs::TakeOffLandToBridge>(
            "/takeoff_land_from_bridge", 100);
    goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 100);

    trigger_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger",10);
    mandatory_stop_pub_ = nh.advertise<std_msgs::Empty>("/mandatory_stop_from_bridge", 10);

    ros::Subscriber self_odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "my_odom", 100, self_odom_sub_cb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber odom_origin_sub = nh.subscribe<nav_msgs::Odometry>(
        "/odom_add_bias", 100, odom_origin_msg_cb);

    ros::Subscriber takeoff_sub =
        nh.subscribe("/takeoff_land_user2brig", 100, takeoff_land_sub_cb,
                     ros::TransportHints().tcpNoDelay());

    ros::Subscriber uwb_receiver =
        nh.subscribe<nlink_parser::LinktrackNodeframe0>(
            "/nlink_linktrack_nodeframe0", 100, uwb_receiver_cb,
            ros::TransportHints().tcpNoDelay());

    ros::Subscriber minco_traj_sub_ = nh.subscribe<traj_utils::MINCOTraj>(
        "/broadcast_traj_from_planner", 100, minco_traj_sub_cb,
        ros::TransportHints().tcpNoDelay());
    ros::Subscriber goal_sub_ =
        nh.subscribe("/goal_user2brig", 100, goal_sub_cb,
                     ros::TransportHints().tcpNoDelay());

    ros::Subscriber heartbeat_sub = nh.subscribe<std_msgs::Empty>(
        "/drone_" + std::to_string(self_id_) + "_traj_server/heartbeat", 100,
        heartbeat_msg_cb);
    ros::Subscriber battery_state_sub = nh.subscribe<sensor_msgs::BatteryState>(
        "/mavros/battery", 100, battery_state_msg_cb);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(
        "/mavros/global_position/raw/fix", 100, gps_msg_cb);


    // track target state 
    tracktargetstate_pub_ = nh.advertise<mission_msgs::TrackTargetState>("/track_state_from_bridge",10);
    ros::Subscriber track_target_state_sub = nh.subscribe<mission_msgs::TrackTargetState>(
        "/track_state_to_bridge", 10, track_target_state_cb, ros::TransportHints().tcpNoDelay());

    // track observation
     ros::Subscriber self_object_observation_sub = nh.subscribe<mission_msgs::ObjectObservation>(
        "/object_observation_to_bridge", 100, self_object_observation_cb, ros::TransportHints().tcpNoDelay());
    other_object_observation_pub_ = nh.advertise<mission_msgs::ObjectObservation>("/others_object_observation", 100);

    // mission 
    mission_pub_ = nh.advertise<mission_msgs::Mission>("/mission_from_bridge", 10);

    track_target_id_pub_ = nh.advertise<std_msgs::Int8>("new_track_target_id", 100);


    // send odom in a fixed rate (hz)
    ros::Timer odom_sender =
        nh.createTimer(ros::Duration(1.0 / full_state_send_rate), state_sender_cb);
    ros::spin();
}
