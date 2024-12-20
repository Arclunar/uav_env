#include "ros/ros.h"
#include "nlink_parser/LinktrackNodeframe3.h"
#include "map"
#include "vector"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "Eigen/Core"
#include <random>

using namespace std;


ros::Subscriber self_odom_sub, other_odom_sub;
ros::Publisher noise_odom_pub,uwb_info_pub;

nav_msgs::Odometry self_odom_msg;
Eigen::Vector3d self_pos; 
map<int,nav_msgs::Odometry> other_odom_msg;
map<int,Eigen::Vector3d> other_pos;
int self_id;
double std_error;
std::default_random_engine generator;
unique_ptr<std::normal_distribution<double>> dist;
double noise()
{
    return (*dist)(generator);
}

void self_odom_cb(const nav_msgs::OdometryPtr &msg)
{
    self_odom_msg = *msg;
    self_pos<<self_odom_msg.pose.pose.position.x,
            self_odom_msg.pose.pose.position.y,
            self_odom_msg.pose.pose.position.z;
}

void other_odom_cb(const nav_msgs::OdometryPtr &msg)
{
    int id = atoi(msg->child_frame_id.substr(6, 10).c_str());
    if ( msg->child_frame_id.substr(0, 6) != string("drone_"))
    {
        ROS_ERROR("[random_goals_node] Wrong child_frame_id: %s", msg->child_frame_id.substr(0, 6).c_str());
        return;
    }
    other_odom_msg[id] = *msg;
    other_pos[id] << msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z;
        
}

void timer_cb(const ros::TimerEvent&)
{
    nlink_parser::LinktrackNodeframe3 send_msg;
    send_msg.id = self_id;
    for (auto& pos : other_pos)
    {
        if(pos.first == self_id)
        {
            continue;
        }
        else
        {
            nlink_parser::LinktrackNode2 node;
            Eigen::Vector3d diff = pos.second - self_pos;
            double distance = diff.norm();
            node.id = pos.first;
            node.dis = distance + noise();
            send_msg.nodes.emplace_back(node);

        }
        
    }
    uwb_info_pub.publish(send_msg);
    
    
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"fake_wub_node");
    ros::NodeHandle nh("~");

    nh.param("std_err",std_error,0.1);
    nh.param("self_id",self_id, -1);
    if(self_id == -1){
        ROS_ERROR("fake_uwb_node: drone id not set");
        exit(1);
    }
    dist.reset(new std::normal_distribution<double>(0,std_error));
    // uwb_info_sub = nh.subscribe("/nlink_linktrack_nodeframe3",10,uwb_info_cb,ros::TransportHints().tcpNoDelay());
    self_odom_sub = nh.subscribe("my_odom",10,self_odom_cb,ros::TransportHints().tcpNoDelay());
    other_odom_sub = nh.subscribe("/others_odom",10,other_odom_cb,ros::TransportHints().tcpNoDelay());
    uwb_info_pub = nh.advertise<nlink_parser::LinktrackNodeframe3>("nlink_linktrack_nodeframe3",50);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/50.0), timer_cb);

    ros::spin();
    return 0;
}

