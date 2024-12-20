// task1:收到一帧起飞就20Hz广播起飞
// 0, 1, 2, 3 排成一列 收到起飞命令后往前走，直到收到下一架飞机的起飞命令
#include "quadrotor_msgs/TakeOffLandToBridge.h"
#include "ros/ros.h"
ros::Publisher takeoff_to_bridge_pub_;
enum CURR_MODE {
    BROCASTING =100,  // 接收到了地面站或上一个飞机的起飞指令，但是没有接收到下一个飞机返回的起飞指令
    HOVER, // 接收到了下一架飞机的起飞指令，进入悬停状态
} curr_mode_;
void takeoff_land_from_bridge_cb(
    const quadrotor_msgs::TakeOffLandToBridge::ConstPtr& msg) {
    // 
}
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "creazy_brocast_node");
    ros::NodeHandle nh("~");
    takeoff_to_bridge_pub_ = nh.advertise<quadrotor_msgs::TakeOffLandToBridge>(
        "/takeoff_land_user2brig", 100);
    ros::Subscriber takeoff_from_bridge =
        nh.subscribe<quadrotor_msgs::TakeOffLandToBridge>(
            "/takeoff_land_from_bridge", 100, takeoff_land_from_bridge_cb);
    ros::spin();
    ros::shutdown();
    return 0;
}

