#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "iostream"


int yaw_cmd_type_ = 0;
double yaw_cmd_ = 0.0;
double last_yaw_ = 0.0;
double last_yawdot_ = 0.0;

constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;


void fix_yaw_cmd_cb(const std_msgs::Float64::ConstPtr& msg) {
    yaw_cmd_ = msg->data;
    // print in green color
    ROS_INFO("\033[32m[yaw_cmd_] yaw_cmd_: %f\033[0m", yaw_cmd_);
}

ros::Publisher pub;
// 回调函数，用于处理接收到的消息
void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    // 创建要发布的消息
    quadrotor_msgs::PositionCommand modified_msg = *msg;

    double yaw_cmd = 0.0;

    switch(yaw_cmd_type_){
        case 0: // fix zero yaw
            yaw_cmd = 0.0;
            break;
        case 1: // forward to trajectory front
            pub.publish(modified_msg);
            return ;            
            break;
        case 2: // externel yaw cmd
            yaw_cmd = yaw_cmd_;
            break;
        default:
            yaw_cmd = 0.0;
            break;
    }

    std::pair<double, double> yaw_yawdot(0, 0);

    static ros::Time time_last = ros::Time::now();

    ros::Time time_now = ros::Time::now();

    double dt = (time_now - time_last).toSec();

 

    // smooth yaw cmd
    // just set the yaw to the cmd yaw
    double yaw_temp = yaw_cmd;

    double yawdot = 0;
    double d_yaw = yaw_temp - last_yaw_;
    while (d_yaw >= M_PI)
    {
      d_yaw -= 2 * M_PI;
    }
    while (d_yaw <= -M_PI)
    {
      d_yaw += 2 * M_PI;
    }

    // yaw rate limit according direction
    const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
    const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
    double d_yaw_max;
    if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM)) // speed will not exceed the maximum speed
    {
      // yawdot = last_yawdot_ + dt * YDDM;
      d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
    }
    else // if speed exceeds the maximum speed
    {
      // yawdot = YDM;
      double t1 = (YDM - last_yawdot_) / YDDM;
      d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
    }

    if (fabs(d_yaw) > fabs(d_yaw_max))
    {
      d_yaw = d_yaw_max;
    }
    yawdot = d_yaw / dt;

    double yaw = last_yaw_ + d_yaw;
    while (yaw > M_PI)
      yaw -= 2 * M_PI;
    while (yaw < -M_PI)
      yaw += 2 * M_PI;
    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    last_yaw_ = yaw_yawdot.first;
    last_yawdot_ = yaw_yawdot.second;
    time_last = time_now;


    // yaw_yawdot.second = yaw_temp;

    modified_msg.yaw = yaw_yawdot.first;
    modified_msg.yaw_dot = yaw_yawdot.second;

    pub.publish(modified_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yaw_modifier_node");
    ros::NodeHandle nh("~");

    // get if use fix yaw cmd
    nh.param("yaw_cmd_type", yaw_cmd_type_, 0);

    // 订阅原始消息
    ros::Subscriber sub =
        nh.subscribe("cmd_in",
                     100, posCmdCallback);
    
    ros::Subscriber fix_yaw_cmd_sub = nh.subscribe<std_msgs::Float64>("yaw_cmd", 10, fix_yaw_cmd_cb);
    pub = nh.advertise<quadrotor_msgs::PositionCommand>("cmd_out", 10);
    ros::spin();

    return 0;
}
