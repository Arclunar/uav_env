#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include "std_msgs/UInt32.h"

double _imu_temp;
int _sate_num;
double _vx,_vy,_vz,_total_speed;
double _px,_py,_pz;
double _px_c,_py_c,_pz_c;
ros::Time ekf_odom_stamp,ekf_odom_corr_stamp,imu_temp_stamp,sate_num_stamp;
bool _corr_available, _ori_available;

void _cb(const ros::TimerEvent& event) {

    ros::Time now = ros::Time::now();

    ROS_INFO("IMU Temp: %f", _imu_temp);
    ROS_INFO("Sat Num: %d", _sate_num);

    if((now - ekf_odom_corr_stamp).toSec() > 1.0)
        _corr_available = false;
    if((now - ekf_odom_stamp).toSec() > 1.0)
        _ori_available = false;    

    if(_ori_available)
    {
        ROS_INFO("OriPos: [%f, %f, %f]", _px, _py, _pz);
    }

    if(_corr_available)
    {
        // 计算总速度
        _total_speed = std::sqrt(_vx * _vx + _vy * _vy + _vz * _vz);

        // 选择颜色，根据合速度的大小
        std::string color;
        if (_total_speed < 0.1) {
            color = "\033[32m"; // 绿色
        } else if (_total_speed < 0.2) {
            color = "\033[33m"; // 黄色
        } else {
            color = "\033[31m"; // 红色
        }

        // 打印信息，速度部分根据总速度改变颜色
        ROS_INFO("CorPos: [%f, %f, %f]",_px_c, _py_c, _pz_c);
        ROS_INFO("%sSpeed: [%f, %f, %f], Total Speed: %f\033[0m",color.c_str(),_vx, _vy, _vz, _total_speed);
    }
}

void ekf_odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    ekf_odom_stamp = ros::Time::now();
    _ori_available = true;
    _px = msg->pose.pose.position.x;
    _py = msg->pose.pose.position.y;
    _pz = msg->pose.pose.position.z;
}

void odom_corr_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    ekf_odom_corr_stamp = ros::Time::now();
    _corr_available = true;
    _vx = msg->twist.twist.linear.x;
    _vy = msg->twist.twist.linear.y;
    _vz = msg->twist.twist.linear.z;
    _px_c = msg->pose.pose.position.x;
    _py_c = msg->pose.pose.position.y;
    _pz_c = msg->pose.pose.position.z;
}

void sate_num_cb(const std_msgs::UInt32::ConstPtr &msg) {
    sate_num_stamp = ros::Time::now();
    _sate_num = msg->data;
}

void imu_temp_cb(const sensor_msgs::TemperatureConstPtr &msg) {
    imu_temp_stamp = ros::Time::now();
    _imu_temp = msg->temperature;
}
int main(int argc, char *argv[]) {
    _ori_available = false;
    _corr_available = false;
    ros::init(argc, argv, "echo_state_node");
    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), _cb);
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("/ekf/ekf_odom", 100, ekf_odom_cb);
    ros::Subscriber odom_corr_sub = nh.subscribe<nav_msgs::Odometry>("/ekf/ekf_odom_corrected", 100, odom_corr_cb);
    ros::Subscriber satellites_sub = nh.subscribe<std_msgs::UInt32>(
        "/mavros/global_position/raw/satellites", 100, sate_num_cb);
    ros::Subscriber imu_temp_sub = nh.subscribe<sensor_msgs::Temperature>(
        "/mavros/imu/temperature_imu", 100, imu_temp_cb);
    ros::spin();
    return 0;
}
