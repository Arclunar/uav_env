#include <ros/ros.h>
#include <serial/serial.h>

#include "eigen3/Eigen/Eigen"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/GoalSet.h"
#include "quadrotor_msgs/TakeOffLandToBridge.h"
#include "quadrotor_msgs/TakeoffLand.h"
using namespace Eigen;
using namespace std;
ros::Time odom_stamp;
vector<Vector3f> odom_vec;
serial::Serial sp;
uint8_t write_buffer[50];
int self_id;
// 定义帧头和CRC校验长度
const uint8_t FRAME_HEADER = 0x3E;
const uint8_t CRC_LENGTH = 2;
ros::Publisher takeoff_land_pub_px4, takeoff_land_pub, goal_formation_pub,
    goal_trans_pub, goal_tcp_pub;
ros::Subscriber odom_sub;
// 定义协议中的字段偏移量
const size_t ID_OFFSET = 1;
const size_t LENGTH_OFFSET = 2;
const size_t DATA_OFFSET = 3;

const uint8_t crctablehi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40};
const uint8_t crctablelo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40};
uint16_t crc16table(uint8_t *ptr, int len) {
    int crchi = 0xff;
    int crclo = 0xff;
    int index;
    while (len--) {
        index = crclo ^ *ptr++;
        crclo = crchi ^ crctablehi[index];
        crchi = crctablelo[index];
    }
    return (crchi << 8 | crclo);
}
uint32_t float2uint(float floatValue) {
    uint32_t intValue;
    std::memcpy(&intValue, &floatValue, sizeof(float));
    // std::cout << "Hex representation: 0x" << std::hex << std::uppercase <<
    // intValue << std::dec << std::nouppercase << std::endl;
    return intValue;
}
float uint2float(uint32_t intValue) {
    float floatValue;
    std::memcpy(&floatValue, &intValue, sizeof(float));
    // printf("%f\n", floatValue);
    return floatValue;
}
uint32_t mergeHex2Uint32(uint8_t *data) {
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

void telemetry_Tx(uint8_t *tmp_buffer, size_t size) {
    // if (self_id != 0) return;
    uint16_t crc16 = crc16table(tmp_buffer, size);
    uint8_t crc16L = (crc16 & 0xFF), crc16H = (crc16 >> 8);

    for (size_t i = 0; i < size; i++) write_buffer[i] = tmp_buffer[i];

    write_buffer[size] = crc16L;
    write_buffer[size + 1] = crc16H;

    // for (size_t i = 0; i < size + 2; i++)
    // {
    //     printf("0x%x, ", write_buffer[i]);
    // }
    // printf("\n");
    sp.write(write_buffer, size + 2);
}

void telemetry_air2ground_odom(uint8_t id, uint32_t sec, uint32_t nsec, float x,
                               float y, float z) {
    static ros::Time t_last;
    ros::Time t_now = ros::Time::now();
    if ((t_now - t_last).toSec() < 0.03) {
        return;
    }
    t_last = t_now;

    uint8_t data_len = 21;

    uint8_t tmp_buffer[] = {0x3E,
                            id,
                            data_len,
                            0x04,
                            (uint8_t)((sec >> 24) & 0xFF),
                            (uint8_t)((sec >> 16) & 0xFF),
                            (uint8_t)((sec >> 8) & 0xFF),
                            (uint8_t)(sec & 0xFF),
                            (uint8_t)((nsec >> 24) & 0xFF),
                            (uint8_t)((nsec >> 16) & 0xFF),
                            (uint8_t)((nsec >> 8) & 0xFF),
                            (uint8_t)(nsec & 0xFF),
                            (uint8_t)((float2uint(x) >> 24) & 0xFF),
                            (uint8_t)((float2uint(x) >> 16) & 0xFF),
                            (uint8_t)((float2uint(x) >> 8) & 0xFF),
                            (uint8_t)(float2uint(x) & 0xFF),
                            (uint8_t)((float2uint(y) >> 24) & 0xFF),
                            (uint8_t)((float2uint(y) >> 16) & 0xFF),
                            (uint8_t)((float2uint(y) >> 8) & 0xFF),
                            (uint8_t)(float2uint(y) & 0xFF),
                            (uint8_t)((float2uint(z) >> 24) & 0xFF),
                            (uint8_t)((float2uint(z) >> 16) & 0xFF),
                            (uint8_t)((float2uint(z) >> 8) & 0xFF),
                            (uint8_t)(float2uint(z) & 0xFF)};
    size_t size = sizeof(tmp_buffer);
    telemetry_Tx(tmp_buffer, size);
}

void telemetry_air2ground_Rx() {
    size_t n = sp.available();
    if (n != 0) {
        uint8_t Rx_buffer[30];
        n = sp.read(Rx_buffer, n);
        if (Rx_buffer[0] != FRAME_HEADER) {
            ROS_ERROR("Telemetry: frame header is wrong");
            return;
        }

        uint8_t drone_id = Rx_buffer[ID_OFFSET];
        uint8_t data_len = Rx_buffer[LENGTH_OFFSET];
        uint8_t data_cmd = Rx_buffer[DATA_OFFSET];

        if (DATA_OFFSET + data_len + CRC_LENGTH != n) {
            printf("%d\n", data_len);
            ROS_ERROR("Telemetry: data length is wrong");
            return;
        }

        uint16_t crc16 = crc16table(Rx_buffer, n - 2);
        uint8_t crc16L = (crc16 & 0xFF), crc16H = (crc16 >> 8);
        if (Rx_buffer[n - 2] != crc16L || Rx_buffer[n - 1] != crc16H) {
            ROS_ERROR("Telemetry: cyclic redundancy check is wrong");
            return;
        }

        // takeoff
        if (data_cmd == 0x01) {
            if (drone_id == 0xff) {
                printf("all drones takeoff command\n");
                quadrotor_msgs::TakeOffLandToBridge cmd;
                cmd.drone_id = drone_id;
                cmd.takeoff_land_cmd = 1;
                takeoff_land_pub.publish(cmd);
                quadrotor_msgs::TakeoffLand cmd_px4;
                cmd_px4.takeoff_land_cmd = 1;
                takeoff_land_pub_px4.publish(cmd_px4);
            } else {
                printf("ID=%d drone takeoff command\n", drone_id);
                if (drone_id == self_id) {
                    quadrotor_msgs::TakeOffLandToBridge cmd;
                    cmd.drone_id = drone_id;
                    cmd.takeoff_land_cmd = 1;
                    takeoff_land_pub.publish(cmd);
                    quadrotor_msgs::TakeoffLand cmd_px4;
                    cmd_px4.takeoff_land_cmd = 1;
                    takeoff_land_pub_px4.publish(cmd_px4);
                }
            }
        }
        // land
        else if (data_cmd == 0x02) {
            if (drone_id == 0xff) {
                printf("all drones land command\n");
                quadrotor_msgs::TakeOffLandToBridge cmd;
                cmd.drone_id = 255;
                cmd.takeoff_land_cmd = 2;
                takeoff_land_pub.publish(cmd);
                quadrotor_msgs::TakeoffLand cmd_px4;
                cmd_px4.takeoff_land_cmd = 2;
                takeoff_land_pub_px4.publish(cmd_px4);
            } else {
                printf("ID=%d drone land command\n", drone_id);
                if (drone_id == self_id) {
                    quadrotor_msgs::TakeOffLandToBridge cmd;
                    cmd.drone_id = drone_id;
                    cmd.takeoff_land_cmd = 2;
                    takeoff_land_pub.publish(cmd);
                    quadrotor_msgs::TakeoffLand cmd_px4;
                    cmd_px4.takeoff_land_cmd = 2;
                    takeoff_land_pub_px4.publish(cmd_px4);
                }
            }
        }
        // goal_with_id
        else if (data_cmd == 0x03) {
            float x = uint2float(mergeHex2Uint32(&Rx_buffer[DATA_OFFSET + 1]));
            float y = uint2float(mergeHex2Uint32(&Rx_buffer[DATA_OFFSET + 5]));
            float z = uint2float(mergeHex2Uint32(&Rx_buffer[DATA_OFFSET + 9]));
            printf("ID=%d drone move to (%f %f %f)\n", drone_id, x, y, z);
            if (drone_id == 254) {
                geometry_msgs::PoseStamped goal_formation;
                goal_formation.header.frame_id = "world";
                goal_formation.header.stamp = ros::Time::now();
                goal_formation.pose.orientation.w = 1.0;
                goal_formation.pose.position.x = x;
                goal_formation.pose.position.y = y;
                goal_formation.pose.position.z = z;
                goal_formation_pub.publish(goal_formation);
                quadrotor_msgs::GoalSet goal_tcp;
                goal_tcp.drone_id = 254;
                goal_tcp.goal[0] = x;
                goal_tcp.goal[1] = y;
                goal_tcp.goal[2] = z;
                goal_tcp_pub.publish(goal_tcp);
            } else {
                quadrotor_msgs::GoalSet goal_to_pub;
                goal_to_pub.drone_id = (uint16_t)drone_id;
                goal_to_pub.goal[0] = x;
                goal_to_pub.goal[1] = y;
                goal_to_pub.goal[2] = z;
                goal_trans_pub.publish(goal_to_pub);
                goal_tcp_pub.publish(goal_to_pub);
            }
        }
    }
}
void odom_cb(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    float x = odom_msg->pose.pose.position.x;
    float y = odom_msg->pose.pose.position.y;
    float z = odom_msg->pose.pose.position.z;
    uint32_t sec = odom_msg->header.stamp.sec;
    uint32_t nsec = odom_msg->header.stamp.nsec;
    int drone_id = std::stoi(odom_msg->child_frame_id.substr(6));
    telemetry_air2ground_odom(uint8_t(drone_id), sec, nsec, x, y, z);
}
void Timer1Callback(const ros::TimerEvent &) { telemetry_air2ground_Rx(); }

int main(int argc, char **argv) {
    ros::init(argc, argv, "telemetry_air2ground_node");
    ros::NodeHandle nh("~");
    std::string serial_port;
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh.param("drone_id", self_id, -1);
    ROS_INFO("did = %d", self_id);
    odom_vec.resize(20);
    takeoff_land_pub = nh.advertise<quadrotor_msgs::TakeOffLandToBridge>(
        "/takeoff_land_user2brig", 100);
    takeoff_land_pub_px4 =
        nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 100);
    goal_formation_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 100);

    goal_trans_pub =
        nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 100);
    goal_tcp_pub =
        nh.advertise<quadrotor_msgs::GoalSet>("/goal_user2brig", 100);
    /*串口初始化*/

    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    sp.setPort(serial_port);

    sp.setBaudrate(57600);
    sp.setTimeout(timeout);

    try {
        sp.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    if (sp.isOpen())
        ROS_INFO_STREAM(serial_port << " is opened.");
    else
        return -1;

    ros::Timer timer1 = nh.createTimer(ros::Duration(0.001), Timer1Callback);
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("all_odom", 100, odom_cb);
    ros::Rate loop_rate(1000);
    ros::spin();
    sp.close();

    return 0;
}
