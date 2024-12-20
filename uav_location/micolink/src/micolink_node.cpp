#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <serial/serial.h>
#include <iostream>
#include "micolink/mtf01.h"
#include "sensor_msgs/Range.h"
#include <queue>
#include <numeric>
ros::Publisher distance_pub_;
// #include "include/mtf01.h"

// ros::Publisher flow_pub;
// 光流-测距数据消息定义
// typedef struct
// {
//     std_msgs::Header header;
//     uint32_t  time_ms;              // 系统时间 ms
//     uint32_t  distance;			    // 距离(mm) 最小值为10，0表示数据不可用
//     uint8_t   strength;	            // 信号强度
//     uint8_t   precision;	        // 精度
//     uint8_t   tof_status;	        // 状态
//     uint8_t  reserved1;			    // 预留
//     int16_t   flow_vel_x;	        // 光流速度x轴
//     int16_t   flow_vel_y;	        // 光流速度y轴
//     uint8_t   flow_quality;	        // 光流质量
//     uint8_t   flow_status;	        // 光流状态
//     uint16_t  reserved2;	        // 预留
// } FlowRangeData;

// FlowRangeData flow_range_data;

void micolink_decode(uint8_t data)
{
    static MICOLINK_MSG_t msg;

    if (micolink_parse_char(&msg, data) == false)
        return;

    switch (msg.msg_id)
    {
    case MICOLINK_MSG_ID_RANGE_SENSOR:
    {
        MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
        memcpy(&payload, msg.payload, msg.len);

        /*
            此处可获取传感器数据:

            距离        = payload.distance;
            强度        = payload.strength;
            精度        = payload.precision;
            距离状态    = payload.tof_status;
            光流速度x轴 = payload.flow_vel_x;
            光流速度y轴 = payload.flow_vel_y;
            光流质量    = payload.flow_quality;
            光流状态    = payload.flow_status;
        */
        break;
    }

    default:
        break;
    }
}

bool micolink_check_sum(MICOLINK_MSG_t *msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for (uint8_t i = 0; i < length; i++)
    {
        checksum += temp[i];
    }

    if (checksum == msg->checksum)
        return true;
    else
        return false;
}

bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data) // 检测数据格式
{
    switch (msg->status)
    {
    case 0: // 帧头
        if (data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
        }
        else
        {
            std::cout << "帧头格式错误，接收到的数据为：" << data << std::endl;
            msg->status = 0; // 重置状态接收下一帧数据
        }
        break;

    case 1: // 设备ID
        msg->dev_id = data;
        msg->status++;
        break;

    case 2: // 系统ID
        msg->sys_id = data;
        msg->status++;
        break;

    case 3: // 消息ID
        msg->msg_id = data;
        msg->status++;
        break;

    case 4: // 包序列
        msg->seq = data;
        msg->status++;
        break;

    case 5: // 负载长度
        msg->len = data;
        if (msg->len == 0)
        {
            msg->status += 2;
        }
        else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN)
        {
            msg->status = 0;
            std::cout << "负载长度格式错误，接收到的数据为：" << data << std::endl;
        }
        else
        {
            msg->status++;
        }
        break;

    case 6: // 数据负载接收
        msg->payload[msg->payload_cnt++] = data;
        if (msg->payload_cnt == msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;

    case 7: // 帧校验
        msg->checksum = data;
        msg->status = 0;
        if (!micolink_check_sum(msg))
        {
            std::cout << "帧校验格式错误，接收到的数据为：" << data << std::endl;
        }
        else
        {
            return true; // 返回 true 表示成功接收到一帧数据
        }
        break;

    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }

    return false;
}

void micolink_decode_callback(uint8_t data)
{
    //  std::string binaryData = hexToBinary(std::to_string(data));
    static MICOLINK_MSG_t msg;

    if (micolink_parse_char(&msg, data) == false)
        // ROS_INFO("micolink_decode_callback");
        return;

    MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;

    payload.time_ms = *(uint32_t *)(msg.payload + 0);
    payload.distance = *(uint32_t *)(msg.payload + 4); // 距离(mm) 最小值为10，0表示数据不可用
    payload.strength = *(uint8_t *)(msg.payload + 8);
    payload.precision = *(uint8_t *)(msg.payload + 9);
    payload.tof_status = *(uint8_t *)(msg.payload + 10);
    payload.reserved1 = *(uint8_t *)(msg.payload + 11);
    payload.flow_vel_x = *(int16_t *)(msg.payload + 12);
    payload.flow_vel_y = *(int16_t *)(msg.payload + 14);
    payload.flow_quality = *(uint8_t *)(msg.payload + 16);
    payload.flow_status = *(uint8_t *)(msg.payload + 17);
    payload.strength = *(uint16_t *)(msg.payload + 18);
    static std::vector<float> height_queue_;
    float measure = static_cast<float>(payload.distance) * 1e-3;
    if(height_queue_.size() < 10){
        height_queue_.emplace_back(measure);
    }
    if(height_queue_.size() == 10){
        height_queue_.erase(height_queue_.begin());
        height_queue_.emplace_back(measure);
        sensor_msgs::Range rng;
        rng.range = 0.0;
        rng.header.stamp = ros::Time::now();
        for(int i =0; i< 10; i++){
            rng.range += 0.1 * height_queue_[i];
        }
        distance_pub_.publish(rng);
    }

    

    ros::Duration(0.008).sleep();
}
void readSerialData(serial::Serial &ser)
{

    if (ser.available())
    {
        // 读取一个字节的数据
        uint8_t byte_data;
        ser.read(&byte_data, 1);

        micolink_decode_callback(byte_data);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "micolink_node");
    ros::NodeHandle nh("~");
    // distance_pub_ = nh.advertise<>
    // flow_pub = nh.advertise<FlowRangeData>("flow_range_data", 10);
    // TODO: 串口读取数据并调用micolink_decode_callback函数处理
    serial::Serial ser;
    std::string port_name;
    nh.getParam("port_name", port_name);
    distance_pub_ = nh.advertise<sensor_msgs::Range>("/micolink_tof", 100);
    try
    {
        // 打开串口
        ser.setPort(port_name);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setBytesize(serial::eightbits);    // 设置数据位为8
        ser.setParity(serial::parity_none);    // 设置无奇偶校验
        ser.setStopbits(serial::stopbits_one); // 设置停止位为1
        ser.setTimeout(to);
        ser.open();
    }
    // catch (serial::IOException& e)
    // {
    //     ROS_ERROR_STREAM("Unable to open port ");
    //     return -1;
    // }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port: " << e.what());
        return -1;
    }

    // 检查串口是否打开
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    // 循环读取数据
    while (ros::ok())
    {
        if (ser.available())
        {
            // ROS_INFO_STREAM("READY!!!");
            // 读取一个字节的数据
            // uint8_t byte_data;
            // ser.read(&byte_data, 1);
            // //打印读取数据
            // std::cout << "Read data: " << static_cast<int>(byte_data) << std::endl;
            // // std_msgs::String serial_data;
            // // serial_data.data = ser.read(ser.available());
            // // 调用解码函数处理数据
            // micolink_decode_callback(byte_data);
            readSerialData(ser);
        }
        ros::spinOnce();
    }

    ros::spin();

    return 0;
}
