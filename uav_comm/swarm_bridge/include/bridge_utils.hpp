#ifndef __BRIDGE_UTILS__
#define __BRIDGE_UTILS__
#include <ros/ros.h>
#include <swarm_bridge/MiniMINCO.h>
#include <traj_utils/MINCOTraj.h>

#include <algorithm>
#include <iostream>
using namespace std;
class DataCompression {
    uint8_t f32_u8(float value, float max_val, float min_val) {
        // Scale the value to the [0, 255] range
        float scaledValue = (value - min_val + 1e-3) / (max_val - min_val + 1e-3);
        return static_cast<uint8_t>(scaledValue * 255.0f);
    }
    uint16_t f32_u16(float value, float max_val, float min_val) {
        float scaledValue = (value - min_val + 1e-3) / (max_val - min_val + 1e-3) ;
        return static_cast<uint16_t>(scaledValue * 65535.0f);
    }
    float u8_f32(uint8_t byteValue, float max_val, float min_val) {
        float scaledValue = static_cast<float>(byteValue) / 255.0f;
        float final_val = (scaledValue * (max_val - min_val)) + min_val;
    }
    float u16_f32(uint16_t byteValue, float max_val, float min_val) {
        float scaledValue = static_cast<float>(byteValue) / 65535.0f;
        return (scaledValue * (max_val - min_val)) + min_val;
    }

   public:
    void compressMinco(const traj_utils::MINCOTraj origin_minco,
                       swarm_bridge::MiniMINCO &mini_minco);

    void decompressMinco(traj_utils::MINCOTraj &origin_minco,
                         swarm_bridge::MiniMINCO &mini_minco);
    // void compressPoly(const traj_utils::PolyTraj origin_poly, swarm_bridge::)
};
void DataCompression::compressMinco(const traj_utils::MINCOTraj origin_minco,
                                    swarm_bridge::MiniMINCO &mini_minco) {
    vector<float> x_lis{origin_minco.end_p[0], origin_minco.start_p[0]};
    vector<float> y_lis{origin_minco.end_p[1], origin_minco.start_p[1]};
    vector<float> z_lis{origin_minco.end_p[2], origin_minco.start_p[2]};
    for (int i = 0; i < origin_minco.inner_x.size(); i++) {
        x_lis.emplace_back(origin_minco.inner_x[i]);
        y_lis.emplace_back(origin_minco.inner_y[i]);
        z_lis.emplace_back(origin_minco.inner_z[i]);
    }
    float max_x = *max_element(x_lis.begin(), x_lis.end());
    float max_y = *max_element(y_lis.begin(), y_lis.end());
    float max_z = *max_element(z_lis.begin(), z_lis.end());
    float min_x = *min_element(x_lis.begin(), x_lis.end());
    float min_y = *min_element(y_lis.begin(), y_lis.end());
    float min_z = *min_element(z_lis.begin(), z_lis.end());
    mini_minco.traj_id = static_cast<uint8_t>(origin_minco.traj_id);
    mini_minco.max_p[0] =
        f32_u16(max_x, 100.0f, -100.0f);  // 认为地图大小是-100->100

    mini_minco.max_p[1] =
        f32_u16(max_y, 100.0f, -100.0f);  // 认为地图大小是-100->100
    mini_minco.max_p[2] =
        f32_u16(max_z, 10.0f, -10.0f);  // 认为地图大小是-10->10
    mini_minco.min_p[0] = f32_u16(min_x, 100.0f, -100.0f);
    mini_minco.min_p[1] = f32_u16(min_y, 100.0f, -100.0f);
    mini_minco.min_p[2] = f32_u16(min_z, 10.0f, -10.0f);

    for (float dur : origin_minco.duration) {
        mini_minco.duration.emplace_back(f32_u8(dur, 5.0f, 0.0f));
    }
    mini_minco.start_p[0] = f32_u8(origin_minco.start_p[0], max_x, min_x);
    mini_minco.start_p[1] = f32_u8(origin_minco.start_p[1], max_y, min_y);
    mini_minco.start_p[2] = f32_u8(origin_minco.start_p[2], max_z, min_z);

    mini_minco.end_p[0] = f32_u8(origin_minco.end_p[0], max_x, min_x);
    mini_minco.end_p[1] = f32_u8(origin_minco.end_p[1], max_y, min_y);
    mini_minco.end_p[2] = f32_u8(origin_minco.end_p[2], max_z, min_z);

    for (int i = 0; i < 3; i++) {
        mini_minco.start_a[i] = f32_u16(origin_minco.start_a[i], 3.0f, -3.0f);
        mini_minco.start_v[i] = f32_u16(origin_minco.start_v[i], 3.0f, -3.0f);

        mini_minco.end_a[i] = f32_u16(origin_minco.end_a[i], 3.0f, -3.0f);
        mini_minco.end_v[i] = f32_u16(origin_minco.end_v[i], 3.0f, -3.0f);
    }
    mini_minco.inner_x.resize(origin_minco.inner_x.size());
    mini_minco.inner_y.resize(origin_minco.inner_x.size());
    mini_minco.inner_z.resize(origin_minco.inner_x.size());
    for (int i = 0; i < origin_minco.inner_x.size(); i++) {
        mini_minco.inner_x[i] = f32_u8(origin_minco.inner_x[i], max_x, min_x);
        mini_minco.inner_y[i] = f32_u8(origin_minco.inner_y[i], max_y, min_y);
        mini_minco.inner_z[i] = f32_u8(origin_minco.inner_z[i], max_z, min_z);
    }
}
void DataCompression::decompressMinco(traj_utils::MINCOTraj &origin_minco,
                                      swarm_bridge::MiniMINCO &mini_minco) {
    origin_minco.des_clearance = 0.5;  // 大家都一样，根据自己的补上
    origin_minco.drone_id = -1;        // 根据uwb的数据补上
    origin_minco.order = 5;
    for (auto dur : mini_minco.duration) {
        origin_minco.duration.emplace_back(u8_f32(dur, 5.0f, 0.0f));
    }
    vector<float> max_p, min_p;
    for (int i = 0; i < 2; i++) {
        max_p.emplace_back(u16_f32(mini_minco.max_p[i], 100.0f, -100.0f));
        min_p.emplace_back(u16_f32(mini_minco.min_p[i], 100.0f, -100.0f));
    }
    max_p.emplace_back(u16_f32(mini_minco.max_p[2], 10.0f, -10.0f));
    min_p.emplace_back(u16_f32(mini_minco.min_p[2], 10.0f, -10.0f));

    for (int i = 0; i < 3; i++) {
        origin_minco.start_p[i] =
            u8_f32(mini_minco.start_p[i], max_p[i], min_p[i]);
        origin_minco.start_v[i] = u16_f32(mini_minco.start_v[i], 3.0f, -3.0f);
        origin_minco.start_a[i] = u16_f32(mini_minco.start_a[i], 3.0f, -3.0f);

        origin_minco.end_p[i] =
            u8_f32(mini_minco.end_p[i], max_p[i], min_p[i]);
        origin_minco.end_v[i] = u16_f32(mini_minco.end_v[i], 3.0f, -3.0f);
        origin_minco.end_a[i] = u16_f32(mini_minco.end_a[i], 3.0f, -3.0f);
    }
    origin_minco.inner_x.resize(mini_minco.inner_x.size());
    origin_minco.inner_y.resize(mini_minco.inner_x.size());
    origin_minco.inner_z.resize(mini_minco.inner_x.size());

    for (int i = 0; i < mini_minco.inner_x.size(); i++) {
        origin_minco.inner_x[i] =
            u8_f32(mini_minco.inner_x[i], max_p[0], min_p[0]);
        origin_minco.inner_y[i] =
            u8_f32(mini_minco.inner_y[i], max_p[1], min_p[1]);
        origin_minco.inner_z[i] =
            u8_f32(mini_minco.inner_z[i], max_p[2], min_p[2]);
    }
    origin_minco.start_time = ros::Time::now();
    origin_minco.traj_id = (int)mini_minco.traj_id;
}
#endif
