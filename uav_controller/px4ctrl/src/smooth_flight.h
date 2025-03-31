#ifndef SMOOTH_FLIGHT_H
#define SMOOTH_FLIGHT_H


struct FlightConstraint_t
{
    double max_acc_xy;
    double max_tilt;
    bool control_xy_pos;
    bool control_height;

    // 默认构造函数
    FlightConstraint_t() : max_acc_xy(-1), max_tilt(-1) 
    {
        control_xy_pos = true;
        control_height = true;
    }

    // 拷贝构造函数
    FlightConstraint_t(const FlightConstraint_t& other)
        : max_acc_xy(other.max_acc_xy), max_tilt(other.max_tilt), 
        control_xy_pos(other.control_xy_pos), 
        control_height(other.control_height) {}

    // 赋值运算符重载
    FlightConstraint_t& operator=(const FlightConstraint_t& other)
    {
        if (this != &other) { // 防止自我赋值
            max_acc_xy = other.max_acc_xy;
            max_tilt = other.max_tilt;
            control_xy_pos = other.control_xy_pos;
            control_height = other.control_height;
        }
        return *this;
    }
};



#endif // SMOOTH_FLIGHT_H