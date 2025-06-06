#include "input.h"

RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;

    // Parameter initilation is very important in RC-Free usage!
    is_hover_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    toggle_reboot = false;
    toggle_land = false;
    exit_takeoff = false;
    exit_land = false;
    toggle_formation_transition = false;

    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    static ros::Time enter_command_time = ros::Time(0);

    for (int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        // if (ch[i] > DEAD_ZONE)
        //     ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        // else if (ch[i] < -DEAD_ZONE)
        //     ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        // else
        //     ch[i] = 0.0;
    }

    mode = ((double)msg.channels[4] - 1000.0) / 1000.0;  // channel 5 999 1499 1999 -> 0 0.5 1
    gear = ((double)msg.channels[5] - 1000.0) / 1000.0;  // channel 6 999 1999 -> 0  1
    reboot_cmd = ((double)msg.channels[7] - 1000.0) / 1000.0;

    // 0 - 0.5 takeoff 
    // 0.5 - 1 land
    takeoff_land_cmd =  ((double)msg.channels[8] - 1000.0) / 1000.0;

    check_validity();

    if(!have_init_last_takeoff_land_cmd)
    {
        have_init_last_takeoff_land_cmd = true;
        last_takeoff_land_cmd = takeoff_land_cmd;
    }

    // land
    if(is_hover_mode)
    {
        if(last_takeoff_land_cmd > TAKEOFF_SHIFT_VALUE && takeoff_land_cmd < TAKEOFF_SHIFT_VALUE)
        {
            toggle_takeoff = true;
            toggle_land = false;
            exit_takeoff = false;
            exit_land = false;
            // std::cout<<"toggle takeoff \n";
        }
        else if(last_takeoff_land_cmd < TAKEOFF_SHIFT_VALUE && takeoff_land_cmd > TAKEOFF_SHIFT_VALUE)
        {
            toggle_takeoff = false;
            toggle_land = false;
            exit_takeoff = true;
            exit_land = false;
            // std::cout<<"exit takeoff \n";
        }
        else if(last_takeoff_land_cmd < LAND_SHIFT_VALUE && takeoff_land_cmd > LAND_SHIFT_VALUE)
        {
            toggle_land = true;
            toggle_takeoff = false;
            exit_takeoff = false;
            exit_land = false;
            // std::cout<<"toggle land \n";
        }
        else if(last_takeoff_land_cmd > LAND_SHIFT_VALUE && takeoff_land_cmd < LAND_SHIFT_VALUE)
        {
            toggle_land = false;
            toggle_takeoff = false;
            exit_takeoff = false;
            exit_land = true;
        }
        else{
            toggle_takeoff = false;
            toggle_land = false;
            exit_takeoff = false;
            exit_land = false;
        }
    }
    

    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }
    if (!have_init_last_reboot_cmd)
    {
        have_init_last_reboot_cmd = true;
        last_reboot_cmd = reboot_cmd;
    }

    // 1
    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_hover_mode = true;
    else
        enter_hover_mode = false;

    if (mode > API_MODE_THRESHOLD_VALUE)
        is_hover_mode = true;
    else
        is_hover_mode = false;



    // 2
    if (is_hover_mode)
    {
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
        {
            enter_command_time = ros::Time::now();
            enter_command_mode = true;
        }
        else if (gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false; // reset only push back gear , but now i reset it in 1 sec

        if (gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else
            is_command_mode = false;
    }

    // just last for 1 sec
    if(enter_command_mode)
    {
        if((rcv_stamp-enter_command_time).toSec() > 1.0)
            enter_command_mode = false;
    }


    if(last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE)
        toggle_formation_transition = true;
    else
        toggle_formation_transition = false;

    // 3
    if (!is_hover_mode && !is_command_mode)
    {
        if (last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE)
            toggle_reboot = true;
        else
            toggle_reboot = false;
    }
    else
        toggle_reboot = false;

    last_takeoff_land_cmd = takeoff_land_cmd;
    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
}

void RC_Data_t::check_validity()
{
    if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. mode=%f, gear=%f, reboot_cmd=%f", mode, gear, reboot_cmd);
    }
}

bool RC_Data_t::check_centered()
{
    bool centered = fabs(ch[0]) < RC_Data_t::DEAD_ZONE 
    && fabs(ch[1]) < RC_Data_t::DEAD_ZONE 
    && fabs(ch[2]) < RC_Data_t::DEAD_ZONE_THROTTLE 
    && fabs(ch[3]) < RC_Data_t::DEAD_ZONE;
    return centered;
}

Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    q.setIdentity();
    recv_new_msg = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    static ros::Time last_rev_time = ros::Time(0);
    ros::Time now = ros::Time::now();
    ROS_INFO_THROTTLE(1.0,"debug:feed dt : %f", (now - last_rev_time).toSec());
    // std::cout<<"feed dt : "<< (now - last_rev_time).toSec() << std::endl;
    last_rev_time = now;

    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;

    uav_utils::extract_odometry(pMsg, p, v, q, w);

// #define VEL_IN_BODY
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;

    static int count = 0;
    if (count++ % 500 == 0)
        ROS_WARN("VEL_IN_BODY!!!");
#endif

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            ROS_WARN("ODOM frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

void Odom_Data_t::feed_simple_odom(quadrotor_msgs::SimpleOdomConstPtr pMsg)
{
    static ros::Time last_rev_time = ros::Time(0);
    ros::Time now = ros::Time::now();
    ROS_INFO_THROTTLE(1.0,"feed simple dt : %f", (now - last_rev_time).toSec());
    last_rev_time = now;

    // msg = *pMsg;
    msg.header = pMsg->header;
    msg.pose.pose.position.x = pMsg->pose.position.x;
    msg.pose.pose.position.y = pMsg->pose.position.y;
    msg.pose.pose.position.z = pMsg->pose.position.z;
    msg.pose.pose.orientation.x = pMsg->pose.orientation.x;
    msg.pose.pose.orientation.y = pMsg->pose.orientation.y;
    msg.pose.pose.orientation.z = pMsg->pose.orientation.z;
    msg.pose.pose.orientation.w = pMsg->pose.orientation.w;
    msg.twist.twist.linear.x = pMsg->twist.linear.x;
    msg.twist.twist.linear.y = pMsg->twist.linear.y;
    msg.twist.twist.linear.z = pMsg->twist.linear.z;
    msg.twist.twist.angular.x = pMsg->twist.angular.x;
    msg.twist.twist.angular.y = pMsg->twist.angular.y;
    msg.twist.twist.angular.z = pMsg->twist.angular.z;
    rcv_stamp = now;
    recv_new_msg = true;

    // uav_utils::extract_odometry(pMsg, p, v, q, w);
    p(0) = pMsg->pose.position.x;
    p(1) = pMsg->pose.position.y;
    p(2) = pMsg->pose.position.z;
    v(0) = pMsg->twist.linear.x;
    v(1) = pMsg->twist.linear.y;
    v(2) = pMsg->twist.linear.z;
    w(0) = pMsg->twist.angular.x;
    w(1) = pMsg->twist.angular.y;
    w(2) = pMsg->twist.angular.z;
    q.x() = pMsg->pose.orientation.x;
    q.y() = pMsg->pose.orientation.y;
    q.z() = pMsg->pose.orientation.z;
    q.w() = pMsg->pose.orientation.w;

// #define VEL_IN_BODY
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;

    static int count = 0;
    if (count++ % 500 == 0)
        ROS_WARN("VEL_IN_BODY!!!");
#endif

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            ROS_WARN("ODOM frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}


Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            ROS_WARN("IMU frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

State_Data_t::State_Data_t()
{
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{

    current_state = *pMsg;
}

ExtendedState_Data_t::ExtendedState_Data_t()
{
}

void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg)
{
    current_extended_state = *pMsg;
}

Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
}

Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{
    is_init = true;
    static bool first_feed = true;
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    double voltage = 0;
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
    {
        voltage += pMsg->cell_voltage[i];
    }

    if(first_feed)
    {
        volt = voltage;
        first_feed = false;
    }
    else
    {
        volt = 0.8 * volt + 0.2 * voltage; // Naive LPF, cell_voltage has a higher frequency
    }

    // volt = 0.8 * volt + 0.2 * pMsg->voltage; // Naive LPF
    percentage = pMsg->percentage;

    static ros::Time last_print_t = ros::Time(0);
    if (percentage > 0.05)
    {
        if ((rcv_stamp - last_print_t).toSec() > 10)
        {
            ROS_INFO("[px4ctrl] Voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
    else
    {
        if ((rcv_stamp - last_print_t).toSec() > 1)
        {
            ROS_ERROR("[px4ctrl] Dangerous! voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
}

Takeoff_Land_Data_t::Takeoff_Land_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
}
