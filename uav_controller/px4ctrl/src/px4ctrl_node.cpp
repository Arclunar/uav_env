#include <ros/ros.h>
#include "PX4CtrlFSM.h"
#include <signal.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

// ros::Publisher test_freq_pub;
// void timerCallback(const ros::TimerEvent &event)
// {
//     nav_msgs::Odometry test_freq;
//     test_freq.header.stamp = ros::Time(0);
//     test_freq.header.frame_id = "world";

//     static int count = 0;
//     test_freq.twist.twist.linear.x = (count++) % 20;
//     test_freq_pub.publish(test_freq);
// }


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    // ros::Timer timer = nh.createTimer(ros::Duration(0.005), timerCallback);
    // test_freq_pub = nh.advertise<nav_msgs::Odometry>("test_freq", 1);


    

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);

    Controller controller(param);
    PX4CtrlFSM fsm(param, controller);


    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         1000,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
                                         
    // ros::Subscriber simple_odom_sub =
    // nh.subscribe<quadrotor_msgs::SimpleOdom>("simple_odom",
    //                                     100,
    //                                     boost::bind(&Odom_Data_t::feed_simple_odom, &fsm.odom_data, _1),
    //                                     ros::VoidConstPtr(),
    //                                     ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());

    // mandatory stop from bridge
    ros::Subscriber mandatory_stop_sub =
        nh.subscribe<std_msgs::Empty>("/mandatory_stop_from_bridge",
                                     10,
                                     boost::bind(&PX4CtrlFSM::mandatory_stop_bridge_cb, &fsm, _1),
                                        ros::VoidConstPtr(),
                                        ros::TransportHints().tcpNoDelay());

    fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
    
    fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug
    fsm.state_debug_pub = nh.advertise<px4ctrl::fsm_debug>("/px4ctrl/fsm_debug", 100);
    fsm.des_debug_pub = nh.advertise<px4ctrl::des_debug>("/px4ctrl/des_debug",100);
    fsm.set_fix_yaw_cmd_pub_ = nh.advertise<std_msgs::Float64>("/set_fix_yaw_cmd", 10);
    fsm.mandatory_stop_pub = nh.advertise<std_msgs::Empty>("/mandatory_stop", 10);
    fsm.cancel_mandatory_stop_pub = nh.advertise<std_msgs::Empty>("/cancel_mandatory_stop", 10);
    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.led_cmd_pub_ = nh.advertise<std_msgs::String>("/led_controller_in", 10);

    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    fsm.rosbag_record_pub = nh.advertise<std_msgs::Bool>("/rosbag_control",1);
    fsm.ctrl_FCU_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

    fsm.trigger_formation_trans_pub = nh.advertise<std_msgs::Int8>("/formation_trans_trigger",5);

    ros::Duration(0.5).sleep();

    if (param.takeoff_land.no_RC)
    {
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}
