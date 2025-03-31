#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>

class Parameter_t
{
public:
	struct Gain
	{
		double Kp0, Kp1, Kp2;
		double Kv0, Kv1, Kv2;
		double Kvi0, Kvi1, Kvi2;
		double Kvd0, Kvd1, Kvd2;
		double KAngR, KAngP, KAngY;
		double Kvilim;
	};

	struct RotorDrag
	{
		double x, y, z;
		double k_thrust_horz;
	};

	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct GlobalFlightConstraint
	{
		double max_takeoff_land_tilt_angle;
		double max_takeoff_land_hor_acc;
	};

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
		double hp_in_standard_voltage;
		double hp_in_low_voltage;
		bool noisy_imu;
		bool hover_percentage_inited = false;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};

	Gain gain;
	RotorDrag rt_drag;
	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;
	GlobalFlightConstraint global_flight_constraint;

	int pose_solver;
	double mass;
	double gra;
	double max_angle;
	double ctrl_freq_max;
	double max_manual_vel;
	double low_voltage;
	double standard_voltage;

	bool use_bodyrate_ctrl;
	bool use_position_ctrl;
	bool use_onboard_landing;
	bool use_onboard_takeoff;
	bool use_smooth_hover_ctrl = false;

	// bool print_dbg;

	Parameter_t();
	void config_from_ros_handle(const ros::NodeHandle &nh);
	void config_full_thrust(double hov);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};

#endif