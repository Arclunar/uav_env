#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <ros/ros.h>
#include <ros/assert.h>
#include "px4ctrl/fsm_debug.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <px4ctrl/des_debug.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include "smooth_flight.h"


#include "input.h"
// #include "ThrustCurve.h"
#include "controller.h"

struct AutoTakeoffLand_t
{
	bool landed{true};
	ros::Time toggle_takeoff_land_time;
	std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};
	Eigen::Vector4d start_pose;
	
	static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // motors idle running for 3 seconds before takeoff
	static constexpr double DELAY_TRIGGER_TIME = 2.0;  // Time to be delayed when reach at target height
};

struct SmoothHoverCtrl_t
{
	// control xy position if lock_xy_pos is true
	// neglect xy position error and xy control velocity if lock_xy is false
	// same as lock_height
	bool lock_xy_pos{true};
	bool lock_height{true};
	bool brake{false}; // enter dead zone and keep 0.5 second
	Eigen::Vector3d hover_position{0, 0, 0};
	double yaw;
	Eigen::Vector2d velocity_xy{0, 0};
	double velocity_z = 0;

	// rc hysteresis
	double DEAD_ZONE_HYSTERESIS{0.05};
};

class PX4CtrlFSM
{
public:
	Parameter_t &param;

	RC_Data_t rc_data;
	State_Data_t state_data;
	ExtendedState_Data_t extended_state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Command_Data_t cmd_data;
	Battery_Data_t bat_data;
	Takeoff_Land_Data_t takeoff_land_data;

	Controller &controller;

	ros::Publisher rosbag_record_pub;
	ros::Publisher traj_start_trigger_pub;
	ros::Publisher ctrl_FCU_pub;
	ros::Publisher debug_pub, state_debug_pub,des_debug_pub; //debug
	ros::Publisher ctrl_FCU_pos_pub;
	ros::Publisher set_fix_yaw_cmd_pub_;
	ros::Publisher mandatory_stop_pub;  // for emergency stop trajectory
	ros::Publisher cancel_mandatory_stop_pub;
	ros::Publisher trigger_formation_trans_pub;
	ros::ServiceClient set_FCU_mode_srv;
	ros::ServiceClient arming_client_srv;
	ros::ServiceClient reboot_FCU_srv;
	ros::Publisher led_cmd_pub_;

	quadrotor_msgs::Px4ctrlDebug debug_msg; //debug

	Eigen::Vector4d hover_pose;
	SmoothHoverCtrl_t smooth_hover_ctrl; 	// for smooth hover ctrl
	ros::Time last_set_hover_pose_time;



	enum State_t
	{
		MANUAL_CTRL = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER, // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL,	// px4ctrl is actived, and controling the drone.
		AUTO_TAKEOFF,
		AUTO_LAND
	};

	PX4CtrlFSM(Parameter_t &, Controller &);
	void process();
	bool rc_is_received(const ros::Time &now_time);
	bool cmd_is_received(const ros::Time &now_time);
	bool odom_is_received(const ros::Time &now_time);
	bool imu_is_received(const ros::Time &now_time);
	bool bat_is_received(const ros::Time &now_time);
	bool recv_new_odom();
	State_t get_state() { return state; }
	bool get_landed() { return takeoff_land.landed; }

	void mandatory_stop_bridge_cb(const std_msgs::Empty::ConstPtr &msg);

private:
	State_t state; // Should only be changed in PX4CtrlFSM::process() function!
	AutoTakeoffLand_t takeoff_land;
	bool mandatory_stop_from_bridge_ = false;

	// ---- control related ----
	Desired_State_t get_hover_des();
	Desired_State_t get_hover_des_smooth();
	Desired_State_t get_cmd_des();

	// ---- auto takeoff/land ----
	void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
	void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing 
	void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
	void set_start_xy_for_takeoff_land(const Odom_Data_t &odom);
	Desired_State_t get_rotor_speed_up_des(const ros::Time now);
	Desired_State_t get_takeoff_land_des(const double speed);

	// ---- tools ----
	void set_hov_with_odom();
	void set_hov_with_rc();
	void set_hov_with_rc_smooth();

	bool toggle_offboard_mode(bool on_off); // It will only try to toggle once, so not blocked.
	bool toggle_arm_disarm(bool arm); // It will only try to toggle once, so not blocked.
	void reboot_FCU();

	void publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_trigger(const nav_msgs::Odometry &odom_msg);
	void publish_position_ctrl(const Controller_Position_t &u, const ros::Time &stamp);


	// initial all vaule to -1 means no constraint
	FlightConstraint_t flight_constraint_;
	const FlightConstraint_t NO_FLIGHT_CONSTRAINT;
};

#endif