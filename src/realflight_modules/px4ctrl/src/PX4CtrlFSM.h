#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <ros/ros.h>
#include <ros/assert.h>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>

#include "input.h"
#include "localization_health.h"
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
	LocalizationHealth_Data_t localization_health_data;

	LinearControl &controller;

	ros::Publisher traj_start_trigger_pub;
	ros::Publisher ctrl_FCU_pub;
	ros::Publisher debug_pub; //debug
	ros::ServiceClient set_FCU_mode_srv;
	ros::ServiceClient arming_client_srv;
	ros::ServiceClient reboot_FCU_srv;

	quadrotor_msgs::Px4ctrlDebug debug_msg; //debug

	Eigen::Vector4d hover_pose;
	ros::Time last_set_hover_pose_time;

	enum State_t
	{
		MANUAL_CTRL = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
		OFFBOARD_PREP, // healthy hold-setpoint prestream, mode confirmation, and optional arming
		AUTO_HOVER, // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL,	// px4ctrl is actived, and controling the drone.
		AUTO_TAKEOFF,
		AUTO_LAND,
		NORMAL_OFFBOARD_EXIT, // landed and disarmed; wait for non-OFFBOARD feedback without setpoints
		FAILSAFE_EXIT // bounded setpoint handover while PX4 leaves OFFBOARD
	};

	PX4CtrlFSM(Parameter_t &, LinearControl &);
	void process();
	bool rc_is_received(const ros::SteadyTime &now_time);
	bool cmd_is_received(const ros::SteadyTime &now_time);
	bool odom_is_received(const ros::SteadyTime &now_time);
	bool imu_is_received(const ros::SteadyTime &now_time);
	bool bat_is_received(const ros::SteadyTime &now_time);
	bool state_is_received(const ros::SteadyTime &now_time);
	bool extended_state_is_received(const ros::SteadyTime &now_time);
	bool recv_new_odom();
	State_t get_state() { return state; }
	bool get_landed() { return takeoff_land.landed; }

private:
	State_t state; // Should only be changed in PX4CtrlFSM::process() function!
	State_t pending_offboard_state{AUTO_HOVER};
	AutoTakeoffLand_t takeoff_land;
	bool landing_disarm_message_pending{true};
	bool localization_fault_latched{false};
	bool offboard_mode_confirmed{false};
	bool mode_request_attempted{false};
	bool mode_request_sent{false}; // MAVROS/PX4 acknowledged at least one request.
	bool arm_request_attempted{false};
	bool arm_request_accepted{false};
	Eigen::Vector3d arm_request_position{Eigen::Vector3d::Zero()};
	ros::SteadyTime offboard_prepare_start;
	ros::SteadyTime offboard_confirmed_time;
	ros::SteadyTime last_mode_request_time;
	ros::SteadyTime last_arm_request_time;

	Controller_Output_t last_control_output;
	bool last_control_output_valid{false};
	Controller_Output_t failsafe_control_output;
	bool failsafe_control_output_valid{false};
	Eigen::Quaterniond failsafe_imu_attitude{Eigen::Quaterniond::Identity()};
	bool failsafe_imu_attitude_valid{false};
	ros::SteadyTime failsafe_start;
	ros::SteadyTime failsafe_last_mode_request;
	bool failsafe_mode_request_sent{false};
	bool failsafe_stream_timeout_reported{false};
	std::string failsafe_target_mode;
	ros::SteadyTime normal_exit_start;
	ros::SteadyTime normal_exit_last_mode_request;
	bool normal_exit_mode_request_accepted{false};
	std::string normal_exit_target_mode;

	// ---- control related ----
	Desired_State_t get_hover_des();
	Desired_State_t get_cmd_des();

	// ---- auto takeoff/land ----
	void prepare_arm_wait_output(const Imu_Data_t &imu, Controller_Output_t &u);
	void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
	void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing 
	void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
	Desired_State_t get_rotor_speed_up_des(const ros::Time now);
	Desired_State_t get_takeoff_land_des(const double speed);

	// ---- tools ----
	void set_hov_with_odom();
	void set_hov_with_rc();
	bool control_inputs_ready(const ros::SteadyTime &now_time, std::string *reason);
	bool is_active_control_state(State_t candidate) const;
	void start_offboard_preparation(State_t target, const ros::SteadyTime &now_time);
	void start_normal_offboard_exit(const ros::SteadyTime &now_time,
									const std::string &target_mode);
	void process_normal_offboard_exit(const ros::SteadyTime &now_time);
	void start_failsafe_exit(const ros::SteadyTime &now_time, const std::string &reason,
							 bool latch_fault, const std::string &target_mode);
	bool prepare_failsafe_output(const ros::SteadyTime &now_time, Controller_Output_t &u);
	bool validate_control_output(Controller_Output_t &u) const;
	std::string localization_failsafe_mode(const ros::SteadyTime &now_time) const;
	std::string previous_mode_or_failsafe(const ros::SteadyTime &now_time) const;
	void clear_input_flags();

	bool request_fcu_mode(const std::string &mode); // Request acceptance is not mode confirmation.
	bool toggle_arm_disarm(bool arm); // It will only try to toggle once, so not blocked.
	void reboot_FCU();

	void publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_trigger(const nav_msgs::Odometry &odom_msg);
};

#endif
