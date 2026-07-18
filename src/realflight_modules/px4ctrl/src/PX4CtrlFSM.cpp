#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

#include <algorithm>
#include <cmath>

using namespace std;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_) : param(param_), controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
	localization_health_data.configure(param.localization_health.freshness_timeout,
										 param.localization_health.recovery_duration);
	ROS_WARN("[px4ctrl] Localization interlock enabled: heartbeat timeout %.2fs, "
			 "failsafe modes RC='%s', no-RC='%s'. Verify PX4 offboard-loss parameters.",
			 param.localization_health.freshness_timeout,
			 param.localization_health.failsafe_mode_with_rc.c_str(),
			 param.localization_health.failsafe_mode_without_rc.c_str());
}

/* 
        Finite State Machine

	      system start
	            |
	            |
	            v
	----- > MANUAL_CTRL <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > AUTO_TAKEOFF  |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       AUTO_HOVER <                 |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > AUTO_LAND -------
	|         |   |
	|         |   v
	-------- CMD_CTRL

*/

void PX4CtrlFSM::process()
{
	const ros::Time now_time = ros::Time::now();
	const ros::SteadyTime steady_now = ros::SteadyTime::now();
	Controller_Output_t u;
	Desired_State_t des;
	bool have_desired_state = false;
	bool rotor_low_speed_during_land = false;

	// A fault can only clear after the aircraft is disarmed, PX4 has left
	// OFFBOARD, and the upstream bridge has completed a new healthy interval.
	if (state == MANUAL_CTRL && localization_fault_latched &&
		state_is_received(steady_now) && state_data.current_state.connected &&
		!state_data.current_state.mode.empty() && !state_data.current_state.armed &&
		state_data.current_state.mode != "OFFBOARD" &&
		localization_health_data.control_allowed(steady_now))
	{
		localization_fault_latched = false;
		ROS_WARN("[px4ctrl] Localization interlock re-armed after disarm and healthy recovery.");
	}

	// Every active OFFBOARD state shares the same fail-closed guard.  In
	// particular AUTO_TAKEOFF can no longer continue on pure IMU odometry after
	// LiDAR corrections disappear.
	if (is_active_control_state(state))
	{
		std::string reason;
		if (!control_inputs_ready(steady_now, &reason))
		{
			start_failsafe_exit(steady_now, reason, true,
							localization_failsafe_mode(steady_now));
		}
		else if (state_data.current_state.mode != "OFFBOARD")
		{
			ROS_WARN("[px4ctrl] PX4 left OFFBOARD; stopping external setpoints immediately.");
			state = MANUAL_CTRL;
			// Do not let an RC edge or takeoff/land message received in this
			// callback immediately start a new OFFBOARD entry.  A subsequent,
			// explicit command must be observed after the mode-exit feedback.
			if (!state_data.current_state.armed)
				takeoff_land.landed = true;
			clear_input_flags();
			return;
		}
		else if (!state_data.current_state.armed)
		{
			const bool expected_landing_disarm =
				state == AUTO_LAND && get_landed() &&
				extended_state_is_received(steady_now) &&
				extended_state_data.current_extended_state.landed_state ==
					mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
			if (!expected_landing_disarm)
			{
				start_failsafe_exit(steady_now,
								"vehicle disarmed unexpectedly while OFFBOARD was active", true,
								previous_mode_or_failsafe(steady_now));
			}
		}
		else if (!param.takeoff_land.no_RC && !rc_is_received(steady_now))
		{
			start_failsafe_exit(steady_now, "RC heartbeat timed out", true,
							localization_failsafe_mode(steady_now));
		}
		else if (!param.takeoff_land.no_RC && !rc_data.is_hover_mode)
		{
			start_failsafe_exit(steady_now, "pilot requested manual control", false,
							previous_mode_or_failsafe(steady_now));
		}
	}

	switch (state)
	{
	case MANUAL_CTRL:
	{
		if (rc_data.enter_hover_mode)
		{
			std::string reason;
			if (!control_inputs_ready(steady_now, &reason))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER: %s.", reason.c_str());
				break;
			}
			if (!param.takeoff_land.no_RC && !rc_is_received(steady_now))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER: RC sample is stale or invalid.");
				break;
			}
			if (state_data.current_state.mode == "OFFBOARD")
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER: PX4 is already in OFFBOARD.");
				break;
			}
			if (!state_data.current_state.armed)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER: arm the vehicle before requesting hover.");
				break;
			}
			if (cmd_is_received(steady_now))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER: stop PositionCommand before entering hover.");
				break;
			}
			if (odom_data.v.norm() > 3.0)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER: odometry velocity %.3f m/s is unsafe.",
						  odom_data.v.norm());
				break;
			}

			start_offboard_preparation(AUTO_HOVER, steady_now);
			des = get_hover_des();
			have_desired_state = true;
			ROS_INFO("[px4ctrl] Starting healthy OFFBOARD prestream for AUTO_HOVER.");
		}
		else if (param.takeoff_land.enable && takeoff_land_data.triggered &&
				 takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF)
		{
			std::string reason;
			if (!control_inputs_ready(steady_now, &reason))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF: %s.", reason.c_str());
				break;
			}
			if (state_data.current_state.mode == "OFFBOARD")
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF: PX4 is already in OFFBOARD.");
				break;
			}
			if (cmd_is_received(steady_now))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF: stop PositionCommand first.");
				break;
			}
			if (odom_data.v.norm() > 0.1)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF: odometry velocity %.3f m/s is not static.",
						  odom_data.v.norm());
				break;
			}
			if (!get_landed())
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF: land detector does not report landed.");
				break;
			}
			if (!extended_state_is_received(steady_now) ||
				extended_state_data.current_extended_state.landed_state !=
					mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF: fresh PX4 on-ground confirmation is required.");
				break;
			}
			if (!param.takeoff_land.enable_auto_arm && !state_data.current_state.armed)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF: auto-arm is disabled; arm the vehicle first.");
				break;
			}
			if (!param.takeoff_land.no_RC &&
				(!rc_is_received(steady_now) || !rc_data.is_hover_mode ||
				 !rc_data.is_command_mode || !rc_data.check_centered()))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF: RC must be fresh, in hover/command "
						  "mode, with all sticks centered.");
				break;
			}

			start_offboard_preparation(AUTO_TAKEOFF, steady_now);
			des = get_hover_des();
			have_desired_state = true;
			ROS_INFO("[px4ctrl] Starting healthy OFFBOARD prestream for AUTO_TAKEOFF.");
		}

		if (state == MANUAL_CTRL && rc_data.toggle_reboot)
		{
			if (state_data.current_state.armed)
			{
				ROS_ERROR("[px4ctrl] Reject reboot: disarm the vehicle first.");
			}
			else
			{
				reboot_FCU();
			}
		}
		break;
	}

	case OFFBOARD_PREP:
	{
		std::string reason;
		if (!control_inputs_ready(steady_now, &reason))
		{
			localization_fault_latched = true;
			if (!mode_request_attempted && state_is_received(steady_now) &&
				state_data.current_state.mode != "OFFBOARD")
			{
				ROS_ERROR("[px4ctrl] OFFBOARD preparation aborted: %s.", reason.c_str());
				state = MANUAL_CTRL;
			}
			else
			{
				// Once an OFFBOARD request has been sent it may still be accepted
				// after this callback.  Keep a bounded handover stream until a
				// subsequent state message proves PX4 is in a safe mode.
				start_failsafe_exit(steady_now, reason, true,
								localization_failsafe_mode(steady_now));
			}
			break;
		}

		if (!param.takeoff_land.no_RC &&
			(!rc_is_received(steady_now) || !rc_data.is_hover_mode ||
			 (pending_offboard_state == AUTO_TAKEOFF && !rc_data.is_command_mode)))
		{
			const bool rc_lost = !rc_is_received(steady_now);
			if (state_data.current_state.mode == "OFFBOARD" || mode_request_attempted)
			{
				start_failsafe_exit(steady_now,
								rc_lost ? "RC heartbeat timed out during OFFBOARD entry"
											: "pilot cancelled OFFBOARD entry",
								rc_lost, rc_lost ? localization_failsafe_mode(steady_now)
												 : previous_mode_or_failsafe(steady_now));
			}
			else
			{
				localization_fault_latched = localization_fault_latched || rc_lost;
				state = MANUAL_CTRL;
				ROS_WARN("[px4ctrl] OFFBOARD preparation cancelled before mode entry.");
			}
			break;
		}

		const bool pending_state_requires_armed =
			pending_offboard_state == AUTO_HOVER ||
			(pending_offboard_state == AUTO_TAKEOFF &&
			 !param.takeoff_land.enable_auto_arm);
		if (pending_state_requires_armed && !state_data.current_state.armed)
		{
			if (mode_request_attempted || state_data.current_state.mode == "OFFBOARD")
			{
				start_failsafe_exit(steady_now,
								"vehicle disarmed during OFFBOARD preparation", true,
								previous_mode_or_failsafe(steady_now));
			}
			else
			{
				localization_fault_latched = true;
				state = MANUAL_CTRL;
				ROS_ERROR("[px4ctrl] OFFBOARD preparation aborted because the vehicle disarmed.");
			}
			break;
		}

		if (pending_offboard_state == AUTO_TAKEOFF)
		{
			const bool takeoff_condition_lost =
				odom_data.v.norm() > 0.1 || !get_landed() ||
				!extended_state_is_received(steady_now) ||
				extended_state_data.current_extended_state.landed_state !=
					mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND ||
				(!param.takeoff_land.no_RC && !rc_data.check_centered());
			if (takeoff_condition_lost)
			{
				if (mode_request_attempted || state_data.current_state.mode == "OFFBOARD")
				{
					start_failsafe_exit(steady_now,
									"takeoff safety condition changed during OFFBOARD entry", true,
									localization_failsafe_mode(steady_now));
				}
				else
				{
					localization_fault_latched = true;
					state = MANUAL_CTRL;
					ROS_ERROR("[px4ctrl] AUTO_TAKEOFF preparation aborted: vehicle moved, "
							  "landed state changed, or RC sticks left center.");
				}
				break;
			}
		}

		const double elapsed = (steady_now - offboard_prepare_start).toSec();
		if (state_data.current_state.mode != "OFFBOARD")
		{
			if (offboard_mode_confirmed)
			{
				if (!state_data.current_state.mode.empty())
				{
					ROS_WARN("[px4ctrl] PX4 left OFFBOARD during entry; stopping setpoints.");
					state = MANUAL_CTRL;
				}
				else
				{
					start_failsafe_exit(steady_now, "PX4 mode became unknown during entry", true,
									localization_failsafe_mode(steady_now));
				}
				break;
			}

			// Follow the current valid pose during prestream.  The target is locked
			// only when PX4 confirms OFFBOARD, avoiding a one-second position step.
			set_hov_with_odom();
			des = get_hover_des();
			have_desired_state = true;

			if (elapsed >= param.localization_health.offboard_entry_timeout)
			{
				start_failsafe_exit(steady_now,
								"OFFBOARD entry timed out after a mode request", true,
								localization_failsafe_mode(steady_now));
				have_desired_state = false;
				break;
			}

			if (elapsed >= param.localization_health.offboard_prestream_time &&
				(!mode_request_attempted ||
				 (steady_now - last_mode_request_time).toSec() >=
					 param.localization_health.mode_retry_interval))
			{
				mode_request_attempted = true;
				if (request_fcu_mode("OFFBOARD"))
					mode_request_sent = true;
				last_mode_request_time = steady_now;
			}
			break;
		}

		// Only a state update observed after this controller's own mode
		// request can confirm entry.  This prevents an external mode change or
		// a delayed pre-request State sample from bypassing the required
		// one-second prestream and handover sequence.
		if (!mode_request_sent)
		{
			start_failsafe_exit(steady_now,
							"PX4 entered OFFBOARD before px4ctrl requested it", true,
							localization_failsafe_mode(steady_now));
			break;
		}
		if (state_data.rcv_steady_stamp <= last_mode_request_time)
		{
			set_hov_with_odom();
			des = get_hover_des();
			have_desired_state = true;
			if (elapsed >= param.localization_health.offboard_entry_timeout)
			{
				start_failsafe_exit(steady_now,
								"OFFBOARD feedback was not newer than px4ctrl's mode request", true,
								localization_failsafe_mode(steady_now));
				have_desired_state = false;
			}
			break;
		}

		if (!offboard_mode_confirmed)
		{
			offboard_mode_confirmed = true;
			offboard_confirmed_time = steady_now;
			set_hov_with_odom();
			ROS_INFO("[px4ctrl] PX4 confirmed OFFBOARD.");
		}

		des = get_hover_des();
		have_desired_state = true;

		if (pending_offboard_state == AUTO_TAKEOFF &&
			param.takeoff_land.enable_auto_arm && !state_data.current_state.armed)
		{
			if ((steady_now - offboard_confirmed_time).toSec() >=
				param.localization_health.arm_confirmation_timeout)
			{
				start_failsafe_exit(steady_now, "arming was not confirmed", true,
								localization_failsafe_mode(steady_now));
				break;
			}
			if (!arm_request_sent ||
				(steady_now - last_arm_request_time).toSec() >=
					param.localization_health.mode_retry_interval)
			{
				toggle_arm_disarm(true);
				arm_request_sent = true;
				last_arm_request_time = steady_now;
			}
			break;
		}

		if (pending_offboard_state == AUTO_TAKEOFF)
		{
			state = AUTO_TAKEOFF;
			takeoff_land.landed = false;
			set_start_pose_for_takeoff_land(odom_data);
			des = get_rotor_speed_up_des(now_time);
			ROS_INFO("\033[32m[px4ctrl] OFFBOARD confirmed --> AUTO_TAKEOFF\033[0m");
		}
		else
		{
			state = AUTO_HOVER;
			takeoff_land.landed = false;
			des = get_hover_des();
			ROS_INFO("\033[32m[px4ctrl] OFFBOARD confirmed --> AUTO_HOVER(L2)\033[0m");
		}
		break;
	}

	case AUTO_HOVER:
	{
		if (rc_data.is_command_mode && cmd_is_received(steady_now))
		{
			state = CMD_CTRL;
			des = get_cmd_des();
			ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[0m");
		}
		else if (takeoff_land_data.triggered &&
				 takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{
			state = AUTO_LAND;
			set_start_pose_for_takeoff_land(odom_data);
			des = get_takeoff_land_des(-param.takeoff_land.speed);
			ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[0m");
		}
		else
		{
			set_hov_with_rc();
			des = get_hover_des();
			if (rc_data.enter_command_mode ||
				(takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
			{
				takeoff_land.delay_trigger.first = false;
				publish_trigger(odom_data.msg);
				ROS_INFO("[px4ctrl] TRIGGER sent; user commands are now allowed.");
			}
		}
		have_desired_state = true;
		break;
	}

	case CMD_CTRL:
	{
		if (!rc_data.is_command_mode || !cmd_is_received(steady_now))
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[px4ctrl] CMD_CTRL(L3) --> AUTO_HOVER(L2).");
		}
		else
		{
			des = get_cmd_des();
		}
		have_desired_state = true;

		if (takeoff_land_data.triggered &&
			takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{
			ROS_ERROR("[px4ctrl] Reject AUTO_LAND in CMD_CTRL; stop commands for %.2fs first.",
					  param.msg_timeout.cmd);
		}
		break;
	}

	case AUTO_TAKEOFF:
	{
		if (!state_data.current_state.armed)
		{
			start_failsafe_exit(steady_now, "vehicle disarmed during AUTO_TAKEOFF", true,
							localization_failsafe_mode(steady_now));
			break;
		}
		if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() <
			AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME)
		{
			des = get_rotor_speed_up_des(now_time);
		}
		else if (odom_data.p(2) >= takeoff_land.start_pose(2) + param.takeoff_land.height)
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			takeoff_land.delay_trigger.first = true;
			takeoff_land.delay_trigger.second =
				now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
			ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[0m");
		}
		else
		{
			des = get_takeoff_land_des(param.takeoff_land.speed);
		}
		have_desired_state = true;
		break;
	}

	case AUTO_LAND:
	{
		if (!rc_data.is_command_mode)
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			have_desired_state = true;
			ROS_INFO("[px4ctrl] AUTO_LAND --> AUTO_HOVER(L2).");
		}
		else if (!get_landed())
		{
			des = get_takeoff_land_des(-param.takeoff_land.speed);
			have_desired_state = true;
		}
		else if (!extended_state_is_received(steady_now) ||
				 extended_state_data.current_extended_state.landed_state !=
					 mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
		{
			// Keep the controlled descent until an independent, fresh PX4
			// land-detector report agrees.  A stale startup ON_GROUND value must
			// never authorize motor idle/disarm in the air.
			des = get_takeoff_land_des(-param.takeoff_land.speed);
			have_desired_state = true;
			ROS_WARN_THROTTLE(1.0,
						  "[px4ctrl] Internal landing detected; waiting for fresh PX4 on-ground confirmation.");
		}
		else
		{
			rotor_low_speed_during_land = true;
			have_desired_state = true;

			static bool print_once_flag = true;
			if (print_once_flag)
			{
				ROS_INFO("[px4ctrl] Internal and PX4 landing detectors agree; requesting disarm.");
				print_once_flag = false;
			}

			static double last_trial_time = 0.0;
			if (state_data.current_state.armed && now_time.toSec() - last_trial_time > 1.0)
			{
				toggle_arm_disarm(false);
				last_trial_time = now_time.toSec();
			}
			else if (!state_data.current_state.armed)
			{
				print_once_flag = true;
				start_failsafe_exit(steady_now, "landing complete", false,
								previous_mode_or_failsafe(steady_now));
			}
		}
		break;
	}

	case FAILSAFE_EXIT:
		break;

	default:
		ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Unknown FSM state; forcing MANUAL_CTRL.");
		state = MANUAL_CTRL;
		break;
	}

	// FAILSAFE_EXIT never evaluates a position controller.  It may forward a
	// bounded snapshot briefly while repeatedly requesting a non-OFFBOARD mode.
	if (state == FAILSAFE_EXIT)
	{
		if (prepare_failsafe_output(steady_now, u))
		{
			if (param.use_bodyrate_ctrl)
				publish_bodyrate_ctrl(u, now_time);
			else
				publish_attitude_ctrl(u, now_time);
		}
		clear_input_flags();
		return;
	}

	// MANUAL_CTRL sends no external setpoints.  OFFBOARD prestreaming is
	// explicit in OFFBOARD_PREP, so a stale/uninitialized odometry sample can no
	// longer be fed through the controller at startup.
	if (!have_desired_state)
	{
		if (state == MANUAL_CTRL && !state_data.current_state.armed)
			takeoff_land.landed = true;
		clear_input_flags();
		return;
	}

	if (state == AUTO_HOVER || state == CMD_CTRL)
	{
		controller.estimateThrustModel(imu_data.a, param);
	}

	if (rotor_low_speed_during_land)
	{
		motors_idling(imu_data, u);
	}
	else
	{
		debug_msg = controller.calculateControl(des, odom_data, imu_data, u);
		debug_msg.header.stamp = now_time;
		debug_pub.publish(debug_msg);
	}

	if (!validate_control_output(u))
	{
		start_failsafe_exit(steady_now, "controller produced an invalid output", true,
							localization_failsafe_mode(steady_now));
		if (prepare_failsafe_output(steady_now, u))
		{
			if (param.use_bodyrate_ctrl)
				publish_bodyrate_ctrl(u, now_time);
			else
				publish_attitude_ctrl(u, now_time);
		}
		clear_input_flags();
		return;
	}

	if (param.use_bodyrate_ctrl)
		publish_bodyrate_ctrl(u, now_time);
	else
		publish_attitude_ctrl(u, now_time);

	last_control_output = u;
	last_control_output_valid = true;

	if (is_active_control_state(state))
		land_detector(state, des, odom_data);

	clear_input_flags();
}

void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
	u.q = imu.q;
	u.bodyrates = Eigen::Vector3d::Zero();
	u.thrust = 0.04;
}

void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
	static State_t last_state = State_t::MANUAL_CTRL;
	if (last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
	{
		takeoff_land.landed = false; // Always holds
	}
	last_state = state;

	if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed)
	{
		takeoff_land.landed = true;
		return; // No need of other decisions
	}

	// land_detector parameters
	constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
	constexpr double VELOCITY_THR_C = 0.1;		  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
	constexpr double TIME_KEEP_C = 3.0;			  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

	static ros::Time time_C12_reached; // time_Constraints12_reached
	static bool is_last_C12_satisfy;
	if (takeoff_land.landed)
	{
		time_C12_reached = ros::Time::now();
		is_last_C12_satisfy = false;
	}
	else
	{
		bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
		if (C12_satisfy && !is_last_C12_satisfy)
		{
			time_C12_reached = ros::Time::now();
		}
		else if (C12_satisfy && is_last_C12_satisfy)
		{
			if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) //Constraint 3 reached
			{
				takeoff_land.landed = true;
			}
		}

		is_last_C12_satisfy = C12_satisfy;
	}
}

Desired_State_t PX4CtrlFSM::get_hover_des()
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.a = cmd_data.a;
	des.j = cmd_data.j;
	des.yaw = cmd_data.yaw;
	des.yaw_rate = cmd_data.yaw_rate;

	return des;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const ros::Time now)
{
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
	double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
	if (des_a_z > 0.1)
	{
		ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
		des_a_z = 0.0;
	}

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d(0, 0, des_a_z);
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed)
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff
	// takeoff_land.last_set_cmd_time = now;

	// takeoff_land.start_pose(2) += speed * delta_t;

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
	des.v = Eigen::Vector3d(0, 0, speed);
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

void PX4CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

	last_set_hover_pose_time = ros::Time::now();
}

void PX4CtrlFSM::set_hov_with_rc()
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - last_set_hover_pose_time).toSec();
	last_set_hover_pose_time = now;

	hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
	hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
	hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
	hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

	if (hover_pose(2) < -0.3)
		hover_pose(2) = -0.3;

	// if (param.print_dbg)
	// {
	// 	static unsigned int count = 0;
	// 	if (count++ % 100 == 0)
	// 	{
	// 		cout << "hover_pose=" << hover_pose.transpose() << endl;
	// 		cout << "ch[0~3]=" << rc_data.ch[0] << " " << rc_data.ch[1] << " " << rc_data.ch[2] << " " << rc_data.ch[3] << endl;
	// 	}
	// }
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
	takeoff_land.start_pose.head<3>() = odom.p;
	takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom.q);

	takeoff_land.toggle_takeoff_land_time = ros::Time::now();
}

bool PX4CtrlFSM::control_inputs_ready(const ros::SteadyTime &now_time,
									std::string *reason)
{
	if (localization_fault_latched)
	{
		if (reason)
			*reason = "localization/control fault is latched until disarm";
		return false;
	}
	if (!localization_health_data.control_allowed(now_time))
	{
		if (reason)
			*reason = "localization health heartbeat is false, stale, or not yet qualified";
		return false;
	}
	if (!odom_is_received(now_time))
	{
		if (reason)
			*reason = "odometry heartbeat timed out";
		return false;
	}
	if (!odom_data.data_valid)
	{
		if (reason)
			*reason = "odometry contains non-finite data or an invalid quaternion";
		return false;
	}
	if (!imu_is_received(now_time))
	{
		if (reason)
			*reason = "FCU IMU heartbeat timed out";
		return false;
	}
	if (!imu_data.data_valid)
	{
		if (reason)
			*reason = "FCU IMU contains non-finite data or an invalid quaternion";
		return false;
	}
	if (!state_is_received(now_time))
	{
		if (reason)
			*reason = "MAVROS state heartbeat timed out";
		return false;
	}
	if (!state_data.current_state.connected)
	{
		if (reason)
			*reason = "FCU is disconnected";
		return false;
	}
	if (state_data.current_state.mode.empty())
	{
		if (reason)
			*reason = "FCU mode feedback is empty";
		return false;
	}
	return true;
}

bool PX4CtrlFSM::is_active_control_state(State_t candidate) const
{
	return candidate == AUTO_HOVER || candidate == CMD_CTRL ||
		   candidate == AUTO_TAKEOFF || candidate == AUTO_LAND;
}

void PX4CtrlFSM::start_offboard_preparation(State_t target,
											const ros::SteadyTime &now_time)
{
	pending_offboard_state = target;
	state_data.state_before_offboard = state_data.current_state;
	if (state_data.state_before_offboard.mode.empty() ||
		state_data.state_before_offboard.mode == "OFFBOARD")
	{
		state_data.state_before_offboard.mode = localization_failsafe_mode(now_time);
	}

	controller.resetThrustMapping();
	set_hov_with_odom();
	offboard_prepare_start = now_time;
	offboard_confirmed_time = ros::SteadyTime();
	last_mode_request_time = ros::SteadyTime();
	last_arm_request_time = ros::SteadyTime();
	mode_request_sent = false;
	mode_request_attempted = false;
	arm_request_sent = false;
	offboard_mode_confirmed = false;
	last_control_output_valid = false;
	state = OFFBOARD_PREP;
}

void PX4CtrlFSM::start_failsafe_exit(const ros::SteadyTime &now_time,
									 const std::string &reason, bool latch_fault,
									 const std::string &target_mode)
{
	if (state == FAILSAFE_EXIT)
		return;

	localization_fault_latched = localization_fault_latched || latch_fault;
	failsafe_start = now_time;
	failsafe_last_mode_request = ros::SteadyTime();
	failsafe_mode_request_sent = false;
	failsafe_stream_timeout_reported = false;
	failsafe_target_mode = target_mode;
	if (failsafe_target_mode.empty() || failsafe_target_mode == "OFFBOARD")
		failsafe_target_mode = localization_failsafe_mode(now_time);

	failsafe_control_output = last_control_output;
	failsafe_control_output_valid = last_control_output_valid;
	failsafe_imu_attitude_valid = imu_is_received(now_time) && imu_data.data_valid;
	if (failsafe_imu_attitude_valid)
		failsafe_imu_attitude = imu_data.q;

	state = FAILSAFE_EXIT;
	ROS_ERROR("[px4ctrl] Enter FAILSAFE_EXIT: %s. Requesting '%s'; setpoint stream "
			  "is bounded to %.2fs.",
			  reason.c_str(), failsafe_target_mode.c_str(),
			  param.localization_health.exit_stream_grace);
}

bool PX4CtrlFSM::prepare_failsafe_output(const ros::SteadyTime &now_time,
										 Controller_Output_t &u)
{
	// A fresh MAVROS state is the only confirmation that mode handover is done.
	if (state_is_received(now_time) && state_data.rcv_steady_stamp > failsafe_start &&
		state_data.current_state.connected &&
		!state_data.current_state.mode.empty() &&
		state_data.current_state.mode != "OFFBOARD")
	{
		ROS_WARN("[px4ctrl] PX4 confirmed mode '%s'; external setpoints stopped.",
				 state_data.current_state.mode.c_str());
		state = MANUAL_CTRL;
		return false;
	}

	if (!failsafe_mode_request_sent ||
		(now_time - failsafe_last_mode_request).toSec() >=
			param.localization_health.mode_retry_interval)
	{
		request_fcu_mode(failsafe_target_mode);
		failsafe_mode_request_sent = true;
		failsafe_last_mode_request = now_time;
	}

	const double elapsed = (now_time - failsafe_start).toSec();
	if (elapsed >= param.localization_health.exit_stream_grace)
	{
		if (!failsafe_stream_timeout_reported)
		{
			ROS_ERROR("[px4ctrl] PX4 did not confirm OFFBOARD exit within %.2fs; "
					  "stopping setpoints to trigger the onboard offboard-loss policy.",
					  param.localization_health.exit_stream_grace);
			failsafe_stream_timeout_reported = true;
		}
		return false;
	}

	if (!failsafe_control_output_valid)
	{
		ROS_ERROR_THROTTLE(1.0, "[px4ctrl] No valid published output is available for handover; "
							 "setpoints remain stopped.");
		return false;
	}

	u = failsafe_control_output;
	const double blend_time = param.localization_health.attitude_blend_time;
	const double alpha = blend_time <= 0.0 ? 1.0 :
		std::max(0.0, std::min(1.0, elapsed / blend_time));
	if (failsafe_imu_attitude_valid)
		u.q = failsafe_control_output.q.slerp(alpha, failsafe_imu_attitude);
	u.bodyrates = (1.0 - alpha) * failsafe_control_output.bodyrates;

	return validate_control_output(u);
}

bool PX4CtrlFSM::validate_control_output(Controller_Output_t &u) const
{
	if (!u.q.coeffs().allFinite() || !u.bodyrates.allFinite() ||
		!std::isfinite(u.thrust))
		return false;

	const double q_norm = u.q.norm();
	if (!std::isfinite(q_norm) || q_norm < 1.0e-6)
		return false;

	u.q.normalize();
	u.thrust = std::max(0.0, std::min(1.0, u.thrust));
	if (param.max_angle >= 0.0)
	{
		const Eigen::Vector3d body_z = u.q * Eigen::Vector3d::UnitZ();
		if (!body_z.allFinite())
			return false;
		const double vertical_component = std::max(-1.0, std::min(1.0, body_z.z()));
		const double tilt = std::acos(vertical_component);
		// The controller clamps its requested tilt exactly.  A small margin
		// accommodates estimator/rounding differences; a larger discrepancy
		// means the FCU IMU and localization attitude no longer agree.
		if (tilt > param.max_angle + 0.02)
			return false;
	}
	return true;
}

std::string PX4CtrlFSM::localization_failsafe_mode(const ros::SteadyTime &now_time) const
{
	const bool rc_fresh = rc_data.data_valid && now_time >= rc_data.rcv_steady_stamp &&
		(now_time - rc_data.rcv_steady_stamp).toSec() < param.msg_timeout.rc;
	return (!param.takeoff_land.no_RC && rc_fresh)
			   ? param.localization_health.failsafe_mode_with_rc
			   : param.localization_health.failsafe_mode_without_rc;
}

std::string PX4CtrlFSM::previous_mode_or_failsafe(const ros::SteadyTime &now_time) const
{
	const std::string &previous = state_data.state_before_offboard.mode;
	if (!previous.empty() && previous != "OFFBOARD")
		return previous;
	return localization_failsafe_mode(now_time);
}

void PX4CtrlFSM::clear_input_flags()
{
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	rc_data.toggle_reboot = false;
	takeoff_land_data.triggered = false;
}

bool PX4CtrlFSM::rc_is_received(const ros::SteadyTime &now_time)
{
	return rc_data.data_valid && now_time >= rc_data.rcv_steady_stamp &&
		(now_time - rc_data.rcv_steady_stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::SteadyTime &now_time)
{
	return cmd_data.data_valid && now_time >= cmd_data.rcv_steady_stamp &&
		(now_time - cmd_data.rcv_steady_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::SteadyTime &now_time)
{
	return now_time >= odom_data.rcv_steady_stamp &&
		(now_time - odom_data.rcv_steady_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::SteadyTime &now_time)
{
	return now_time >= imu_data.rcv_steady_stamp &&
		(now_time - imu_data.rcv_steady_stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const ros::SteadyTime &now_time)
{
	return now_time >= bat_data.rcv_steady_stamp &&
		(now_time - bat_data.rcv_steady_stamp).toSec() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::state_is_received(const ros::SteadyTime &now_time)
{
	return now_time >= state_data.rcv_steady_stamp &&
		(now_time - state_data.rcv_steady_stamp).toSec() < param.msg_timeout.state;
}

bool PX4CtrlFSM::extended_state_is_received(const ros::SteadyTime &now_time)
{
	return now_time >= extended_state_data.rcv_steady_stamp &&
		(now_time - extended_state_data.rcv_steady_stamp).toSec() <
			param.msg_timeout.extended_state;
}

bool PX4CtrlFSM::recv_new_odom()
{
	if (odom_data.recv_new_msg)
	{
		odom_data.recv_new_msg = false;
		return true;
	}

	return false;
}

void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

	msg.body_rate.x = u.bodyrates.x();
	msg.body_rate.y = u.bodyrates.y();
	msg.body_rate.z = u.bodyrates.z();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	msg.orientation.x = u.q.x();
	msg.orientation.y = u.q.y();
	msg.orientation.z = u.q.z();
	msg.orientation.w = u.q.w();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub.publish(msg);
}

bool PX4CtrlFSM::request_fcu_mode(const std::string &mode)
{
	if (mode.empty())
	{
		ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Refusing to request an empty PX4 mode.");
		return false;
	}

	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = mode;
	if (!(set_FCU_mode_srv.call(set_mode) && set_mode.response.mode_sent))
	{
		ROS_ERROR_THROTTLE(1.0, "[px4ctrl] PX4 rejected mode request '%s'.", mode.c_str());
		return false;
	}

	// mode_sent only acknowledges the request.  /mavros/state is checked by the
	// FSM before it treats either OFFBOARD entry or exit as complete.
	return true;
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
	{
		if (arm)
			ROS_ERROR("ARM rejected by PX4!");
		else
			ROS_ERROR("DISARM rejected by PX4!");

		return false;
	}

	return true;
}

void PX4CtrlFSM::reboot_FCU()
{
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	mavros_msgs::CommandLong reboot_srv;
	reboot_srv.request.broadcast = false;
	reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	reboot_srv.request.param1 = 1;	  // Reboot autopilot
	reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
	reboot_srv.request.confirmation = true;

	reboot_FCU_srv.call(reboot_srv);

	ROS_INFO("Reboot FCU");

	// if (param.print_dbg)
	// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}
