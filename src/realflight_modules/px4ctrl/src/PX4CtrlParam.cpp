#include "PX4CtrlParam.h"

#include <cmath>
#include <stdexcept>

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
	read_essential_param(nh, "gain/Kp0", gain.Kp0);
	read_essential_param(nh, "gain/Kp1", gain.Kp1);
	read_essential_param(nh, "gain/Kp2", gain.Kp2);
	read_essential_param(nh, "gain/Kv0", gain.Kv0);
	read_essential_param(nh, "gain/Kv1", gain.Kv1);
	read_essential_param(nh, "gain/Kv2", gain.Kv2);
	read_essential_param(nh, "gain/Kvi0", gain.Kvi0);
	read_essential_param(nh, "gain/Kvi1", gain.Kvi1);
	read_essential_param(nh, "gain/Kvi2", gain.Kvi2);
	read_essential_param(nh, "gain/KAngR", gain.KAngR);
	read_essential_param(nh, "gain/KAngP", gain.KAngP);
	read_essential_param(nh, "gain/KAngY", gain.KAngY);

	read_essential_param(nh, "rotor_drag/x", rt_drag.x);
	read_essential_param(nh, "rotor_drag/y", rt_drag.y);
	read_essential_param(nh, "rotor_drag/z", rt_drag.z);
	read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
	read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);
	read_essential_param(nh, "msg_timeout/state", msg_timeout.state);
	read_essential_param(nh, "msg_timeout/extended_state", msg_timeout.extended_state);

	read_essential_param(nh, "localization_health/freshness_timeout", localization_health.freshness_timeout);
	read_essential_param(nh, "localization_health/recovery_duration", localization_health.recovery_duration);
	read_essential_param(nh, "localization_health/offboard_prestream_time", localization_health.offboard_prestream_time);
	read_essential_param(nh, "localization_health/offboard_entry_timeout", localization_health.offboard_entry_timeout);
	read_essential_param(nh, "localization_health/arm_confirmation_timeout", localization_health.arm_confirmation_timeout);
	read_essential_param(nh, "localization_health/mode_retry_interval", localization_health.mode_retry_interval);
	read_essential_param(nh, "localization_health/attitude_blend_time", localization_health.attitude_blend_time);
	read_essential_param(nh, "localization_health/exit_stream_grace", localization_health.exit_stream_grace);
	read_essential_param(nh, "localization_health/failsafe_mode_with_rc", localization_health.failsafe_mode_with_rc);
	read_essential_param(nh, "localization_health/failsafe_mode_without_rc", localization_health.failsafe_mode_without_rc);

	read_essential_param(nh, "pose_solver", pose_solver);
	read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
	read_essential_param(nh, "max_manual_vel", max_manual_vel);
	read_essential_param(nh, "max_angle", max_angle);
	read_essential_param(nh, "low_voltage", low_voltage);

	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

	read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
	read_essential_param(nh, "thrust_model/K1", thr_map.K1);
	read_essential_param(nh, "thrust_model/K2", thr_map.K2);
	read_essential_param(nh, "thrust_model/K3", thr_map.K3);
	read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
	read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
	

	max_angle /= (180.0 / M_PI);

	if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
	{
		takeoff_land.enable_auto_arm = false;
		ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
	}
	if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
	{
		takeoff_land.no_RC = false;
		ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	}

	if ( thr_map.print_val )
	{
		ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
	}

	const LocalizationHealth &health = localization_health;
	if (!std::isfinite(max_angle) ||
		(max_angle >= 0.0 && max_angle >= M_PI_2) ||
		!std::isfinite(msg_timeout.odom) || msg_timeout.odom <= 0.0 ||
		!std::isfinite(msg_timeout.rc) || msg_timeout.rc <= 0.0 ||
		!std::isfinite(msg_timeout.cmd) || msg_timeout.cmd <= 0.0 ||
		!std::isfinite(msg_timeout.imu) || msg_timeout.imu <= 0.0 ||
		!std::isfinite(msg_timeout.bat) || msg_timeout.bat <= 0.0 ||
		!std::isfinite(msg_timeout.state) || msg_timeout.state <= 0.0 ||
		!std::isfinite(msg_timeout.extended_state) || msg_timeout.extended_state <= 0.0 ||
		!std::isfinite(health.freshness_timeout) || health.freshness_timeout <= 0.0 ||
		!std::isfinite(health.recovery_duration) || health.recovery_duration < 0.0 ||
		!std::isfinite(health.offboard_prestream_time) || health.offboard_prestream_time < 1.0 ||
		!std::isfinite(health.offboard_entry_timeout) ||
		health.offboard_entry_timeout <= health.offboard_prestream_time ||
		!std::isfinite(health.arm_confirmation_timeout) || health.arm_confirmation_timeout <= 0.0 ||
		!std::isfinite(health.mode_retry_interval) || health.mode_retry_interval <= 0.0 ||
		!std::isfinite(health.attitude_blend_time) || health.attitude_blend_time < 0.0 ||
		!std::isfinite(health.exit_stream_grace) || health.exit_stream_grace <= 0.0 ||
		health.exit_stream_grace > 0.5 ||
		health.attitude_blend_time > health.exit_stream_grace ||
		health.failsafe_mode_with_rc.empty() || health.failsafe_mode_with_rc == "OFFBOARD" ||
		health.failsafe_mode_without_rc.empty() || health.failsafe_mode_without_rc == "OFFBOARD")
	{
		throw std::runtime_error("invalid max_angle, localization_health, or msg_timeout parameters");
	}
};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
