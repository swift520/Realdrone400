#include "input.h"

#include <cmath>

namespace
{

bool quaternionIsValid(const Eigen::Quaterniond &q)
{
    return q.coeffs().allFinite() && std::isfinite(q.norm()) &&
           std::abs(q.norm() - 1.0) <= 0.10;
}

} // namespace

RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);
    rcv_steady_stamp = ros::SteadyTime();

    last_mode = -1.0;
    last_gear = -1.0;
    last_reboot_cmd = -1.0;
    mode = 0.0;
    gear = 0.0;
    reboot_cmd = 0.0;

    // Parameter initilation is very important in RC-Free usage!
    is_hover_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    toggle_reboot = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    rcv_steady_stamp = ros::SteadyTime::now();

    if (msg.channels.size() < 8)
    {
        data_valid = false;
        enter_hover_mode = false;
        enter_command_mode = false;
        toggle_reboot = false;
        ROS_ERROR_THROTTLE(1.0, "RC message has %zu channels; at least 8 are required.",
                           msg.channels.size());
        return;
    }

    data_valid = true;

    for (int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    mode = ((double)msg.channels[4] - 1000.0) / 1000.0;
    gear = ((double)msg.channels[5] - 1000.0) / 1000.0;
    reboot_cmd = ((double)msg.channels[7] - 1000.0) / 1000.0;

    check_validity();

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
            enter_command_mode = true;
        else if (gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false;

        if (gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else
            is_command_mode = false;
    }

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

    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
}

void RC_Data_t::check_validity()
{
    const bool sticks_valid =
        std::abs(ch[0]) <= 1.1 && std::abs(ch[1]) <= 1.1 &&
        std::abs(ch[2]) <= 1.1 && std::abs(ch[3]) <= 1.1;
    if (sticks_valid && mode >= -1.1 && mode <= 1.1 &&
        gear >= -1.1 && gear <= 1.1 &&
        reboot_cmd >= -1.1 && reboot_cmd <= 1.1)
    {
        // pass
    }
    else
    {
        data_valid = false;
        ROS_ERROR("RC data validity check fail. sticks=[%f, %f, %f, %f], "
                  "mode=%f, gear=%f, reboot_cmd=%f",
                  ch[0], ch[1], ch[2], ch[3], mode, gear, reboot_cmd);
    }
}

bool RC_Data_t::check_centered()
{
    bool centered = std::abs(ch[0]) < 1e-5 && std::abs(ch[1]) < 1e-5 &&
                    std::abs(ch[2]) < 1e-5 && std::abs(ch[3]) < 1e-5;
    return centered;
}

Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    rcv_steady_stamp = ros::SteadyTime();
    p.setZero();
    v.setZero();
    q.setIdentity();
    w.setZero();
    recv_new_msg = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;
    rcv_steady_stamp = ros::SteadyTime::now();
    recv_new_msg = true;

    uav_utils::extract_odometry(pMsg, p, v, q, w);
    const bool source_stamp_valid = !msg.header.stamp.isZero() &&
        (!source_stamp_seen || msg.header.stamp > last_source_stamp);
    if (source_stamp_valid)
    {
        source_stamp_seen = true;
        last_source_stamp = msg.header.stamp;
    }
    data_valid = source_stamp_valid && p.allFinite() && v.allFinite() &&
                 w.allFinite() && quaternionIsValid(q);
    if (data_valid)
    {
        q.normalize();
    }
    else
    {
        ROS_ERROR_THROTTLE(1.0, "Invalid odometry sample received; control will be inhibited.");
    }

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
    rcv_steady_stamp = ros::SteadyTime();
    q.setIdentity();
    w.setZero();
    a.setZero();
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;
    rcv_steady_stamp = ros::SteadyTime::now();

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

    const bool source_stamp_valid = !msg.header.stamp.isZero() &&
        (!source_stamp_seen || msg.header.stamp > last_source_stamp);
    if (source_stamp_valid)
    {
        source_stamp_seen = true;
        last_source_stamp = msg.header.stamp;
    }
    data_valid = source_stamp_valid && w.allFinite() && a.allFinite() &&
                 quaternionIsValid(q);
    if (data_valid)
    {
        q.normalize();
    }
    else
    {
        ROS_ERROR_THROTTLE(1.0, "Invalid FCU IMU sample received; control will be inhibited.");
    }

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
    rcv_steady_stamp = ros::SteadyTime();
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{

    current_state = *pMsg;
    rcv_steady_stamp = ros::SteadyTime::now();
}

ExtendedState_Data_t::ExtendedState_Data_t()
{
    rcv_steady_stamp = ros::SteadyTime();
}

void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg)
{
    current_extended_state = *pMsg;
    rcv_steady_stamp = ros::SteadyTime::now();
}

Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
    rcv_steady_stamp = ros::SteadyTime();
    p.setZero();
    v.setZero();
    a.setZero();
    j.setZero();
    yaw = 0.0;
    yaw_rate = 0.0;
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    rcv_steady_stamp = ros::SteadyTime::now();

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

    const bool source_stamp_valid = !msg.header.stamp.isZero() &&
        (!source_stamp_seen || msg.header.stamp > last_source_stamp);
    if (source_stamp_valid)
    {
        source_stamp_seen = true;
        last_source_stamp = msg.header.stamp;
    }
    const bool scalar_valid = std::isfinite(msg.yaw) && std::isfinite(msg.yaw_dot);
    yaw = scalar_valid ? std::remainder(msg.yaw, 2.0 * M_PI) : 0.0;
    yaw_rate = scalar_valid ? msg.yaw_dot : 0.0;
    data_valid = source_stamp_valid && p.allFinite() && v.allFinite() &&
                 a.allFinite() && j.allFinite() && scalar_valid;
    if (!data_valid)
    {
        ROS_ERROR_THROTTLE(1.0, "Invalid position command received; command will be ignored.");
    }
}

Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);
    rcv_steady_stamp = ros::SteadyTime();
}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    rcv_steady_stamp = ros::SteadyTime::now();

    double voltage = 0;
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
    {
        voltage += pMsg->cell_voltage[i];
    }
    volt = 0.8 * volt + 0.2 * voltage; // Naive LPF, cell_voltage has a higher frequency

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
            // ROS_ERROR("[px4ctrl] Dangerous! voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
}

Takeoff_Land_Data_t::Takeoff_Land_Data_t()
{
    rcv_stamp = ros::Time(0);
    rcv_steady_stamp = ros::SteadyTime();
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    rcv_steady_stamp = ros::SteadyTime::now();

    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
}
