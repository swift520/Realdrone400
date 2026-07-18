#include "localization_health.h"

#include <cmath>
#include <stdexcept>

LocalizationHealth_Data_t::LocalizationHealth_Data_t() = default;

void LocalizationHealth_Data_t::configure(double freshness_timeout,
                                           double recovery_duration)
{
  if (!std::isfinite(freshness_timeout) || freshness_timeout <= 0.0 ||
      !std::isfinite(recovery_duration) || recovery_duration < 0.0)
  {
    throw std::invalid_argument("invalid localization health timing");
  }

  freshness_timeout_ = freshness_timeout;
  recovery_duration_ = recovery_duration;
  configured_ = true;
  received_ = false;
  reported_healthy_ = false;
  qualifying_ = false;
  last_receive_ = ros::SteadyTime();
  qualifying_since_ = ros::SteadyTime();
}

void LocalizationHealth_Data_t::feed_msg(std_msgs::BoolConstPtr msg)
{
  feed(msg->data, ros::SteadyTime::now());
}

void LocalizationHealth_Data_t::feed(bool healthy,
                                     const ros::SteadyTime &receive_time)
{
  if (!configured_)
  {
    // A missing configure call must never silently enable control.
    reported_healthy_ = false;
    qualifying_ = false;
    return;
  }

  if (received_ && receive_time < last_receive_)
  {
    // Steady time cannot move backwards in production.  Treat an impossible
    // clock sequence as a broken heartbeat instead of extending its lifetime.
    received_ = true;
    reported_healthy_ = false;
    qualifying_ = false;
    last_receive_ = receive_time;
    return;
  }

  const bool continuity_broken =
      !received_ || !reported_healthy_ ||
      (receive_time - last_receive_) >= ros::WallDuration(freshness_timeout_);

  received_ = true;
  reported_healthy_ = healthy;
  last_receive_ = receive_time;

  if (!healthy)
  {
    qualifying_ = false;
    return;
  }

  if (continuity_broken || !qualifying_)
  {
    qualifying_ = true;
    qualifying_since_ = receive_time;
  }
}

bool LocalizationHealth_Data_t::is_fresh(const ros::SteadyTime &now) const
{
  if (!configured_ || !received_ || !reported_healthy_ || now < last_receive_)
  {
    return false;
  }

  return (now - last_receive_) < ros::WallDuration(freshness_timeout_);
}

bool LocalizationHealth_Data_t::control_allowed(const ros::SteadyTime &now)
{
  if (!is_fresh(now))
  {
    qualifying_ = false;
    return false;
  }

  if (!qualifying_ || now < qualifying_since_)
  {
    return false;
  }

  return (now - qualifying_since_) >= ros::WallDuration(recovery_duration_);
}
