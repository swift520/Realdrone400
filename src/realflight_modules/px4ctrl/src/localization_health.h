#ifndef PX4CTRL_LOCALIZATION_HEALTH_H
#define PX4CTRL_LOCALIZATION_HEALTH_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>

// A fail-closed heartbeat gate for the localization watchdog.  Steady time is
// deliberate: safety timeouts must keep advancing if ROS time is paused or
// moved backwards.
class LocalizationHealth_Data_t
{
public:
  LocalizationHealth_Data_t();

  void configure(double freshness_timeout, double recovery_duration);
  void feed_msg(std_msgs::BoolConstPtr msg);

  // Explicit receive time keeps the state machine deterministic in unit tests.
  void feed(bool healthy, const ros::SteadyTime &receive_time);

  bool is_fresh(const ros::SteadyTime &now) const;
  bool control_allowed(const ros::SteadyTime &now);
  bool reported_healthy() const { return received_ && reported_healthy_; }

private:
  double freshness_timeout_{0.0};
  double recovery_duration_{0.0};
  bool configured_{false};
  bool received_{false};
  bool reported_healthy_{false};
  bool qualifying_{false};
  ros::SteadyTime last_receive_;
  ros::SteadyTime qualifying_since_;
};

#endif
