#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

namespace
{

bool isFinite(double value)
{
  return std::isfinite(value);
}

double positionDistance(const geometry_msgs::Pose &lhs, const geometry_msgs::Pose &rhs)
{
  const double dx = lhs.position.x - rhs.position.x;
  const double dy = lhs.position.y - rhs.position.y;
  const double dz = lhs.position.z - rhs.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double orientationDistance(const geometry_msgs::Pose &lhs, const geometry_msgs::Pose &rhs)
{
  const auto &a = lhs.orientation;
  const auto &b = rhs.orientation;
  const double a_norm = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
  const double b_norm = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w);
  double dot = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w) /
               (a_norm * b_norm);
  dot = std::max(0.0, std::min(1.0, dot));
  return 2.0 * std::acos(dot);
}

}  // namespace

class VisionPoseBridge
{
public:
  VisionPoseBridge(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh))
  {
    private_nh_.param("publish_rate", publish_rate_, 50.0);
    private_nh_.param("watchdog_rate", watchdog_rate_, 100.0);
    private_nh_.param("health_publish_rate", health_publish_rate_, 20.0);
    private_nh_.param("high_rate_timeout", high_rate_timeout_, 0.15);
    private_nh_.param("correction_timeout", correction_timeout_, 0.50);
    private_nh_.param("recovery_duration", recovery_duration_, 1.0);
    private_nh_.param("min_recovery_high_rate_samples", min_recovery_high_rate_samples_, 20);
    private_nh_.param("min_recovery_corrections", min_recovery_corrections_, 5);
    private_nh_.param("max_input_age", max_input_age_, 0.50);
    private_nh_.param("max_future_stamp", max_future_stamp_, 0.10);
    private_nh_.param("max_timestamp_skew", max_timestamp_skew_, 0.25);
    private_nh_.param("quaternion_norm_tolerance", quaternion_norm_tolerance_, 0.10);
    private_nh_.param("max_active_position_jump", max_active_position_jump_, 1.0);
    private_nh_.param("max_active_orientation_jump", max_active_orientation_jump_, 0.80);
    private_nh_.param("max_recovery_position_jump", max_recovery_position_jump_, 0.50);
    private_nh_.param("max_recovery_orientation_jump", max_recovery_orientation_jump_, 0.35);
    private_nh_.param("max_stream_position_difference", max_stream_position_difference_, 1.0);
    private_nh_.param("max_stream_orientation_difference", max_stream_orientation_difference_, 0.80);
    private_nh_.param("body_to_sensor_x", body_to_sensor_x_, 0.0);
    private_nh_.param("body_to_sensor_y", body_to_sensor_y_, 0.0);
    private_nh_.param("body_to_sensor_z", body_to_sensor_z_, 0.0);
    private_nh_.param("max_body_to_sensor_distance", max_body_to_sensor_distance_, 1.0);
    private_nh_.param<std::string>("output_frame_id", output_frame_id_, "map");
    private_nh_.param<std::string>("expected_high_rate_frame", expected_high_rate_frame_, "world");
    private_nh_.param<std::string>("expected_high_rate_child_frame", expected_high_rate_child_frame_,
                                   "odom_imu");
    private_nh_.param<std::string>("expected_correction_frame", expected_correction_frame_,
                                   "camera_init");
    private_nh_.param<std::string>("expected_correction_child_frame",
                                   expected_correction_child_frame_, "body");

    validateParameters();

    const ros::TransportHints transport_hints = ros::TransportHints().tcpNoDelay();
    high_rate_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "odom_high_rate", 1, &VisionPoseBridge::highRateCallback, this, transport_hints);
    correction_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "odom_correction", 1, &VisionPoseBridge::correctionCallback, this, transport_hints);

    vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);
    validated_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("localization/validated_odom", 1);
    health_pub_ = nh_.advertise<std_msgs::Bool>("localization/healthy", 1, true);
    reset_service_ = private_nh_.advertiseService("reset_fault", &VisionPoseBridge::resetFault, this);

    watchdog_timer_ = nh_.createSteadyTimer(
        ros::WallDuration(1.0 / watchdog_rate_), &VisionPoseBridge::watchdogCallback, this);
    publish_timer_ = nh_.createSteadyTimer(
        ros::WallDuration(1.0 / publish_rate_), &VisionPoseBridge::publishCallback, this);
    health_timer_ = nh_.createSteadyTimer(
        ros::WallDuration(1.0 / health_publish_rate_),
        &VisionPoseBridge::healthHeartbeatCallback, this);

    publishHealth(false, true);
    ROS_INFO("Applying body-to-FAST-LIO-sensor translation [%.3f, %.3f, %.3f] m; "
             "sensor and airframe axes must be aligned.",
             body_to_sensor_x_, body_to_sensor_y_, body_to_sensor_z_);
    ROS_INFO("Vision bridge waiting for high-rate odometry and LiDAR-corrected odometry.");
  }

private:
  enum class State
  {
    WAITING,
    RECOVERING,
    ACTIVE,
    FAULT_LATCHED
  };

  void validateParameters() const
  {
    const double numeric_parameters[] = {
        publish_rate_, watchdog_rate_, health_publish_rate_, high_rate_timeout_,
        correction_timeout_, recovery_duration_,
        max_input_age_, max_future_stamp_, max_timestamp_skew_, quaternion_norm_tolerance_,
        max_active_position_jump_, max_active_orientation_jump_, max_recovery_position_jump_,
        max_recovery_orientation_jump_, max_stream_position_difference_,
        max_stream_orientation_difference_, body_to_sensor_x_, body_to_sensor_y_,
        body_to_sensor_z_, max_body_to_sensor_distance_};
    for (const double value : numeric_parameters)
    {
      if (!isFinite(value))
      {
        throw std::runtime_error("vision bridge parameters must be finite numbers");
      }
    }

    if (publish_rate_ <= 0.0 || watchdog_rate_ <= 0.0 || health_publish_rate_ <= 0.0 ||
        high_rate_timeout_ <= 0.0 ||
        correction_timeout_ <= 0.0 ||
        recovery_duration_ < 0.0 || min_recovery_high_rate_samples_ < 1 ||
        min_recovery_corrections_ < 1 || quaternion_norm_tolerance_ <= 0.0 ||
        max_body_to_sensor_distance_ <= 0.0)
    {
      throw std::runtime_error("invalid vision bridge parameter");
    }

    const double body_to_sensor_distance =
        std::sqrt(body_to_sensor_x_ * body_to_sensor_x_ +
                  body_to_sensor_y_ * body_to_sensor_y_ +
                  body_to_sensor_z_ * body_to_sensor_z_);
    if (body_to_sensor_distance > max_body_to_sensor_distance_)
    {
      throw std::runtime_error("body-to-sensor translation exceeds the configured safety limit");
    }

    const double watchdog_period = 1.0 / watchdog_rate_;
    if (watchdog_period > 0.5 * std::min(high_rate_timeout_, correction_timeout_))
    {
      throw std::runtime_error("watchdog period must be at most half of the shortest input timeout");
    }
  }

  bool validateOdometry(const nav_msgs::Odometry &msg, const char *stream_name,
                        const std::string &expected_frame,
                        const std::string &expected_child_frame) const
  {
    if (msg.header.stamp.isZero())
    {
      ROS_WARN_THROTTLE(1.0, "%s odometry has a zero timestamp; sample rejected.", stream_name);
      return false;
    }

    if ((!expected_frame.empty() && msg.header.frame_id != expected_frame) ||
        (!expected_child_frame.empty() && msg.child_frame_id != expected_child_frame))
    {
      ROS_ERROR_THROTTLE(1.0,
                         "%s odometry frame changed or is misconfigured (got '%s' -> '%s', "
                         "expected '%s' -> '%s'); sample rejected.",
                         stream_name, msg.header.frame_id.c_str(), msg.child_frame_id.c_str(),
                         expected_frame.c_str(), expected_child_frame.c_str());
      return false;
    }

    const auto &position = msg.pose.pose.position;
    const auto &orientation = msg.pose.pose.orientation;
    const auto &linear_velocity = msg.twist.twist.linear;
    const auto &angular_velocity = msg.twist.twist.angular;
    if (!isFinite(position.x) || !isFinite(position.y) || !isFinite(position.z) ||
        !isFinite(orientation.x) || !isFinite(orientation.y) || !isFinite(orientation.z) ||
        !isFinite(orientation.w) ||
        !isFinite(linear_velocity.x) || !isFinite(linear_velocity.y) ||
        !isFinite(linear_velocity.z) || !isFinite(angular_velocity.x) ||
        !isFinite(angular_velocity.y) || !isFinite(angular_velocity.z))
    {
      ROS_ERROR_THROTTLE(1.0, "%s odometry contains NaN or Inf; sample rejected.", stream_name);
      return false;
    }

    const double norm = std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                                  orientation.z * orientation.z + orientation.w * orientation.w);
    if (norm < 1.0e-6 || std::abs(norm - 1.0) > quaternion_norm_tolerance_)
    {
      ROS_ERROR_THROTTLE(1.0, "%s odometry has an invalid quaternion norm %.6f; sample rejected.",
                         stream_name, norm);
      return false;
    }

    const ros::Time now = ros::Time::now();
    if (!now.isZero())
    {
      const double age = (now - msg.header.stamp).toSec();
      if (max_input_age_ > 0.0 && age > max_input_age_)
      {
        ROS_WARN_THROTTLE(1.0, "%s odometry is %.3f s old; sample rejected.", stream_name, age);
        return false;
      }
      if (max_future_stamp_ >= 0.0 && age < -max_future_stamp_)
      {
        ROS_WARN_THROTTLE(1.0, "%s odometry timestamp is %.3f s in the future; sample rejected.",
                          stream_name, -age);
        return false;
      }
    }

    return true;
  }

  void normalizePose(geometry_msgs::Pose &pose) const
  {
    auto &q = pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  }

  void transformSensorPoseToBodyPose(geometry_msgs::Pose &pose) const
  {
    // FAST-LIO publishes the pose of the MID360 internal IMU.  The configured
    // vector points from the flight-controller/body origin to that sensor
    // origin and is expressed in the aligned body/sensor axes.  With
    // T_world_sensor known and no relative rotation, the desired body pose is:
    //   p_world_body = p_world_sensor - R_world_sensor * p_body_sensor.
    const auto &q = pose.orientation;
    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;

    const double world_offset_x =
        (1.0 - 2.0 * (yy + zz)) * body_to_sensor_x_ +
        2.0 * (xy - wz) * body_to_sensor_y_ +
        2.0 * (xz + wy) * body_to_sensor_z_;
    const double world_offset_y =
        2.0 * (xy + wz) * body_to_sensor_x_ +
        (1.0 - 2.0 * (xx + zz)) * body_to_sensor_y_ +
        2.0 * (yz - wx) * body_to_sensor_z_;
    const double world_offset_z =
        2.0 * (xz - wy) * body_to_sensor_x_ +
        2.0 * (yz + wx) * body_to_sensor_y_ +
        (1.0 - 2.0 * (xx + yy)) * body_to_sensor_z_;

    pose.position.x -= world_offset_x;
    pose.position.y -= world_offset_y;
    pose.position.z -= world_offset_z;
  }

  void transformSensorVelocityToBodyVelocity(nav_msgs::Odometry &odom) const
  {
    // This FAST-LIO fork stores linear velocity in world coordinates and
    // angular velocity in aligned sensor/body coordinates.  For r_BS from the
    // body origin to the sensor origin:
    //   v_WB = v_WS - R_WS * (omega_S x r_BS).
    const auto &q = odom.pose.pose.orientation;
    const auto &omega = odom.twist.twist.angular;
    const double cross_x = omega.y * body_to_sensor_z_ - omega.z * body_to_sensor_y_;
    const double cross_y = omega.z * body_to_sensor_x_ - omega.x * body_to_sensor_z_;
    const double cross_z = omega.x * body_to_sensor_y_ - omega.y * body_to_sensor_x_;

    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;

    const double world_cross_x =
        (1.0 - 2.0 * (yy + zz)) * cross_x +
        2.0 * (xy - wz) * cross_y + 2.0 * (xz + wy) * cross_z;
    const double world_cross_y =
        2.0 * (xy + wz) * cross_x +
        (1.0 - 2.0 * (xx + zz)) * cross_y + 2.0 * (yz - wx) * cross_z;
    const double world_cross_z =
        2.0 * (xz - wy) * cross_x + 2.0 * (yz + wx) * cross_y +
        (1.0 - 2.0 * (xx + yy)) * cross_z;

    odom.twist.twist.linear.x -= world_cross_x;
    odom.twist.twist.linear.y -= world_cross_y;
    odom.twist.twist.linear.z -= world_cross_z;
  }

  void handleGapBeforeUpdate(const ros::SteadyTime &now, const ros::SteadyTime &last_receive,
                             bool stream_seen, double timeout, const char *stream_name)
  {
    if (!stream_seen || (now - last_receive).toSec() <= timeout)
    {
      return;
    }

    std::ostringstream reason;
    reason << stream_name << " input gap exceeded " << timeout << " s";
    if (state_ == State::ACTIVE || (state_ == State::RECOVERING && recovering_after_fault_))
    {
      latchFault(reason.str());
    }
    else if (state_ == State::RECOVERING)
    {
      enterWaiting(reason.str());
    }
  }

  void handleRejectedSample(const std::string &reason)
  {
    if (state_ == State::ACTIVE || (state_ == State::RECOVERING && recovering_after_fault_))
    {
      latchFault(reason);
    }
    else if (state_ == State::RECOVERING)
    {
      enterWaiting(reason);
    }
  }

  void handleTimestampDiscontinuity(const std::string &reason)
  {
    if (last_published_pose_valid_)
    {
      restart_required_ = true;
      latchFault(reason + "; bridge/localization restart required");
      ROS_ERROR("%s. Online fault reset is disabled because PX4 must never receive a timestamp "
                "epoch discontinuity.", reason.c_str());
    }
    else if (state_ == State::RECOVERING)
    {
      enterWaiting(reason);
    }
    else if (state_ == State::ACTIVE)
    {
      latchFault(reason);
    }
    else
    {
      ROS_WARN("%s; starting a new startup candidate sequence.", reason.c_str());
    }
  }

  bool acceptStamp(const ros::Time &stamp, const ros::SteadyTime &now, bool stream_seen,
                   bool &stamp_seen, ros::Time &last_stamp,
                   const ros::SteadyTime &last_receive, const char *stream_name)
  {
    if (!stamp_seen)
    {
      stamp_seen = true;
      last_stamp = stamp;
      return true;
    }

    if (stamp == last_stamp)
    {
      ROS_WARN_THROTTLE(1.0, "%s odometry timestamp repeated; watchdog was not refreshed.", stream_name);
      return false;
    }

    const double source_delta = (stamp - last_stamp).toSec();
    if (source_delta < 0.0)
    {
      std::ostringstream reason;
      reason << stream_name << " timestamp moved backwards by " << -source_delta << " s";
      handleTimestampDiscontinuity(reason.str());
      last_stamp = stamp;
      return true;
    }

    if (stream_seen && max_timestamp_skew_ > 0.0)
    {
      const double receive_delta = (now - last_receive).toSec();
      if (source_delta - receive_delta > max_timestamp_skew_)
      {
        std::ostringstream reason;
        reason << stream_name << " timestamp advanced " << source_delta
               << " s during only " << receive_delta << " s of steady time";
        handleTimestampDiscontinuity(reason.str());
        last_stamp = stamp;
        return true;
      }
    }

    last_stamp = stamp;
    return true;
  }

  bool activePoseJumped(const geometry_msgs::Pose &new_pose) const
  {
    if (!high_rate_seen_)
    {
      return false;
    }
    return (max_active_position_jump_ > 0.0 &&
            positionDistance(latest_high_rate_.pose.pose, new_pose) > max_active_position_jump_) ||
           (max_active_orientation_jump_ > 0.0 &&
            orientationDistance(latest_high_rate_.pose.pose, new_pose) > max_active_orientation_jump_);
  }

  bool recoveryPoseSampleIsContinuous(const geometry_msgs::Pose &new_pose)
  {
    if (state_ != State::RECOVERING)
    {
      return true;
    }

    bool discontinuous = false;
    std::ostringstream reason;
    if (recovering_after_fault_ && last_published_pose_valid_)
    {
      const double position_jump = positionDistance(last_published_pose_, new_pose);
      const double orientation_jump = orientationDistance(last_published_pose_, new_pose);
      if ((max_recovery_position_jump_ > 0.0 && position_jump > max_recovery_position_jump_) ||
          (max_recovery_orientation_jump_ > 0.0 &&
           orientation_jump > max_recovery_orientation_jump_))
      {
        discontinuous = true;
        reason << "recovery pose differs from the last trusted output (position " << position_jump
               << " m, orientation " << orientation_jump << " rad)";
      }
    }

    if (!discontinuous && recovery_previous_pose_valid_)
    {
      const double position_step = positionDistance(recovery_previous_pose_, new_pose);
      const double orientation_step = orientationDistance(recovery_previous_pose_, new_pose);
      if ((max_active_position_jump_ > 0.0 && position_step > max_active_position_jump_) ||
          (max_active_orientation_jump_ > 0.0 &&
           orientation_step > max_active_orientation_jump_))
      {
        discontinuous = true;
        reason << "pose jumped during recovery validation (position " << position_step
               << " m, orientation " << orientation_step << " rad)";
      }
    }

    if (discontinuous)
    {
      handleRejectedSample(reason.str());
      return false;
    }

    recovery_previous_pose_ = new_pose;
    recovery_previous_pose_valid_ = true;
    return true;
  }

  bool correctionAgreesWithHighRate(const geometry_msgs::Pose &correction_pose,
                                    const ros::SteadyTime &now) const
  {
    if (!high_rate_seen_ || (now - last_high_rate_receive_).toSec() > high_rate_timeout_)
    {
      return true;
    }

    return (max_stream_position_difference_ <= 0.0 ||
            positionDistance(latest_high_rate_.pose.pose, correction_pose) <=
                max_stream_position_difference_) &&
           (max_stream_orientation_difference_ <= 0.0 ||
            orientationDistance(latest_high_rate_.pose.pose, correction_pose) <=
                max_stream_orientation_difference_);
  }

  void highRateCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    const ros::SteadyTime now = ros::SteadyTime::now();
    handleGapBeforeUpdate(now, last_high_rate_receive_, high_rate_seen_, high_rate_timeout_, "high-rate");

    if (!validateOdometry(*msg, "high-rate", expected_high_rate_frame_,
                          expected_high_rate_child_frame_))
    {
      handleRejectedSample("high-rate odometry sample failed validation");
      return;
    }
    if (!acceptStamp(msg->header.stamp, now, high_rate_seen_, high_rate_stamp_seen_,
                     last_high_rate_stamp_, last_high_rate_receive_, "high-rate"))
    {
      return;
    }

    nav_msgs::Odometry candidate = *msg;
    normalizePose(candidate.pose.pose);
    transformSensorPoseToBodyPose(candidate.pose.pose);
    transformSensorVelocityToBodyVelocity(candidate);
    candidate.child_frame_id = "body";
    if (state_ == State::ACTIVE && activePoseJumped(candidate.pose.pose))
    {
      latchFault("high-rate pose jump exceeded the configured limit");
      return;
    }
    if (!recoveryPoseSampleIsContinuous(candidate.pose.pose))
    {
      return;
    }

    latest_high_rate_ = candidate;
    high_rate_seen_ = true;
    last_high_rate_receive_ = now;
    if (state_ == State::RECOVERING)
    {
      ++recovery_high_rate_samples_;
    }

    maybeStartRecovery(now);

    // px4ctrl consumes only this checked, body-origin odometry. Publishing it
    // from the callback that accepted the sample prevents a finite pose jump
    // from racing ahead of the separate health Bool.
    if (state_ == State::ACTIVE && streamsFresh(now))
    {
      validated_odom_pub_.publish(candidate);
    }
  }

  void correctionCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    const ros::SteadyTime now = ros::SteadyTime::now();
    handleGapBeforeUpdate(now, last_correction_receive_, correction_seen_, correction_timeout_,
                          "LiDAR-corrected");

    if (!validateOdometry(*msg, "LiDAR-corrected", expected_correction_frame_,
                          expected_correction_child_frame_))
    {
      handleRejectedSample("LiDAR-corrected odometry sample failed validation");
      return;
    }
    if (!acceptStamp(msg->header.stamp, now, correction_seen_, correction_stamp_seen_,
                     last_correction_stamp_, last_correction_receive_, "LiDAR-corrected"))
    {
      return;
    }

    nav_msgs::Odometry candidate = *msg;
    normalizePose(candidate.pose.pose);
    transformSensorPoseToBodyPose(candidate.pose.pose);
    if (!correctionAgreesWithHighRate(candidate.pose.pose, now))
    {
      ROS_ERROR_THROTTLE(1.0, "LiDAR correction disagrees with high-rate pose; sample rejected.");
      handleRejectedSample("LiDAR correction disagrees with high-rate pose");
      return;
    }

    correction_seen_ = true;
    latest_correction_source_stamp_ = candidate.header.stamp;
    last_correction_receive_ = now;
    if (state_ == State::RECOVERING)
    {
      ++recovery_corrections_;
    }

    maybeStartRecovery(now);
  }

  bool streamsFresh(const ros::SteadyTime &now) const
  {
    if (!high_rate_seen_ || !correction_seen_ ||
        (now - last_high_rate_receive_).toSec() > high_rate_timeout_ ||
        (now - last_correction_receive_).toSec() > correction_timeout_)
    {
      return false;
    }

    // Reception freshness alone is insufficient: a delayed sample that was
    // almost too old on arrival must not remain usable for another full
    // reception-timeout interval.
    const ros::Time ros_now = ros::Time::now();
    if (ros_now.isZero())
    {
      return true;
    }

    const double high_source_age = (ros_now - latest_high_rate_.header.stamp).toSec();
    const double correction_source_age = (ros_now - latest_correction_source_stamp_).toSec();
    const bool input_age_ok = max_input_age_ <= 0.0 ||
                              (high_source_age <= max_input_age_ &&
                               correction_source_age <= max_input_age_);
    const bool future_stamp_ok = max_future_stamp_ < 0.0 ||
                                 (high_source_age >= -max_future_stamp_ &&
                                  correction_source_age >= -max_future_stamp_);
    return input_age_ok && future_stamp_ok;
  }

  void maybeStartRecovery(const ros::SteadyTime &now)
  {
    if (state_ != State::WAITING || !streamsFresh(now))
    {
      return;
    }

    state_ = State::RECOVERING;
    recovering_after_fault_ = false;
    recovery_start_ = now;
    recovery_high_rate_samples_ = 0;
    recovery_corrections_ = 0;
    recovery_previous_pose_valid_ = false;
    ROS_INFO("Both odometry streams are present; validating them before enabling PX4 output.");
  }

  bool recoveryPoseIsContinuous() const
  {
    if (!recovering_after_fault_ || !last_published_pose_valid_)
    {
      return true;
    }

    const double position_jump = positionDistance(last_published_pose_, latest_high_rate_.pose.pose);
    const double orientation_jump = orientationDistance(last_published_pose_, latest_high_rate_.pose.pose);
    if ((max_recovery_position_jump_ > 0.0 && position_jump > max_recovery_position_jump_) ||
        (max_recovery_orientation_jump_ > 0.0 && orientation_jump > max_recovery_orientation_jump_))
    {
      ROS_ERROR("Recovery pose is discontinuous (position %.3f m, orientation %.3f rad).",
                position_jump, orientation_jump);
      return false;
    }
    return true;
  }

  void maybeFinishRecovery(const ros::SteadyTime &now)
  {
    if (state_ != State::RECOVERING || !streamsFresh(now))
    {
      return;
    }
    if ((now - recovery_start_).toSec() < recovery_duration_ ||
        recovery_high_rate_samples_ < min_recovery_high_rate_samples_ ||
        recovery_corrections_ < min_recovery_corrections_)
    {
      return;
    }
    if (!recoveryPoseIsContinuous())
    {
      latchFault("recovered pose is not continuous with the last trusted output");
      return;
    }

    state_ = State::ACTIVE;
    recovering_after_fault_ = false;
    publishHealth(true);
    ROS_INFO("Localization is healthy; fresh high-rate poses are now forwarded to PX4.");
  }

  void watchdogCallback(const ros::SteadyTimerEvent &)
  {
    const ros::SteadyTime now = ros::SteadyTime::now();

    if (state_ == State::ACTIVE && !streamsFresh(now))
    {
      const double high_age = high_rate_seen_ ? (now - last_high_rate_receive_).toSec() : -1.0;
      const double correction_age = correction_seen_ ? (now - last_correction_receive_).toSec() : -1.0;
      std::ostringstream reason;
      reason << "odometry watchdog expired (high-rate age=" << high_age
             << " s, correction age=" << correction_age << " s)";
      latchFault(reason.str());
      return;
    }

    if (state_ == State::RECOVERING && !streamsFresh(now))
    {
      if (recovering_after_fault_)
      {
        latchFault("odometry stream became stale during fault recovery");
      }
      else
      {
        enterWaiting("odometry stream became stale during startup validation");
      }
      return;
    }

    maybeFinishRecovery(now);
  }

  void publishCallback(const ros::SteadyTimerEvent &)
  {
    if (state_ != State::ACTIVE)
    {
      return;
    }

    const ros::SteadyTime now = ros::SteadyTime::now();
    if (!streamsFresh(now))
    {
      const double high_age = high_rate_seen_ ? (now - last_high_rate_receive_).toSec() : -1.0;
      const double correction_age =
          correction_seen_ ? (now - last_correction_receive_).toSec() : -1.0;
      std::ostringstream reason;
      reason << "odometry watchdog expired before publish (high-rate age=" << high_age
             << " s, correction age=" << correction_age << " s)";
      latchFault(reason.str());
      return;
    }

    if (state_ != State::ACTIVE || latest_high_rate_.header.stamp <= last_published_stamp_)
    {
      return;
    }

    geometry_msgs::PoseStamped output;
    output.header = latest_high_rate_.header;
    if (!output_frame_id_.empty())
    {
      output.header.frame_id = output_frame_id_;
    }
    output.pose = latest_high_rate_.pose.pose;

    vision_pose_pub_.publish(output);
    last_published_stamp_ = output.header.stamp;
    last_published_pose_ = output.pose;
    last_published_pose_valid_ = true;
  }

  void healthHeartbeatCallback(const ros::SteadyTimerEvent &)
  {
    // Repeating the latched value lets downstream consumers distinguish an
    // ACTIVE bridge from a dead process whose final latched message was true.
    publishHealth(state_ == State::ACTIVE, true);
  }

  void enterWaiting(const std::string &reason)
  {
    state_ = State::WAITING;
    recovering_after_fault_ = false;
    recovery_high_rate_samples_ = 0;
    recovery_corrections_ = 0;
    recovery_previous_pose_valid_ = false;
    publishHealth(false);
    ROS_WARN("Vision bridge returned to WAITING: %s", reason.c_str());
  }

  void latchFault(const std::string &reason)
  {
    if (state_ == State::FAULT_LATCHED)
    {
      return;
    }
    state_ = State::FAULT_LATCHED;
    recovering_after_fault_ = false;
    recovery_high_rate_samples_ = 0;
    recovery_corrections_ = 0;
    recovery_previous_pose_valid_ = false;
    publishHealth(false);
    if (restart_required_)
    {
      ROS_ERROR("Localization fault latched: %s. PX4 vision output stopped. Land/disarm and restart "
                "the localization bridge before reuse.", reason.c_str());
    }
    else
    {
      ROS_ERROR("Localization fault latched: %s. PX4 vision output stopped. Call ~reset_fault only "
                "after the cause is understood and both streams are healthy.", reason.c_str());
    }
  }

  bool resetFault(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response)
  {
    if (state_ != State::FAULT_LATCHED)
    {
      response.success = false;
      response.message = "no localization fault is latched";
      return true;
    }

    if (restart_required_)
    {
      response.success = false;
      response.message =
          "timestamp epoch changed; land/disarm and restart the localization bridge before reuse";
      return true;
    }

    const ros::SteadyTime now = ros::SteadyTime::now();
    if (!streamsFresh(now))
    {
      response.success = false;
      response.message = "both odometry streams must be fresh before recovery can start";
      return true;
    }

    state_ = State::RECOVERING;
    recovering_after_fault_ = true;
    recovery_start_ = now;
    recovery_high_rate_samples_ = 0;
    recovery_corrections_ = 0;
    recovery_previous_pose_valid_ = false;
    publishHealth(false);
    response.success = true;
    response.message = "recovery validation started; PX4 output remains disabled until checks pass";
    ROS_WARN("Localization fault reset requested; validating continuous fresh data before resuming output.");
    return true;
  }

  void publishHealth(bool healthy, bool force = false)
  {
    if (!force && health_initialized_ && healthy == last_health_)
    {
      return;
    }
    std_msgs::Bool message;
    message.data = healthy;
    health_pub_.publish(message);
    health_initialized_ = true;
    last_health_ = healthy;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber high_rate_sub_;
  ros::Subscriber correction_sub_;
  ros::Publisher vision_pose_pub_;
  ros::Publisher validated_odom_pub_;
  ros::Publisher health_pub_;
  ros::ServiceServer reset_service_;
  ros::SteadyTimer watchdog_timer_;
  ros::SteadyTimer publish_timer_;
  ros::SteadyTimer health_timer_;

  State state_{State::WAITING};
  bool recovering_after_fault_{false};
  bool high_rate_seen_{false};
  bool correction_seen_{false};
  bool high_rate_stamp_seen_{false};
  bool correction_stamp_seen_{false};
  bool health_initialized_{false};
  bool last_health_{false};
  bool last_published_pose_valid_{false};
  bool recovery_previous_pose_valid_{false};
  bool restart_required_{false};

  nav_msgs::Odometry latest_high_rate_;
  geometry_msgs::Pose last_published_pose_;
  geometry_msgs::Pose recovery_previous_pose_;
  ros::Time last_high_rate_stamp_;
  ros::Time last_correction_stamp_;
  ros::Time latest_correction_source_stamp_;
  ros::Time last_published_stamp_;
  ros::SteadyTime last_high_rate_receive_;
  ros::SteadyTime last_correction_receive_;
  ros::SteadyTime recovery_start_;

  int recovery_high_rate_samples_{0};
  int recovery_corrections_{0};
  int min_recovery_high_rate_samples_{20};
  int min_recovery_corrections_{5};

  double publish_rate_{50.0};
  double watchdog_rate_{100.0};
  double health_publish_rate_{20.0};
  double high_rate_timeout_{0.15};
  double correction_timeout_{0.50};
  double recovery_duration_{1.0};
  double max_input_age_{0.50};
  double max_future_stamp_{0.10};
  double max_timestamp_skew_{0.25};
  double quaternion_norm_tolerance_{0.10};
  double max_active_position_jump_{1.0};
  double max_active_orientation_jump_{0.80};
  double max_recovery_position_jump_{0.50};
  double max_recovery_orientation_jump_{0.35};
  double max_stream_position_difference_{1.0};
  double max_stream_orientation_difference_{0.80};
  double body_to_sensor_x_{0.0};
  double body_to_sensor_y_{0.0};
  double body_to_sensor_z_{0.0};
  double max_body_to_sensor_distance_{1.0};
  std::string output_frame_id_{"map"};
  std::string expected_high_rate_frame_{"world"};
  std::string expected_high_rate_child_frame_{"odom_imu"};
  std::string expected_correction_frame_{"camera_init"};
  std::string expected_correction_child_frame_{"body"};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_pose_node");
  try
  {
    VisionPoseBridge bridge(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
  }
  catch (const std::exception &exception)
  {
    ROS_FATAL("Failed to start vision pose bridge: %s", exception.what());
    return 1;
  }
  return 0;
}
