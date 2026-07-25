#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "input.h"

namespace
{

ros::Time stamp(double seconds)
{
  ros::Time value;
  value.fromSec(seconds);
  return value;
}

TEST(InputValidity, OdometryRequiresAdvancingSourceTime)
{
  Odom_Data_t data;
  EXPECT_FALSE(data.data_valid);
  EXPECT_TRUE(data.p.isZero());
  EXPECT_TRUE(data.v.isZero());

  nav_msgs::OdometryPtr msg(new nav_msgs::Odometry());
  msg->pose.pose.orientation.w = 1.0;
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);

  msg->header.stamp = stamp(1.0);
  data.feed(msg);
  EXPECT_TRUE(data.data_valid);
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);

  msg->header.stamp = stamp(1.01);
  data.feed(msg);
  EXPECT_TRUE(data.data_valid);
}

TEST(InputValidity, ImuIgnoresNonAdvancingSourceTimeWithoutRefreshingData)
{
  Imu_Data_t data;
  EXPECT_FALSE(data.data_valid);
  EXPECT_DOUBLE_EQ(data.q.w(), 1.0);
  EXPECT_EQ(data.validation_error,
            Imu_Data_t::ValidationError::NO_VALID_SAMPLE);

  sensor_msgs::ImuPtr msg(new sensor_msgs::Imu());
  msg->header.stamp = stamp(2.0);
  msg->orientation.w = 1.0;
  msg->angular_velocity.x = 0.1;
  msg->linear_acceleration.z = 9.81;
  data.feed(msg);
  EXPECT_TRUE(data.data_valid);
  EXPECT_EQ(data.validation_error, Imu_Data_t::ValidationError::NONE);

  const ros::SteadyTime accepted_receive_time = data.rcv_steady_stamp;
  const Eigen::Quaterniond accepted_q = data.q;
  const Eigen::Vector3d accepted_w = data.w;
  const Eigen::Vector3d accepted_a = data.a;

  msg->header.stamp = stamp(1.999);
  msg->angular_velocity.x = 4.0;
  msg->linear_acceleration.z = 1.0;
  data.feed(msg);
  EXPECT_TRUE(data.data_valid);
  EXPECT_TRUE(data.last_sample_stamp_nonadvancing);
  EXPECT_EQ(data.validation_error, Imu_Data_t::ValidationError::NONE);
  EXPECT_TRUE(data.last_source_stamp == stamp(2.0));
  EXPECT_TRUE(data.msg.header.stamp == stamp(2.0));
  EXPECT_TRUE(data.rcv_steady_stamp == accepted_receive_time);
  EXPECT_TRUE(data.q.isApprox(accepted_q));
  EXPECT_TRUE(data.w.isApprox(accepted_w));
  EXPECT_TRUE(data.a.isApprox(accepted_a));

  msg->header.stamp = stamp(2.0);
  data.feed(msg);
  EXPECT_TRUE(data.data_valid);
  EXPECT_TRUE(data.last_sample_stamp_nonadvancing);
  EXPECT_TRUE(data.rcv_steady_stamp == accepted_receive_time);

  msg->header.stamp = stamp(2.01);
  msg->angular_velocity.x = 0.2;
  msg->linear_acceleration.z = 9.7;
  data.feed(msg);
  EXPECT_TRUE(data.data_valid);
  EXPECT_FALSE(data.last_sample_stamp_nonadvancing);
  EXPECT_TRUE(data.last_source_stamp == stamp(2.01));
  EXPECT_DOUBLE_EQ(data.w.x(), 0.2);
  EXPECT_DOUBLE_EQ(data.a.z(), 9.7);
}

TEST(InputValidity, ImuPayloadAndZeroTimestampErrorsRemainHardFaults)
{
  Imu_Data_t data;
  sensor_msgs::ImuPtr msg(new sensor_msgs::Imu());
  msg->header.stamp = stamp(4.0);
  msg->orientation.w = 1.0;
  msg->linear_acceleration.z = 9.81;
  data.feed(msg);
  ASSERT_TRUE(data.data_valid);

  const ros::SteadyTime accepted_receive_time = data.rcv_steady_stamp;
  const Eigen::Quaterniond accepted_q = data.q;

  msg->header.stamp = stamp(4.01);
  msg->angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);
  EXPECT_EQ(data.validation_error,
            Imu_Data_t::ValidationError::NON_FINITE_ANGULAR_VELOCITY);
  EXPECT_STREQ(data.validation_error_reason(),
               "FCU IMU angular velocity contains non-finite data");
  EXPECT_TRUE(data.last_source_stamp == stamp(4.0));
  EXPECT_TRUE(data.rcv_steady_stamp == accepted_receive_time);
  EXPECT_TRUE(data.q.isApprox(accepted_q));

  msg->angular_velocity.x = 0.0;
  data.feed(msg);
  ASSERT_TRUE(data.data_valid);
  EXPECT_TRUE(data.last_source_stamp == stamp(4.01));

  msg->header.stamp = stamp(4.02);
  msg->linear_acceleration.x = std::numeric_limits<double>::infinity();
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);
  EXPECT_EQ(data.validation_error,
            Imu_Data_t::ValidationError::NON_FINITE_LINEAR_ACCELERATION);

  msg->linear_acceleration.x = 0.0;
  data.feed(msg);
  ASSERT_TRUE(data.data_valid);

  msg->header.stamp = stamp(4.03);
  msg->orientation.w = 0.0;
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);
  EXPECT_EQ(data.validation_error,
            Imu_Data_t::ValidationError::INVALID_QUATERNION);
  EXPECT_STREQ(data.validation_error_reason(),
               "FCU IMU orientation quaternion is invalid");

  msg->header.stamp = ros::Time(0);
  msg->orientation.w = 1.0;
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);
  EXPECT_EQ(data.validation_error,
            Imu_Data_t::ValidationError::ZERO_SOURCE_TIMESTAMP);
  EXPECT_STREQ(data.validation_error_reason(),
               "FCU IMU source timestamp is zero");
}

TEST(InputValidity, CommandYawNormalizationIsBoundedTime)
{
  Command_Data_t data;
  quadrotor_msgs::PositionCommandPtr msg(new quadrotor_msgs::PositionCommand());
  msg->header.stamp = stamp(3.0);
  msg->yaw = 1.0e300;
  data.feed(msg);
  EXPECT_TRUE(data.data_valid);
  EXPECT_TRUE(std::isfinite(data.yaw));
  EXPECT_LE(std::abs(data.yaw), M_PI);

  msg->header.stamp = stamp(3.01);
  msg->yaw = std::numeric_limits<double>::infinity();
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);
}

TEST(InputValidity, ShortRcMessagesAreRejectedWithoutIndexingPastTheArray)
{
  RC_Data_t data;
  mavros_msgs::RCInPtr msg(new mavros_msgs::RCIn());
  msg->channels.resize(7, 1500);
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);

  msg->channels.resize(8, 1500);
  data.feed(msg);
  EXPECT_TRUE(data.data_valid);
  EXPECT_TRUE(data.check_centered());

  msg->channels[0] = 2500;
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);
}

}  // namespace

int main(int argc, char **argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
