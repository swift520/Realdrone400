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

TEST(InputValidity, ImuRequiresAdvancingSourceTimeAndValidQuaternion)
{
  Imu_Data_t data;
  EXPECT_FALSE(data.data_valid);
  EXPECT_DOUBLE_EQ(data.q.w(), 1.0);

  sensor_msgs::ImuPtr msg(new sensor_msgs::Imu());
  msg->header.stamp = stamp(2.0);
  msg->orientation.w = 1.0;
  data.feed(msg);
  EXPECT_TRUE(data.data_valid);
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);

  msg->header.stamp = stamp(2.01);
  msg->orientation.w = 0.0;
  data.feed(msg);
  EXPECT_FALSE(data.data_valid);
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
