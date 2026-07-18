#include <gtest/gtest.h>

#include <limits>

#include "localization_health.h"

namespace
{

ros::SteadyTime steady(double seconds)
{
  ros::SteadyTime time;
  time.fromSec(seconds);
  return time;
}

TEST(LocalizationHealth, DefaultsFailClosed)
{
  LocalizationHealth_Data_t health;
  EXPECT_FALSE(health.control_allowed(steady(10.0)));
  EXPECT_FALSE(health.is_fresh(steady(10.0)));
}

TEST(LocalizationHealth, RequiresContinuousHealthyHeartbeat)
{
  LocalizationHealth_Data_t health;
  health.configure(0.25, 0.50);

  health.feed(true, steady(1.00));
  EXPECT_FALSE(health.control_allowed(steady(1.10)));
  health.feed(true, steady(1.20));
  health.feed(true, steady(1.40));
  EXPECT_TRUE(health.control_allowed(steady(1.50)));
}

TEST(LocalizationHealth, FalseAndTimeoutRevokeImmediately)
{
  LocalizationHealth_Data_t health;
  health.configure(0.25, 0.0);

  health.feed(true, steady(2.00));
  EXPECT_TRUE(health.control_allowed(steady(2.00)));
  health.feed(false, steady(2.05));
  EXPECT_FALSE(health.control_allowed(steady(2.05)));

  health.feed(true, steady(3.00));
  EXPECT_TRUE(health.control_allowed(steady(3.249)));
  EXPECT_FALSE(health.control_allowed(steady(3.25)));
}

TEST(LocalizationHealth, BrokenHeartbeatMustQualifyAgain)
{
  LocalizationHealth_Data_t health;
  health.configure(0.25, 0.50);

  health.feed(true, steady(4.00));
  health.feed(true, steady(4.20));
  EXPECT_FALSE(health.control_allowed(steady(4.45)));

  health.feed(true, steady(4.46));
  health.feed(true, steady(4.65));
  EXPECT_FALSE(health.control_allowed(steady(4.75)));
  health.feed(true, steady(4.85));
  EXPECT_TRUE(health.control_allowed(steady(4.96)));
}

TEST(LocalizationHealth, FeedDetectsGapWithoutAnIntermediatePoll)
{
  LocalizationHealth_Data_t health;
  health.configure(0.25, 0.50);

  health.feed(true, steady(6.00));
  health.feed(true, steady(6.20));
  health.feed(true, steady(6.46));
  health.feed(true, steady(6.65));
  EXPECT_FALSE(health.control_allowed(steady(6.70)));
  health.feed(true, steady(6.85));
  EXPECT_TRUE(health.control_allowed(steady(6.96)));
}

TEST(LocalizationHealth, FalseToTrueStartsANewRecoveryWindow)
{
  LocalizationHealth_Data_t health;
  health.configure(0.25, 0.50);

  health.feed(true, steady(7.00));
  health.feed(true, steady(7.20));
  health.feed(false, steady(7.25));
  health.feed(true, steady(7.30));
  health.feed(true, steady(7.50));
  EXPECT_FALSE(health.control_allowed(steady(7.60)));
  health.feed(true, steady(7.70));
  EXPECT_TRUE(health.control_allowed(steady(7.80)));
}

TEST(LocalizationHealth, BackwardsReceiveTimeFailsClosed)
{
  LocalizationHealth_Data_t health;
  health.configure(0.25, 0.0);

  health.feed(true, steady(5.00));
  EXPECT_TRUE(health.control_allowed(steady(5.00)));
  health.feed(true, steady(4.99));
  EXPECT_FALSE(health.control_allowed(steady(5.00)));
  EXPECT_FALSE(health.reported_healthy());
}

TEST(LocalizationHealth, RejectsInvalidConfiguration)
{
  LocalizationHealth_Data_t health;
  EXPECT_THROW(health.configure(0.0, 0.5), std::invalid_argument);
  EXPECT_THROW(health.configure(0.2, -0.1), std::invalid_argument);
  EXPECT_THROW(health.configure(std::numeric_limits<double>::infinity(), 0.5),
               std::invalid_argument);
  EXPECT_THROW(health.configure(0.2, std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
