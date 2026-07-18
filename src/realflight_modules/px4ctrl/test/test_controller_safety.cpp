#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "controller.h"

namespace
{

TEST(ControllerSafety, LimitsCombinedTiltForLargeFiniteTrackingError)
{
  Parameter_t param;
  param.gra = 9.81;
  param.max_angle = 30.0 * M_PI / 180.0;
  param.gain.Kp0 = 2.0;
  param.gain.Kp1 = 2.0;
  param.gain.Kp2 = 2.0;
  param.gain.Kv0 = 1.5;
  param.gain.Kv1 = 1.5;
  param.gain.Kv2 = 1.5;
  param.thr_map.hover_percentage = 0.3;

  LinearControl controller(param);
  Odom_Data_t odom;
  Imu_Data_t imu;
  Desired_State_t desired;
  Controller_Output_t output;

  desired.p = Eigen::Vector3d(100.0, -100.0, 0.0);
  controller.calculateControl(desired, odom, imu, output);

  ASSERT_TRUE(output.q.coeffs().allFinite());
  output.q.normalize();
  const Eigen::Vector3d body_z = output.q * Eigen::Vector3d::UnitZ();
  const double tilt = std::acos(std::max(-1.0, std::min(1.0, body_z.z())));
  EXPECT_LE(tilt, param.max_angle + 1.0e-9);
  EXPECT_GT(tilt, 0.9 * param.max_angle);
}

}  // namespace

int main(int argc, char **argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
