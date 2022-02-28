#include "gtest/gtest.h"

#include "gps_tools/utm_gpsfix_to_odometry.hpp"

TEST(estimate_track_error, basic_test)
{
  double all_zero = gps_tools::estimate_track_err({0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(0.0, all_zero);

  double no_error = gps_tools::estimate_track_err({1.0, 0.0, 1.0, 0.0}, {2.0, 0.0, 2.0, 0.0});
  EXPECT_EQ(0.0, no_error);

  double no_movement = gps_tools::estimate_track_err({0.0, 1.0, 0.0, 1.0}, {0.0, 2.0, 0.0, 0.3});
  EXPECT_EQ(0.0, no_movement);

  double movement = gps_tools::estimate_track_err({0.0, 1.0, 0.0, 1.0}, {20.0, 2.0, 30.0, 0.3});
  EXPECT_NE(0.0, movement);
}

TEST(estimate_track_error, relative_test)
{
  double movement = gps_tools::estimate_track_err({0.0, 0.2, 0.0, 0.2}, {100.0, 0.4, 20.0, 0.3});
  EXPECT_GT(0.017, movement);  // 0.017 ~= 1 deg

  double movement_v2 = gps_tools::estimate_track_err({0.0, 0.2, 0.0, 0.2}, {90.0, 0.4, 15.0, 0.3});
  double movement_v3 = gps_tools::estimate_track_err({0.0, 0.2, 0.0, 0.2}, {140.0, 0.4, 13.0, 0.3});

  EXPECT_LT(movement, movement_v3);
  EXPECT_LT(movement_v2, movement);
}
