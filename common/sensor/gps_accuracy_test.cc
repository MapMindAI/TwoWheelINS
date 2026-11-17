// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/sensor/gps_accuracy.h"

#include <gtest/gtest.h>

#include <cmath>

namespace mapmind::sensor {
namespace {

TEST(GpsPositionStd, ShrinksWithSatelliteCount) {
  // single solutions
  EXPECT_DOUBLE_EQ(gps_position_std(8).horizontal, 1.5);
  EXPECT_DOUBLE_EQ(gps_position_std(8).vertical, 10.0);
  EXPECT_DOUBLE_EQ(gps_position_std(4).horizontal, 7.5);
  EXPECT_DOUBLE_EQ(gps_position_std(10).vertical, 50.0 / 7);
  EXPECT_TRUE(std::isfinite(gps_position_std(0).horizontal));
  EXPECT_DOUBLE_EQ(gps_position_std(0).horizontal, 7.5);
  // float solutions
  EXPECT_DOUBLE_EQ(gps_position_std(11).horizontal, 0.5);
  EXPECT_DOUBLE_EQ(gps_position_std(20).vertical, 1.0);
  // fix solutions
  EXPECT_DOUBLE_EQ(gps_position_std(21).horizontal, 0.05);
  EXPECT_DOUBLE_EQ(gps_position_std(30).vertical, 0.1);
}

}  // namespace
}  // namespace mapmind::sensor
