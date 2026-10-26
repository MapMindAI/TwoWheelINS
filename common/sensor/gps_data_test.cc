// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/sensor/gps_data.h"

#include <gtest/gtest.h>

namespace mapmind::sensor {
namespace {

TEST(GpsCovariance, ShrinksWithSatelliteCount) {
  // single solutions
  EXPECT_DOUBLE_EQ(GpsCovariance(8)(0, 0), 1.5 * 1.5);
  EXPECT_DOUBLE_EQ(GpsCovariance(8)(1, 1), 1.5 * 1.5);
  EXPECT_DOUBLE_EQ(GpsCovariance(8)(2, 2), 10.0 * 10.0);
  EXPECT_DOUBLE_EQ(GpsCovariance(4)(0, 0), 7.5 * 7.5);
  EXPECT_DOUBLE_EQ(GpsCovariance(10)(2, 2), (50.0 / 7) * (50.0 / 7));
  EXPECT_TRUE(GpsCovariance(0).allFinite());
  // float solutions
  EXPECT_DOUBLE_EQ(GpsCovariance(11)(0, 0), 0.5 * 0.5);
  EXPECT_DOUBLE_EQ(GpsCovariance(20)(2, 2), 1.0 * 1.0);
  // fix solutions
  EXPECT_DOUBLE_EQ(GpsCovariance(21)(1, 1), 0.05 * 0.05);
  EXPECT_DOUBLE_EQ(GpsCovariance(30)(2, 2), 0.1 * 0.1);
}

}  // namespace
}  // namespace mapmind::sensor
