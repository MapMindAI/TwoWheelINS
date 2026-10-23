// Copyright 2026 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "common/math/geodesy.h"

#include <gtest/gtest.h>

namespace mapmind::math {
namespace {

TEST(Geodesy, UtmZone) {
  EXPECT_EQ(UtmZone(-180.0), 1);
  EXPECT_EQ(UtmZone(0.0), 31);
  EXPECT_EQ(UtmZone(113.5210145), 49);
  EXPECT_EQ(UtmZone(179.999), 60);
  EXPECT_EQ(UtmZone(180.0), 1);
}

// Reference values from PROJ (pyproj, EPSG:326xx / 327xx).
TEST(Geodesy, GeodeticToUtmMatchesProj) {
  struct Case {
    Eigen::Vector3d geodetic;
    int zone;
    Eigen::Vector3d utm;
  };
  const Case cases[] = {
      {{22.787839, 113.5210145, 13.0}, 49, {758818.2856, 2522240.2898, 13.0}},
      {{22.803932, 113.528933, 2.2}, 49, {759601.0986, 2524036.8492, 2.2}},
      {{0.0, 0.0, 0.0}, 31, {166021.4431, 0.0, 0.0}},
      {{0.0, 3.0, 0.0}, 31, {500000.0, 0.0, 0.0}},
      {{-33.8688, 151.2093, 0.0}, 56, {334368.6336, 6250948.3454, 0.0}},
      {{60.0, 10.0, 0.0}, 32, {555776.2668, 6651832.7354, 0.0}},
  };
  for (const Case& c : cases) {
    const Eigen::Vector3d utm = GeodeticToUtm(c.geodetic, c.zone);
    EXPECT_NEAR(utm.x(), c.utm.x(), 1e-3) << c.geodetic.transpose();
    EXPECT_NEAR(utm.y(), c.utm.y(), 1e-3) << c.geodetic.transpose();
    EXPECT_DOUBLE_EQ(utm.z(), c.utm.z());
  }
}

}  // namespace
}  // namespace mapmind::math
