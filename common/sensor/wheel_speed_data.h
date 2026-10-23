// Copyright 2026 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#ifndef COMMON_SENSOR_WHEEL_SPEED_DATA_H_
#define COMMON_SENSOR_WHEEL_SPEED_DATA_H_

#include <cstdint>

namespace mapmind::sensor {

struct WheelSpeedData {
  int64_t timestamp_ns;
  double speed_mps;
};

}  // namespace mapmind::sensor

#endif  // COMMON_SENSOR_WHEEL_SPEED_DATA_H_
