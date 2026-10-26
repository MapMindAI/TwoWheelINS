// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

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
