// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_SENSOR_BAROMETER_DATA_H_
#define COMMON_SENSOR_BAROMETER_DATA_H_

#include <cstdint>

namespace mapmind::sensor {

struct BarometerData {
  int64_t timestamp_ns;
  double temperature_c;
  double pressure_pa;
};

}  // namespace mapmind::sensor

#endif  // COMMON_SENSOR_BAROMETER_DATA_H_
