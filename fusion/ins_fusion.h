// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FUSION_INS_FUSION_H_
#define FUSION_INS_FUSION_H_

#include <stdbool.h>
#include <stdint.h>

#include "fusion/kfcore_ins.h"

#ifdef __cplusplus
extern "C" {
#endif

// IMU + GPS INS on the KfcoreIns filter: IMU propagation and GPS position updates, plus velocity
// constraints for a road vehicle: zero velocity while the wheel speed reads zero (GPS is ignored
// then, since a parked receiver drifts) and, if `constrain_vertical_velocity`, near-zero vertical
// velocity while moving (see InsFusionOptions).
// A GPS altitude whose innovation fails a chi-square gate is dropped.
//
// Feed samples in timestamp order. The mean specific force of the first second of IMU data levels
// roll and pitch; the gyroscope bias starts at zero (the start need not be static).
// The filter starts at the second usable GPS fix after levelling: position from that fix, velocity
// from the step between the two, and yaw turning `forward`, integrated in the levelled yaw-free
// frame over the step, onto the step's course. After an IMU gap over `max_imu_gap_seconds` it
// starts again from the next two fixes, keeping the last attitude.
// C11, fixed-size storage only, no heap.

typedef struct {
  // level-road prior: near-zero world vertical velocity while moving
  bool constrain_vertical_velocity;
  // A longer IMU gap restarts the initialization, like INSLIB (its max_prediction_time_sec); a
  // shorter one is integrated with the first sample after it.
  double max_imu_gap_seconds;
  // direction of travel in the IMU frame (unit vector)
  double forward[3];
} InsFusionOptions;

typedef struct {
  int64_t timestamp_ns;
  double latitude_deg;  // WGS84
  double longitude_deg;
  double altitude_m;
  int satellite_count;
} InsFusionGps;

// State; read `initialized`, `utm_zone` and `filter` (biases), and get the pose with
// ins_fusion_pose(). The rest is internal.
typedef struct {
  InsFusionOptions options;
  KfcoreIns filter;
  // true once the filter started; ins_fusion_pose() is valid from then on
  bool initialized;
  // UTM zone of the first usable GPS fix, used for all fixes; 0 before it
  int utm_zone;
  // UTM position of the first usable fix: the filter runs in UTM axes relative to it, so its state
  // holds metres from the start rather than ~1e6 m northings.
  double origin[3];

  // the first of the two fixes the filter starts from: UTM relative to origin (m) and its
  // horizontal variance (m^2)
  bool has_first_fix;
  int64_t first_fix_timestamp_ns;
  double first_fix_position[3];
  double first_fix_variance;

  // levelling, and the pre-initialization attitude (body to levelled frame, w x y z) and forward
  // integral (levelled frame, s)
  int64_t first_imu_timestamp_ns;
  int64_t last_imu_timestamp_ns;
  bool has_last_imu;
  double last_acceleration[3];      // m/s^2, body
  double last_angular_velocity[3];  // rad/s, body
  int static_sample_count;
  double static_acceleration_sum[3];
  bool has_attitude;
  double attitude[4];
  double integrated_forward[3];
  // rad/s, body: zero until a restart carries the filter's estimate
  double gyroscope_bias[3];

  bool has_last_gps;
  double last_gps_latitude_deg;
  double last_gps_longitude_deg;
  int dropped_altitude_count;
  bool has_wheel_speed;
  int64_t wheel_speed_timestamp_ns;
  double wheel_speed_mps;
} InsFusion;

// No vertical constraint, 0.5 s maximum IMU gap, forward along IMU x.
InsFusionOptions ins_fusion_default_options(void);

void ins_fusion_init(InsFusion* fusion, const InsFusionOptions* options);

// `acceleration`: specific force (m/s^2), `angular_velocity` (rad/s), both IMU body frame.
void ins_fusion_add_imu(InsFusion* fusion, int64_t timestamp_ns, const double acceleration[3],
                        const double angular_velocity[3]);

// Returns true if the fix was applied to the initialized filter, and then writes `innovation`
// (fix minus predicted position, UTM, m) unless it is NULL.
bool ins_fusion_add_gps(InsFusion* fusion, const InsFusionGps* gps, double innovation[3]);

// Only used to detect standstill.
void ins_fusion_add_wheel_speed(InsFusion* fusion, int64_t timestamp_ns, double speed_mps);

// IMU pose: body to UTM (easting, northing, altitude; m) in `utm_zone`, rotation as a unit
// quaternion (w, x, y, z). Valid once `initialized`.
void ins_fusion_pose(const InsFusion* fusion, double position[3], double rotation[4]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // FUSION_INS_FUSION_H_
