// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

// Runs InsFusion on a benchmark dataset (benchmark/datasets/<name>, see
// benchmark/make_datasets.py) and writes the IMU pose for every IMU sample after initialization in
// TUM format (t_s x y z qx qy qz qw), in the dataset's world frame like its gt.txt. Compare with
//   tools/trajectory_error.py <output.tum> <dataset>/gt.txt
//
// simulation::AddGpsNoise turns the noise-free GPS positions into WGS84 fixes (world x, y, z as UTM
// easting, northing and altitude offsets from kOrigin) with white noise, a drift that builds up in
// degraded stretches and decays in clear ones, and a satellite count that follows the drift.
// Trailing "# gps t_s x y z satellites" comments carry the noisy fixes in the world frame.
// The dataset's forward.txt (walking direction in the IMU frame) sets InsFusionOptions::forward,
// so the initial yaw comes from the course of the first two GPS fixes.
//
// Usage: run_dataset_main [--seed N] [--gps_noise_scale S] <dataset> <out.tum>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <gflags/gflags.h>

#include "common/math/geodesy.h"
#include "common/reader/benchmark_dataset_reader.h"
#include "common/simulation/gps_noise.h"
#include "fusion/ins_fusion.h"

DEFINE_uint32(seed, 1, "random seed of the GPS noise");
DEFINE_double(gps_noise_scale, 0.1, "scales the drift and white noise of the GPS");
DEFINE_double(max_imu_gap_seconds, 1.1,
              "longer IMU gaps restart the initialization (the lens bags drop 1 s every 2-4 s)");
DEFINE_bool(constrain_vertical_velocity, false,
            "level-floor prior: near-zero world vertical velocity while moving");

namespace mapmind::fusion {
namespace {

// any place works; the world frame is placed with its x, y, z axes along UTM east, north and up
const Eigen::Vector3d kOrigin(22.787839, 113.5210145, 13.0);

int Run(const std::string& dataset_path, const std::string& output_path) {
  const std::optional<reader::BenchmarkDataset> dataset =
      reader::ReadBenchmarkDataset(dataset_path);
  if (!dataset) {
    std::cerr << "cannot read imu.txt and gps.txt in " << dataset_path << "\n";
    return 1;
  }
  const int zone = math::UtmZone(kOrigin.y());
  const Eigen::Vector3d origin_utm = math::GeodeticToUtm(kOrigin, zone);
  simulation::GpsNoiseOptions noise;
  noise.horizontal_walk *= FLAGS_gps_noise_scale;
  noise.vertical_walk *= FLAGS_gps_noise_scale;
  noise.horizontal_white_std *= FLAGS_gps_noise_scale;
  noise.vertical_white_std *= FLAGS_gps_noise_scale;
  const std::vector<sensor::GpsData> gps =
      simulation::AddGpsNoise(dataset->gps, kOrigin, noise, FLAGS_seed);
  std::cout << "imu " << dataset->imu.size() << ", gps " << gps.size() << "\n";

  std::ofstream output(output_path);
  if (!output.is_open()) {
    std::cerr << "cannot write " << output_path << "\n";
    return 1;
  }
  output << std::fixed << "# " << dataset_path
         << " world frame, t_s x_m y_m z_m qx qy qz qw (world_from_imu)\n";

  // a fix is applied before the IMU sample it precedes
  InsFusionOptions options;
  options.constrain_vertical_velocity = FLAGS_constrain_vertical_velocity;
  options.max_imu_gap_seconds = FLAGS_max_imu_gap_seconds;
  options.forward = dataset->forward;
  InsFusion fusion(options);
  size_t gps_index = 0;
  int pose_count = 0;
  double initialized_s = 0.0;
  for (const sensor::ImuData& imu : dataset->imu) {
    for (; gps_index < gps.size() && gps[gps_index].timestamp_ns < imu.timestamp_ns; ++gps_index) {
      fusion.AddGps(gps[gps_index]);
    }
    fusion.AddImu(imu);
    if (!fusion.initialized()) {
      continue;
    }
    if (pose_count == 0) {
      initialized_s =
          1e-9 * static_cast<double>(imu.timestamp_ns - dataset->imu.front().timestamp_ns);
    }
    const Sophus::SE3d pose = fusion.pose();
    const Eigen::Vector3d position = pose.translation() - origin_utm;
    const Eigen::Quaterniond& rotation = pose.unit_quaternion();
    output << std::setprecision(6) << 1e-9 * static_cast<double>(imu.timestamp_ns)
           << std::setprecision(4) << " " << position.x() << " " << position.y() << " "
           << position.z() << std::setprecision(7) << " " << rotation.x() << " " << rotation.y()
           << " " << rotation.z() << " " << rotation.w() << "\n";
    ++pose_count;
  }
  // the GPS lines also when never initialized: other methods are run on them
  for (const sensor::GpsData& fix : gps) {
    const Eigen::Vector3d position =
        math::GeodeticToUtm({fix.latitude_deg, fix.longitude_deg, fix.altitude_m}, zone) -
        origin_utm;
    output << "# gps " << std::setprecision(6) << 1e-9 * static_cast<double>(fix.timestamp_ns)
           << std::setprecision(4) << " " << position.x() << " " << position.y() << " "
           << position.z() << " " << fix.satellite_count << "\n";
  }
  if (pose_count == 0) {
    std::cerr << "never initialized (fewer than two usable GPS fixes)\n";
    return 1;
  }
  std::cout << "wrote " << pose_count << " poses to " << output_path << ", initialized after "
            << std::setprecision(1) << initialized_s << " s\n"
            << "final bias: gyroscope " << fusion.gyroscope_bias().transpose() << ", accelerometer "
            << fusion.accelerometer_bias().transpose() << "\n";
  return 0;
}

}  // namespace
}  // namespace mapmind::fusion

int main(int argc, char** argv) {
  gflags::SetUsageMessage("[--seed N] [--gps_noise_scale S] <dataset> <out.tum>");
  gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);
  if (argc != 3) {
    gflags::ShowUsageWithFlagsRestrict(argv[0], "fusion/run_dataset_main");
    return 1;
  }
  return mapmind::fusion::Run(argv[1], argv[2]);
}
