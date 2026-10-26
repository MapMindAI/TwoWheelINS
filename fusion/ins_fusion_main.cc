// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

// Runs InsFusion on a MOKUKU text record and writes the IMU pose in UTM (easting, northing,
// altitude) for every IMU sample after initialization, in TUM format: t_s x y z qx qy qz qw.
// A leading "# UTM zone <number><N|S>" comment lets fusion/trajectory_viewer.html put it on a map;
// trailing "# gps t_s easting_m northing_m altitude_m satellites" comments carry the record's GPS
// fixes for the viewer to overlay.
//
// Usage: ins_fusion_main <record> <output.tum>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "common/math/geodesy.h"
#include "common/reader/txt_record_reader.h"
#include "fusion/ins_fusion.h"

namespace mapmind::fusion {
namespace {

double Percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  return values[static_cast<size_t>(fraction * static_cast<double>(values.size() - 1))];
}

int Run(const std::string& record_path, const std::string& output_path) {
  std::optional<reader::TxtRecord> record = reader::ReadTxtRecord(record_path);
  if (!record) {
    std::cerr << "cannot open " << record_path << "\n";
    return 1;
  }
  const int removed_imu = reader::RemoveCorruptImuSamples(&record->imu);
  std::cout << "imu " << record->imu.size() << " (removed " << removed_imu << " corrupt), gps "
            << record->gps.size() << ", wheel speed " << record->wheel_speed.size()
            << ", skipped lines " << record->skipped_lines << "\n";

  std::ofstream output(output_path);
  if (!output.is_open()) {
    std::cerr << "cannot write " << output_path << "\n";
    return 1;
  }
  output << std::fixed;

  // Merge the streams by timestamp; a measurement is applied before the IMU sample it precedes.
  InsFusion fusion;
  const std::vector<sensor::GpsData>& gps = record->gps;
  const std::vector<sensor::WheelSpeedData>& wheel_speed = record->wheel_speed;
  size_t gps_index = 0;
  size_t wheel_index = 0;
  std::vector<double> gps_horizontal_innovation;
  int pose_count = 0;
  for (const sensor::ImuData& imu : record->imu) {
    for (; gps_index < gps.size() && gps[gps_index].timestamp_ns < imu.timestamp_ns; ++gps_index) {
      if (const std::optional<Eigen::Vector3d> innovation = fusion.AddGps(gps[gps_index])) {
        gps_horizontal_innovation.push_back(innovation->head<2>().norm());
      }
    }
    for (; wheel_index < wheel_speed.size() &&
           wheel_speed[wheel_index].timestamp_ns < imu.timestamp_ns;
         ++wheel_index) {
      fusion.AddWheelSpeed(wheel_speed[wheel_index]);
    }
    fusion.AddImu(imu);
    if (!fusion.initialized()) {
      continue;
    }
    if (pose_count == 0) {
      output << "# UTM zone " << *fusion.utm_zone() << (gps.front().latitude_deg < 0 ? "S" : "N")
             << " (WGS84), t_s easting_m northing_m altitude_m qx qy qz qw\n";
    }
    const Sophus::SE3d pose = fusion.pose();
    const Eigen::Quaterniond& rotation = pose.unit_quaternion();
    output << std::setprecision(6) << 1e-9 * static_cast<double>(imu.timestamp_ns)
           << std::setprecision(4) << " " << pose.translation().x() << " " << pose.translation().y()
           << " " << pose.translation().z() << std::setprecision(7) << " " << rotation.x() << " "
           << rotation.y() << " " << rotation.z() << " " << rotation.w() << "\n";
    ++pose_count;
  }

  if (!fusion.initialized()) {
    std::cerr << "vehicle never moves fast enough to initialize heading\n";
    return 1;
  }
  for (const sensor::GpsData& fix : gps) {
    if (fix.latitude_deg == 0.0 && fix.longitude_deg == 0.0) {
      continue;
    }
    const Eigen::Vector3d utm = math::GeodeticToUtm(
        Eigen::Vector3d(fix.latitude_deg, fix.longitude_deg, fix.altitude_m), *fusion.utm_zone());
    output << "# gps " << std::setprecision(6) << 1e-9 * static_cast<double>(fix.timestamp_ns)
           << std::setprecision(4) << " " << utm.x() << " " << utm.y() << " " << utm.z() << " "
           << fix.satellite_count << "\n";
  }
  std::cout << "wrote " << pose_count << " poses to " << output_path << " (UTM zone "
            << *fusion.utm_zone() << ")\n"
            << "gps horizontal innovation (m) over " << gps_horizontal_innovation.size()
            << " fixes: median " << Percentile(gps_horizontal_innovation, 0.5) << ", p95 "
            << Percentile(gps_horizontal_innovation, 0.95) << ", max "
            << Percentile(gps_horizontal_innovation, 1.0) << "\n"
            << "final bias: gyroscope " << fusion.gyroscope_bias().transpose() << ", accelerometer "
            << fusion.accelerometer_bias().transpose() << "\n";
  return 0;
}

}  // namespace
}  // namespace mapmind::fusion

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: " << argv[0] << " <record> <output.tum>\n";
    return 1;
  }
  return mapmind::fusion::Run(argv[1], argv[2]);
}
