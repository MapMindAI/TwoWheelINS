// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

// Runs InsFusion on a binary or text record and writes the IMU pose in UTM (easting,
// northing, altitude) for every IMU sample after initialization, in TUM format: t_s x y z qx qy qz
// qw. A leading "# UTM zone <number><N|S>" comment lets tools/trajectory_viewer.html put it on a
// map; trailing "# gps t_s easting_m northing_m altitude_m satellites" comments carry the record's
// GPS fixes for the viewer to overlay.
//
// Usage: ins_fusion_main [--constrain_vertical_velocity] [--forward=x,y,z] <record> <output.tum>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>
#include <Eigen/Core>

#include "common/math/geodesy.h"
#include "common/reader/binary_record_reader.h"
#include "common/reader/txt_record_reader.h"
#include "fusion/ins_fusion.h"

DEFINE_bool(constrain_vertical_velocity, false,
            "level-road prior: near-zero world vertical velocity while moving");
DEFINE_string(forward, "1,0,0",
              "direction of travel in the IMU frame, x,y,z (MOKUKU II, mounted X right: 0,1,0)");

namespace mapmind::fusion {
namespace {

double Percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  return values[static_cast<size_t>(fraction * static_cast<double>(values.size() - 1))];
}

// Reads a binary record (doc/binary_record_format.md) or, without its header, a text record.
std::optional<reader::TxtRecord> ReadRecord(const std::string& path) {
  if (!reader::IsBinaryRecord(path)) {
    std::optional<reader::TxtRecord> record = reader::ReadTxtRecord(path);
    if (record) {
      std::cout << "text record, skipped lines " << record->skipped_lines << "\n";
    }
    return record;
  }
  std::optional<reader::BinaryRecord> binary = reader::ReadBinaryRecord(path);
  if (!binary) {
    return std::nullopt;
  }
  std::cout << "binary record, crc errors " << binary->crc_errors << ", skipped bytes "
            << binary->skipped_bytes << "\n";
  return reader::TxtRecord{std::move(binary->imu), std::move(binary->gps),
                           std::move(binary->wheel_speed), 0};
}

std::optional<Eigen::Vector3d> ParseVector(const std::string& text) {
  Eigen::Vector3d vector;
  char comma0 = 0;
  char comma1 = 0;
  std::istringstream stream(text);
  stream >> vector.x() >> comma0 >> vector.y() >> comma1 >> vector.z();
  if (stream.fail() || comma0 != ',' || comma1 != ',' || vector.norm() == 0.0) {
    return std::nullopt;
  }
  return vector.normalized();
}

int Run(const std::string& record_path, const std::string& output_path) {
  std::optional<reader::TxtRecord> record = ReadRecord(record_path);
  if (!record) {
    std::cerr << "cannot read " << record_path << "\n";
    return 1;
  }
  const int removed_imu = reader::RemoveCorruptImuSamples(&record->imu);
  std::cout << "imu " << record->imu.size() << " (removed " << removed_imu << " corrupt), gps "
            << record->gps.size() << ", wheel speed " << record->wheel_speed.size() << "\n";

  std::ofstream output(output_path);
  if (!output.is_open()) {
    std::cerr << "cannot write " << output_path << "\n";
    return 1;
  }
  output << std::fixed;

  // Merge the streams by timestamp; a measurement is applied before the IMU sample it precedes.
  InsFusionOptions options = ins_fusion_default_options();
  options.constrain_vertical_velocity = FLAGS_constrain_vertical_velocity;
  const std::optional<Eigen::Vector3d> forward = ParseVector(FLAGS_forward);
  if (!forward) {
    std::cerr << "bad --forward " << FLAGS_forward << "\n";
    return 1;
  }
  Eigen::Map<Eigen::Vector3d>(options.forward) = *forward;
  InsFusion fusion;
  ins_fusion_init(&fusion, &options);
  const std::vector<sensor::GpsData>& gps = record->gps;
  const std::vector<sensor::WheelSpeedData>& wheel_speed = record->wheel_speed;
  size_t gps_index = 0;
  size_t wheel_index = 0;
  std::vector<double> gps_horizontal_innovation;
  int pose_count = 0;
  for (const sensor::ImuData& imu : record->imu) {
    for (; gps_index < gps.size() && gps[gps_index].timestamp_ns < imu.timestamp_ns; ++gps_index) {
      const sensor::GpsData& fix = gps[gps_index];
      const InsFusionGps fusion_fix = {fix.timestamp_ns, fix.latitude_deg, fix.longitude_deg,
                                       fix.altitude_m, fix.satellite_count};
      double innovation[3];
      if (ins_fusion_add_gps(&fusion, &fusion_fix, innovation)) {
        gps_horizontal_innovation.push_back(std::hypot(innovation[0], innovation[1]));
      }
    }
    for (; wheel_index < wheel_speed.size() &&
           wheel_speed[wheel_index].timestamp_ns < imu.timestamp_ns;
         ++wheel_index) {
      ins_fusion_add_wheel_speed(&fusion, wheel_speed[wheel_index].timestamp_ns,
                                 wheel_speed[wheel_index].speed_mps);
    }
    ins_fusion_add_imu(&fusion, imu.timestamp_ns, imu.linear_acceleration.data(),
                       imu.angular_velocity.data());
    if (!fusion.initialized) {
      continue;
    }
    if (pose_count == 0) {
      output << "# UTM zone " << fusion.utm_zone << (gps.front().latitude_deg < 0 ? "S" : "N")
             << " (WGS84), t_s easting_m northing_m altitude_m qx qy qz qw\n";
    }
    double position[3];
    double rotation[4];  // w x y z
    ins_fusion_pose(&fusion, position, rotation);
    output << std::setprecision(6) << 1e-9 * static_cast<double>(imu.timestamp_ns)
           << std::setprecision(4) << " " << position[0] << " " << position[1] << " " << position[2]
           << std::setprecision(7) << " " << rotation[1] << " " << rotation[2] << " " << rotation[3]
           << " " << rotation[0] << "\n";
    ++pose_count;
  }

  if (!fusion.initialized) {
    std::cerr << (gps.empty() ? "record has no GPS fixes\n"
                              : "vehicle never moves fast enough to initialize heading\n");
    return 1;
  }
  for (const sensor::GpsData& fix : gps) {
    if (fix.latitude_deg == 0.0 && fix.longitude_deg == 0.0) {
      continue;
    }
    const double geodetic[3] = {fix.latitude_deg, fix.longitude_deg, fix.altitude_m};
    Eigen::Vector3d utm;
    geodetic_to_utm(geodetic, fusion.utm_zone, utm.data());
    output << "# gps " << std::setprecision(6) << 1e-9 * static_cast<double>(fix.timestamp_ns)
           << std::setprecision(4) << " " << utm.x() << " " << utm.y() << " " << utm.z() << " "
           << fix.satellite_count << "\n";
  }
  std::cout << "wrote " << pose_count << " poses to " << output_path << " (UTM zone "
            << fusion.utm_zone << ")\n"
            << "gps horizontal innovation (m) over " << gps_horizontal_innovation.size()
            << " fixes: median " << Percentile(gps_horizontal_innovation, 0.5) << ", p95 "
            << Percentile(gps_horizontal_innovation, 0.95) << ", max "
            << Percentile(gps_horizontal_innovation, 1.0) << "\n"
            << "final bias: gyroscope "
            << Eigen::Map<const Eigen::Vector3d>(fusion.filter.gyroscope_bias).transpose()
            << ", accelerometer "
            << Eigen::Map<const Eigen::Vector3d>(fusion.filter.accelerometer_bias).transpose()
            << "\n";
  return 0;
}

}  // namespace
}  // namespace mapmind::fusion

int main(int argc, char** argv) {
  gflags::SetUsageMessage(
      "[--constrain_vertical_velocity] [--forward=x,y,z] <record> <output.tum>");
  gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);
  if (argc != 3) {
    gflags::ShowUsageWithFlagsRestrict(argv[0], "fusion/ins_fusion_main");
    return 1;
  }
  return mapmind::fusion::Run(argv[1], argv[2]);
}
