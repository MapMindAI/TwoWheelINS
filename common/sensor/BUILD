load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "imu_data",
    hdrs = ["imu_data.h"],
    deps = ["@eigen"],
)

cc_library(
    name = "imu_intrinsics",
    srcs = ["imu_intrinsics.cc"],
    hdrs = ["imu_intrinsics.h"],
    deps = ["@eigen"],
)

cc_library(
    name = "gps_data",
    srcs = ["gps_data.cc"],
    hdrs = ["gps_data.h"],
    deps = ["@eigen"],
)

cc_test(
    name = "gps_data_test",
    srcs = ["gps_data_test.cc"],
    deps = [
        ":gps_data",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "wheel_speed_data",
    hdrs = ["wheel_speed_data.h"],
)
