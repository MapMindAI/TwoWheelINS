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
    hdrs = ["gps_data.h"],
)

# C11
cc_library(
    name = "gps_accuracy",
    srcs = ["gps_accuracy.c"],
    hdrs = ["gps_accuracy.h"],
)

cc_test(
    name = "gps_accuracy_test",
    srcs = ["gps_accuracy_test.cc"],
    deps = [
        ":gps_accuracy",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "barometer_data",
    hdrs = ["barometer_data.h"],
)

cc_library(
    name = "wheel_speed_data",
    hdrs = ["wheel_speed_data.h"],
)
