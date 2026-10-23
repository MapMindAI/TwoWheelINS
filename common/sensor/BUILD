load("@rules_cc//cc:defs.bzl", "cc_library")

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
