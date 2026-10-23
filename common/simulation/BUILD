load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "imu_simulation",
    srcs = ["imu_simulation.cc"],
    hdrs = ["imu_simulation.h"],
    deps = [
        "//common/sensor:imu_data",
        "@eigen",
        "@sophus",
    ],
)
