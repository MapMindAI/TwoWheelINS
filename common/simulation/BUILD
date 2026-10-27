load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

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

cc_library(
    name = "gps_noise",
    srcs = ["gps_noise.cc"],
    hdrs = ["gps_noise.h"],
    deps = [
        "//common/math:geodesy",
        "//common/reader:benchmark_dataset_reader",
        "//common/sensor:gps_data",
        "@eigen",
    ],
)

cc_test(
    name = "gps_noise_test",
    srcs = ["gps_noise_test.cc"],
    deps = [
        ":gps_noise",
        "//common/math:geodesy",
        "//common/sensor:gps_data",
        "@googletest//:gtest_main",
    ],
)
