load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "covariance_transformation",
    srcs = ["covariance_transformation.cc"],
    hdrs = ["covariance_transformation.h"],
    deps = [
        "//common/math:sophus_utils",
        "@glog",
        "@eigen",
    ],
)

cc_test(
    name = "covariance_transformation_test",
    srcs = ["covariance_transformation_test.cc"],
    deps = [
        ":covariance_transformation",
        "//common/math:covariance_test_utils",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "utils",
    srcs = ["utils.cc"],
    hdrs = ["utils.h"],
    deps = [
        "//common/sensor:imu_data",
        "//common/sensor:imu_intrinsics",
        "//ikfom/toolkit:ikfom_toolkit",
        "@sophus",
    ],
)
