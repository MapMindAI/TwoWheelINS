load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sophus_utils",
    srcs = ["sophus_utils.cc"],
    hdrs = ["sophus_utils.h"],
    deps = [
        "@eigen",
        "@sophus",
    ],
)

cc_library(
    name = "covariance",
    srcs = ["covariance.cc"],
    hdrs = ["covariance.h"],
    deps = [
        ":sophus_utils",
        "@glog",
        "@eigen",
    ],
)

cc_test(
    name = "covariance_test",
    srcs = ["covariance_test.cc"],
    deps = [
        ":covariance",
        ":covariance_test_utils",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "covariance_test_utils",
    testonly = True,
    hdrs = ["covariance_test_utils.h"],
    deps = [":covariance"],
)
