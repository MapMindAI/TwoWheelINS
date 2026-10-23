load("@rules_cc//cc:defs.bzl", "cc_library")

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
    hdrs = ["covariance.h"],
    deps = [
        "@glog",
        "@eigen",
    ],
)

cc_library(
    name = "covariance_test_utils",
    testonly = True,
    hdrs = ["covariance_test_utils.h"],
    deps = [":covariance"],
)
