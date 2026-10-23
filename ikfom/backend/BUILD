load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = ["common.cc"],
    hdrs = ["common.h"],
    deps = [
        "//common/math:sophus_utils",
        "//ikfom/common:utils",
    ],
)

cc_library(
    name = "ikf_ins",
    srcs = ["ikf_ins.cc"],
    hdrs = ["ikf_ins.h"],
    deps = [":common"],
)

cc_test(
    name = "ikf_ins_test",
    srcs = ["ikf_ins_test.cc"],
    deps = [
        ":ikf_ins",
        "@googletest//:gtest_main",
    ],
)
