load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ins_fusion",
    srcs = ["ins_fusion.cc"],
    hdrs = ["ins_fusion.h"],
    deps = [
        "//common/math:geodesy",
        "//common/sensor:gps_data",
        "//common/sensor:imu_data",
        "//common/sensor:wheel_speed_data",
        "//ikfom/backend:ikf_ins",
        "@sophus",
    ],
)

cc_binary(
    name = "ins_fusion_main",
    srcs = ["ins_fusion_main.cc"],
    deps = [
        ":ins_fusion",
        "//common/math:geodesy",
        "//common/reader:txt_record_reader",
    ],
)

cc_test(
    name = "ins_fusion_test",
    srcs = ["ins_fusion_test.cc"],
    deps = [
        ":ins_fusion",
        "//common/math:geodesy",
        "@googletest//:gtest_main",
    ],
)
