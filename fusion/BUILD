load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kfcore_ins",
    srcs = ["kfcore_ins.cc"],
    hdrs = ["kfcore_ins.h"],
    deps = [
        "//third_party/kfcore",
        "@eigen",
        "@sophus",
    ],
)

cc_test(
    name = "kfcore_ins_test",
    srcs = ["kfcore_ins_test.cc"],
    deps = [
        ":kfcore_ins",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "ins_fusion",
    srcs = ["ins_fusion.cc"],
    hdrs = ["ins_fusion.h"],
    deps = [
        ":kfcore_ins",
        "//common/math:geodesy",
        "//common/sensor:gps_data",
        "//common/sensor:imu_data",
        "//common/sensor:wheel_speed_data",
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
        "@gflags",
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

cc_binary(
    name = "run_dataset_main",
    srcs = ["run_dataset_main.cc"],
    deps = [
        ":ins_fusion",
        "//common/math:geodesy",
        "//common/reader:benchmark_dataset_reader",
        "//common/simulation:gps_noise",
        "@gflags",
    ],
)
